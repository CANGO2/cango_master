#include <sequence_manager.hpp>

namespace cango_master {

SequenceManager::SequenceManager(rclcpp::Node* node,
                                 const std::string& yaml_path)
    : node_(node) {
  coordinate_converter.load_semantic_map(yaml_path);
  planner_service_client_ = node_->create_client<nav_msgs::srv::GetPlan>("get_plan");
  navigation_action_client_ = rclcpp_action::create_client<FollowPath>(node_, "follow_path");
  assisted_teleop_client_ = rclcpp_action::create_client<nav2_msgs::action::AssistedTeleop>(node_, "assisted_teleop");
}

void SequenceManager::reset() {}

void SequenceManager::update_status() { prev_status = new_status; }

void SequenceManager::search_path(std::vector<std::string> waypoint_list) {
  path_list.clear();
  for (const auto& wp : waypoint_list) {
    Point pt;
    coordinate_converter.id2pcd(wp, pt);
    path_list.push_back(pt);
  }
}

geometry_msgs::msg::PoseStamped SequenceManager::get_current_pose(
    const Point& current_location) {
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header.frame_id = "map";
  current_pose.header.stamp = node_->now();

  try {
    current_pose.pose.position.x = current_location.x;
    current_pose.pose.position.y = current_location.y;
    current_pose.pose.position.z = 0.0;
    current_pose.pose.orientation.w = 1.0;
    if (current_location.x == 0.0 && current_location.y == 0.0) {
      throw std::runtime_error("Current location is (0,0), likely an error.");
    }
  } catch (const std::exception& ex) {
    RCLCPP_WARN(node_->get_logger(), "cannot find robot_location: %s",
                ex.what());
  }

  return current_pose;
}

bool SequenceManager::create_full_path(const std::vector<Point>& path_list,
                                       const Point& current_location) {
  if (path_list.size() < 1) {
    RCLCPP_WARN(node_->get_logger(),
                "Need at least 1 waypoint to create path.");
    return false;
  }

  while (!planner_service_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) return false;
    RCLCPP_INFO(node_->get_logger(),
                "Waiting for Planner service (GetPlan)...");
  }

  last_generated_path_.poses.clear();
  last_generated_path_.header.frame_id = "map";
  last_generated_path_.header.stamp = node_->now();

  std::vector<geometry_msgs::msg::PoseStamped> all_points;
  geometry_msgs::msg::PoseStamped current_pose =
      get_current_pose(current_location);
  all_points.push_back(current_pose);

  for (const auto& pt : path_list) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = node_->now();
    p.pose.position.x = pt.x;
    p.pose.position.y = pt.y;
    p.pose.position.z = 0.0;
    p.pose.orientation.w = 1.0;
    all_points.push_back(p);
  }

  for (size_t i = 0; i < all_points.size() - 1; ++i) {
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = all_points[i];
    request->goal = all_points[i + 1];
    request->tolerance = 0.1f;  // 필요시 허용 오차 설정

    // 서비스 호출 (비동기 요청 후 대기)
    auto result_future = planner_service_client_->async_send_request(request);

    // node_가 rclcpp::Node* 일 경우 get_node_base_interface() 사용
    if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                           result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = result_future.get();

      if (response && !response->plan.poses.empty()) {
        // 이전 경로의 마지막 점과 새 경로의 시작점이 겹치면 마지막 점 제거
        if (!last_generated_path_.poses.empty()) {
          last_generated_path_.poses.pop_back();
        }

        last_generated_path_.poses.insert(last_generated_path_.poses.end(),
                                          response->plan.poses.begin(),
                                          response->plan.poses.end());
      } else {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to get plan between point %zu and %zu", i, i + 1);
        return false;
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(),
                   "Service call failed for plan segment %zu", i);
      return false;
    }
  }

  RCLCPP_INFO(node_->get_logger(),
              "Successfully created full path with %zu poses",
              last_generated_path_.poses.size());
  return true;
}

bool SequenceManager::path_tracking(const std::vector<Point>& path_list) {
  if (path_list.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Path list is empty.");
    return false;
  }

  // 1. Point 벡터를 nav_msgs::msg::Path 형식으로 변환
  nav_msgs::msg::Path nav2_path;
  nav2_path.header.frame_id = "map";
  nav2_path.header.stamp = node_->now();

  for (const auto& pt : path_list) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = nav2_path.header;
    pose.pose.position.x = pt.x;
    pose.pose.position.y = pt.y;
    pose.pose.orientation.w = 1.0;  // 기본 방향
    nav2_path.poses.push_back(pose);
  }

  // 2. 액션 서버 연결 확인
  if (!navigation_action_client_->wait_for_action_server(
          std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Nav2 Controller Server (FollowPath) not found.");
    return false;
  }

  // 3. 목표(Goal) 설정
  auto goal_msg = FollowPath::Goal();
  goal_msg.path = nav2_path;
  goal_msg.controller_id = "FollowPath";  // DWA 설정 이름

  // 4. 액션 전송
  RCLCPP_INFO(node_->get_logger(), "Sending path to Nav2 DWA Controller...");
  navigation_action_client_->async_send_goal(goal_msg);

  return true;  // 요청 성공
}
bool SequenceManager::send_assisted_teleop() {
    if (!assisted_teleop_client_->action_server_is_ready()) {
        return false;
    }
    auto goal_msg = nav2_msgs::action::AssistedTeleop::Goal();  
    
    // 1. 에러 발생했던 필드 제거 및 시간 설정 (예: 0.5초 동안 활성화)
    goal_msg.time_allowance.sec = 3600;

    // 3. 액션 전송
    assisted_teleop_client_->async_send_goal(goal_msg);
    return true;
}

void SequenceManager::cancel_assisted_teleop() {
    assisted_teleop_client_->async_cancel_all_goals();
}

void SequenceManager::check_sound_trigger(const Point& current_location) {
  for (size_t i = 0; i < path_list.size(); ++i) {
    double dist = std::hypot(current_location.x - path_list[i].x,
                             current_location.y - path_list[i].y);
    if (dist < 0.5) {  // 50cm 이내
      sound_trigger = 1;
    } else if (i == path_list.size() - 1 && dist < 5.0) {
      sound_trigger = 2;  // 목적지 근처
    } else if (i == path_list.size() - 1 && dist < 0.5) {
      sound_trigger = 3;  // 목적지 도착
    } else {
      sound_trigger = 0;  // None
    }
  }
}
}  // namespace cango_master