#include <sequence_manager.hpp>

namespace cango_master
{

  SequenceManager::SequenceManager(rclcpp::Node *node,
                                   const std::string &yaml_path)
      : node_(node)
  {
    coordinate_converter.load_semantic_map(yaml_path);
    planner_service_client_ = node_->create_client<nav_msgs::srv::GetPlan>("/get_plan");
    navigation_action_client_ = rclcpp_action::create_client<FollowPath>(node_, "/follow_path");
  }

  void SequenceManager::reset() {}

  void SequenceManager::update_status() { prev_status = new_status; }

  void SequenceManager::search_path(std::vector<std::string> waypoint_list)
  {
    path_list.clear();
    for (const auto &wp : waypoint_list)
    {
      Point pt;
      coordinate_converter.id2pcd(wp, pt);
      path_list.push_back(pt);
    }
  }

  geometry_msgs::msg::PoseStamped SequenceManager::get_current_pose(
      const Point &current_location)
  {
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header.frame_id = "map";
    current_pose.header.stamp = node_->now();

    try
    {
      current_pose.pose.position.x = current_location.x;
      current_pose.pose.position.y = current_location.y;
      current_pose.pose.position.z = 0.0;
      current_pose.pose.orientation.w = 1.0;
      if (current_location.x == 0.0 && current_location.y == 0.0)
      {
        throw std::runtime_error("Current location is (0,0), likely an error.");
      }
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(node_->get_logger(), "cannot find robot_location: %s",
                  ex.what());
    }

    return current_pose;
  }

bool SequenceManager::create_full_path(const std::vector<Point> &path_list,
                                       const Point &current_location)
{
  // 1. 웨이포인트 체크
  if (path_list.empty())
  {
    RCLCPP_WARN(node_->get_logger(), "Need at least 1 waypoint to create path.");
    return false;
  }

 if (!planner_service_client_) {
      RCLCPP_ERROR(node_->get_logger(), "Planner client is NULL!");
      return false;
  }

  // 2. 서비스가 준비되었는지 확인 (절대 경로로 다시 시도)
  if (!planner_service_client_->service_is_ready()) {
      RCLCPP_WARN(node_->get_logger(), "Service /get_plan is not ready, waiting...");
      if (!planner_service_client_->wait_for_service(std::chrono::milliseconds(500))) {
          RCLCPP_ERROR(node_->get_logger(), "Planner service (/get_plan) still not available!");
          return false;
      }
  }if (!planner_service_client_) {
      RCLCPP_ERROR(node_->get_logger(), "Planner client is NULL!");
      return false;
  }

  // 2. 서비스가 준비되었는지 확인 (절대 경로로 다시 시도)
  if (!planner_service_client_->service_is_ready()) {
      RCLCPP_WARN(node_->get_logger(), "Service /get_plan is not ready, waiting...");
      if (!planner_service_client_->wait_for_service(std::chrono::milliseconds(500))) {
          RCLCPP_ERROR(node_->get_logger(), "Planner service (/get_plan) still not available!");
          return false;
      }
  }

  // 3. 경로 초기화
  last_generated_path_.poses.clear();
  last_generated_path_.header.frame_id = "map";
  last_generated_path_.header.stamp = node_->now();

  // 4. 전체 지점 리스트 구성 (현재 위치 + 모든 웨이포인트)
  std::vector<geometry_msgs::msg::PoseStamped> all_points;
  geometry_msgs::msg::PoseStamped current_pose = get_current_pose(current_location);
  all_points.push_back(current_pose);

  for (const auto &pt : path_list)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = node_->now();
    p.pose.position.x = pt.x;
    p.pose.position.y = pt.y;
    p.pose.position.z = 0.0;
    p.pose.orientation.w = 1.0;
    all_points.push_back(p);
  }

  // 5. 각 지점 사이의 경로를 Planner에게 요청
  for (size_t i = 0; i < all_points.size() - 1; ++i)
  {
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = all_points[i];
    request->goal = all_points[i + 1];
    request->tolerance = 0.1f;

    // 비동기 요청
    auto result_future = planner_service_client_->async_send_request(request);

    // 서비스 응답 대기 (최대 1초 타임아웃 설정으로 노드 멈춤 방지)
    if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                           result_future,
                                           std::chrono::seconds(1)) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = result_future.get();

      if (response && !response->plan.poses.empty())
      {
        // 경로 연결 시 중복되는 지점 제거 (이전 구간의 끝점 == 현재 구간의 시작점)
        if (!last_generated_path_.poses.empty())
        {
          last_generated_path_.poses.pop_back();
        }

        last_generated_path_.poses.insert(last_generated_path_.poses.end(),
                                          response->plan.poses.begin(),
                                          response->plan.poses.end());
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Planner returned an empty plan between point %zu and %zu", i, i + 1);
        return false;
      }
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Service call timeout or failed for plan segment %zu", i);
      return false;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Successfully created full path with %zu poses", last_generated_path_.poses.size());
  return true;
}

  bool SequenceManager::path_tracking()
  {

      if (last_generated_path_.poses.empty())
  {
    RCLCPP_WARN(node_->get_logger(), "Generated path is empty.");
    return false;
  }

    // 2. 액션 서버 연결 확인
    if (!navigation_action_client_->wait_for_action_server(
            std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Nav2 Controller Server (FollowPath) not found.");
      return false;
    }

    // 3. 목표(Goal) 설정
    auto goal_msg = FollowPath::Goal();
    goal_msg.path = last_generated_path_;
    goal_msg.controller_id = "FollowPath"; // DWA 설정 이름

    // 4. 액션 전송
    RCLCPP_INFO(node_->get_logger(), "Sending path to Nav2 DWA Controller...");
    navigation_action_client_->async_send_goal(goal_msg);

    return true; // 요청 성공
  }

  void SequenceManager::check_sound_trigger(const Point &current_location)
  {
    int detected_trigger = 0;
    for (size_t i = 0; i < path_list.size(); ++i)
    {
      double dist = std::hypot(current_location.x - path_list[i].x,
                               current_location.y - path_list[i].y);

      if (i == path_list.size() - 1)
      { // 목적지 관련
        if (dist < 0.5)
          detected_trigger = 3;
        else if (dist < 5.0)
          detected_trigger = 2;
      }
      else if (dist < 0.5)
      { // 경유지
        detected_trigger = 1;
        break; // 하나라도 걸리면 중단
      }
    }
    sound_trigger = detected_trigger;
  }
} // namespace cango_master