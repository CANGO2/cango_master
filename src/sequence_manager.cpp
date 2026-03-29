#include <sequence_manager.hpp>

namespace cango_master {

SequenceManager::SequenceManager(rclcpp::Node* node,
                                 const std::string& yaml_path)
    : node_(node) {
  coordinate_converter.load_semantic_map(yaml_path);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

SequenceManager::~SequenceManager() {}

void SequenceManager::reset() {}

void SequenceManager::change_status(enum task_name task_num, int& act) {
  if task_num
    == task_name::LLM { now_status.use_llm = act; }
}

SequenceManager::update_status() { prev_status = update_status; }

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
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "cannot find robot_location: %s",
                ex.what());
  }

  return current_pose;
}

void SequenceManager::create_full_path(const std::vector<Point> path_list,
                                       const Point& current_location) {
  if (path_list.size() < 2) {
    RCLCPP_WARN(node_->get_logger(), "need more than 2 path points");
    return;
  }

  while (!planner_service_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) return;
    RCLCPP_INFO(node_->get_logger(), "waitting for Planner service...");
  }

  last_generated_path_.poses.clear();
  last_generated_path_.header.frame_id = "map";
  last_generated_path_.header.stamp = node_->now();

  geometry_msgs::msg::PoseStamped current_pose =
      get_current_pose(current_location);

  // make waypoints list
  std::vector<geometry_msgs::msg::PoseStamped> all_points;
  all_points.push_back(current_pose);
  for (const auto& pt : path_list) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = pt.x;
    p.pose.position.y = pt.y;
    p.pose.orientation.w = 1.0;
    all_points.push_back(p);
  }

  for (size_t i = 0; i < all_points.size() - 1; ++i) {
    auto request = std::make_shared<nav2_msgs::srv::GetPlan::Request>();
    request->start = all_points[i];
    request->goal = all_points[i + 1];
    request->planner_id = "GridBased";

    // 서비스 호출 (결과 대기)
    auto result = planner_service_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = result.get();
      // 경로 합치기 (중복 점 제거 로직 포함)
      if (!last_generated_path_.poses.empty() &&
          !response->path.poses.empty()) {
        last_generated_path_.poses.pop_back();
      }
      last_generated_path_.poses.insert(last_generated_path_.poses.end(),
                                        response->path.poses.begin(),
                                        response->path.poses.end());
    }
  }

  RCLCPP_INFO(node_->get_logger(), "created_full_path with %zu poses",
              last_generated_path_.poses.size());
}

void SequenceManager::check_sound_trigger(const Point& current_location) {
  for (size_t i = 0; i < path_list.size(); ++i) {
    double dist = std::hypot(current_location.x - path_list[i].x,
                             current_location.y - path_list[i].y);
    if (dist < 0.5) {  // 50cm 이내
      sound_trigger = 1;
    }
    if (i == path_list.size() - 1 && dist < 5.0) {
      sound_trigger = 2;  // 목적지 근처
    }
    if (i == path_list.size() - 1 && dist < 0.5) {
      sound_trigger = 3;  // 목적지 도착
    } else {
      sound_trigger = 0;  // None
    }
  }
}
}  // namespace cango_master