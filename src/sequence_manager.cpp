#include <sequence_manager.hpp>

#include <chrono>
#include <cmath>
#include <future>

namespace cango_master
{

  SequenceManager::SequenceManager(rclcpp::Node *node,
                                   const std::string &yaml_path)
      : node_(node)
  {
    coordinate_converter.load_semantic_map(yaml_path);

    action_callback_group_ =
        node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    compute_path_client_ =
        rclcpp_action::create_client<ComputePathToPose>(
            node_,
            "/compute_path_to_pose",
            action_callback_group_);

    navigation_action_client_ =
        rclcpp_action::create_client<FollowPath>(
            node_,
            "/follow_path",
            action_callback_group_);
    full_path_pub_ =
        node_->create_publisher<nav_msgs::msg::Path>("/cango/full_path", 10);
  }

  void SequenceManager::reset() {}

  void SequenceManager::update_status()
  {
    prev_status = new_status;
  }

  void SequenceManager::set_path_orientations(nav_msgs::msg::Path &path)
  {
    if (path.poses.size() < 2)
      return;

    for (size_t i = 0; i < path.poses.size() - 1; ++i)
    {
      double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
      double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
      double yaw = std::atan2(dy, dx);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      path.poses[i].pose.orientation = tf2::toMsg(q);
    }

    path.poses.back().pose.orientation =
        path.poses[path.poses.size() - 2].pose.orientation;
  }

  void SequenceManager::search_path(std::vector<std::string> waypoint_list)
  {
    path_list.clear();

    for (const auto &wp : waypoint_list)
    {
      Point pt;
      coordinate_converter.id2pcd(wp, pt);
      path_list.push_back(pt);

      RCLCPP_INFO(
          node_->get_logger(),
          "Waypoint %s -> x=%.3f, y=%.3f",
          wp.c_str(),
          pt.x,
          pt.y);
    }
  }

  geometry_msgs::msg::PoseStamped SequenceManager::get_current_pose(
      const Point &current_location)
  {
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose.header.frame_id = "map";
    current_pose.header.stamp = node_->now();

    current_pose.pose.position.x = current_location.x;
    current_pose.pose.position.y = current_location.y;
    current_pose.pose.position.z = 0.0;
    current_pose.pose.orientation.w = 1.0;

    if (current_location.x == 0.0 && current_location.y == 0.0)
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "Current location is (0,0). This may be wrong.");
    }

    return current_pose;
  }

  bool SequenceManager::create_full_path(const std::vector<Point> &path_list,
                                         const Point &current_location)
  {
    RCLCPP_INFO(node_->get_logger(), "create_full_path() called");
    RCLCPP_INFO(node_->get_logger(), "Number of waypoints: %zu", path_list.size());
    RCLCPP_INFO(
        node_->get_logger(),
        "Current location: x=%.3f, y=%.3f",
        current_location.x,
        current_location.y);

    if (path_list.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "Need at least 1 waypoint to create path.");
      return false;
    }

    if (!compute_path_client_)
    {
      RCLCPP_ERROR(node_->get_logger(), "ComputePathToPose client is NULL.");
      return false;
    }

    if (!compute_path_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Action server /compute_path_to_pose is not available.");
      return false;
    }

    last_generated_path_.poses.clear();
    last_generated_path_.header.frame_id = "map";
    last_generated_path_.header.stamp = node_->now();

    std::vector<geometry_msgs::msg::PoseStamped> all_points;

    geometry_msgs::msg::PoseStamped current_pose =
        get_current_pose(current_location);

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

    for (size_t i = 0; i < all_points.size(); ++i)
    {
      RCLCPP_INFO(
          node_->get_logger(),
          "Path point %zu: x=%.3f, y=%.3f",
          i,
          all_points[i].pose.position.x,
          all_points[i].pose.position.y);
    }

    for (size_t i = 0; i < all_points.size() - 1; ++i)
    {
      RCLCPP_INFO(
          node_->get_logger(),
          "Requesting segment %zu: start(%.3f, %.3f) -> goal(%.3f, %.3f)",
          i,
          all_points[i].pose.position.x,
          all_points[i].pose.position.y,
          all_points[i + 1].pose.position.x,
          all_points[i + 1].pose.position.y);

      auto goal_msg = ComputePathToPose::Goal();

      goal_msg.start = all_points[i];
      goal_msg.goal = all_points[i + 1];
      goal_msg.planner_id = "GridBased";
      goal_msg.use_start = true;

      auto goal_handle_future =
          compute_path_client_->async_send_goal(goal_msg);

      if (goal_handle_future.wait_for(std::chrono::seconds(5)) !=
          std::future_status::ready)
      {
        RCLCPP_ERROR(node_->get_logger(), "ComputePathToPose goal send timeout.");
        return false;
      }

      auto goal_handle = goal_handle_future.get();

      if (!goal_handle)
      {
        RCLCPP_ERROR(node_->get_logger(), "ComputePathToPose goal was rejected.");
        return false;
      }

      RCLCPP_INFO(node_->get_logger(), "ComputePathToPose goal accepted.");

      auto result_future =
          compute_path_client_->async_get_result(goal_handle);

      if (result_future.wait_for(std::chrono::seconds(10)) !=
          std::future_status::ready)
      {
        RCLCPP_ERROR(node_->get_logger(), "ComputePathToPose result timeout.");
        return false;
      }

      auto wrapped_result = result_future.get();

      if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED)
      {
        RCLCPP_ERROR(
            node_->get_logger(),
            "ComputePathToPose failed. Result code: %d",
            static_cast<int>(wrapped_result.code));
        return false;
      }

      if (!wrapped_result.result)
      {
        RCLCPP_ERROR(node_->get_logger(), "ComputePathToPose result is NULL.");
        return false;
      }

      auto path = wrapped_result.result->path;

      if (path.poses.empty())
      {
        RCLCPP_ERROR(node_->get_logger(), "Computed path segment is empty.");
        return false;
      }

      if (!last_generated_path_.poses.empty())
      {
        last_generated_path_.poses.pop_back();
      }

      last_generated_path_.poses.insert(
          last_generated_path_.poses.end(),
          path.poses.begin(),
          path.poses.end());

      RCLCPP_INFO(
          node_->get_logger(),
          "Segment %zu added. Total poses: %zu",
          i,
          last_generated_path_.poses.size());
    }

    last_generated_path_.header.frame_id = "map";
    last_generated_path_.header.stamp = node_->now();

    for (auto &pose : last_generated_path_.poses)
    {
      pose.header.frame_id = "map";
      pose.header.stamp = last_generated_path_.header.stamp;
    }
    set_path_orientations(last_generated_path_);

    full_path_pub_->publish(last_generated_path_);

    RCLCPP_INFO(
        node_->get_logger(),
        "Published full path to /cango/full_path. poses=%zu",
        last_generated_path_.poses.size());

    auto &poses = last_generated_path_.poses;

    double first_x = poses.front().pose.position.x;
    double first_y = poses.front().pose.position.y;
    double last_x = poses.back().pose.position.x;
    double last_y = poses.back().pose.position.y;

    double d_first = hypot(current_location.x - first_x, current_location.y - first_y);
    double d_last = hypot(current_location.x - last_x, last_y);

    RCLCPP_INFO(node_->get_logger(),
                "[PATH DEBUG] robot=(%.2f, %.2f), first=(%.2f, %.2f), last=(%.2f, %.2f), d_first=%.2f, d_last=%.2f, size=%zu",
                current_location.x, current_location.y, first_x, first_y, last_x, last_y, d_first, d_last, poses.size());

    return true;
  }

  bool SequenceManager::path_tracking()
  {
    RCLCPP_INFO(
        node_->get_logger(),
        "iiiinto path trraackinnnng.");

    if (last_generated_path_.poses.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "Generated path is empty.");
      return false;
    }

    if (!navigation_action_client_)
    {
      RCLCPP_ERROR(node_->get_logger(), "FollowPath action client is NULL.");
      return false;
    }

    if (!navigation_action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "Action server /follow_path is not available.");
      return false;
    }

    // =========================================================================
    auto stamp = node_->now();

    last_generated_path_.header.stamp = stamp;
    for (auto &pose : last_generated_path_.poses)
    {
      pose.header.frame_id = "map";
      pose.header.stamp = stamp;
    }
    // =========================================================================

    auto goal_msg = FollowPath::Goal();

    goal_msg.path = last_generated_path_;
    goal_msg.controller_id = "FollowPath";

    RCLCPP_INFO(
        node_->get_logger(),
        "Sending path to /follow_path. poses=%zu",
        last_generated_path_.poses.size());

    navigation_action_client_->async_send_goal(goal_msg);

    return true;
  }

  void SequenceManager::check_sound_trigger(const Point &current_location)
  {
    int detected_trigger = 0;

    for (size_t i = 0; i < path_list.size(); ++i)
    {
      double dist = std::hypot(
          current_location.x - path_list[i].x,
          current_location.y - path_list[i].y);

      if (i == path_list.size() - 1)
      {
        if (dist < 0.5)
        {
          detected_trigger = 3;
        }
        else if (dist < 5.0)
        {
          detected_trigger = 2;
        }
      }
      else if (dist < 0.5)
      {
        detected_trigger = 1;
        break;
      }
    }

    sound_trigger = detected_trigger;
  }

} // namespace cango_master