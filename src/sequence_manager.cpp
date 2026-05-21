#include <sequence_manager.hpp>

#include <chrono>
#include <cmath>
#include <future>
#include <algorithm>
#include <limits>

#include <tf2/LinearMath/Matrix3x3.h>

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

    full_path_pub_ =
        node_->create_publisher<nav_msgs::msg::Path>("/cango/full_path", 10);
  }

  void SequenceManager::reset()
  {
    tracking_active_ = false;
    tracking_index_ = 0;
  }

  void SequenceManager::update_status()
  {
    prev_status = new_status;
  }

  double SequenceManager::normalize_angle(double angle)
  {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;

    while (angle < -M_PI)
      angle += 2.0 * M_PI;

    return angle;
  }

  double SequenceManager::get_yaw_from_pose(
      const geometry_msgs::msg::PoseStamped &pose)
  {
    tf2::Quaternion q(
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    return yaw;
  }

  void SequenceManager::set_path_orientations(nav_msgs::msg::Path &path)
  {
    if (path.poses.size() < 2)
      return;

    const int smooth_window = 4;

    for (size_t i = 0; i < path.poses.size(); ++i)
    {
      int start = std::max<int>(0, static_cast<int>(i) - smooth_window);
      int end = std::min<int>(
          static_cast<int>(path.poses.size()) - 1,
          static_cast<int>(i) + smooth_window);

      double vx = 0.0;
      double vy = 0.0;

      for (int j = start; j < end; ++j)
      {
        double dx =
            path.poses[j + 1].pose.position.x -
            path.poses[j].pose.position.x;

        double dy =
            path.poses[j + 1].pose.position.y -
            path.poses[j].pose.position.y;

        double len = std::hypot(dx, dy);

        if (len < 1e-6)
          continue;

        vx += dx / len;
        vy += dy / len;
      }

      if (std::hypot(vx, vy) < 1e-6)
      {
        if (i > 0)
        {
          path.poses[i].pose.orientation =
              path.poses[i - 1].pose.orientation;
        }

        continue;
      }

      double yaw = std::atan2(vy, vx);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);

      path.poses[i].pose.orientation = tf2::toMsg(q);
    }
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

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, current_location.theta);
    current_pose.pose.orientation = tf2::toMsg(q);

    if (current_location.x == 0.0 && current_location.y == 0.0)
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "Current location is (0,0). This may be wrong.");
    }

    return current_pose;
  }

  bool SequenceManager::create_full_path(
      const std::vector<Point> &path_list,
      const Point &current_location)
  {
    RCLCPP_INFO(node_->get_logger(), "create_full_path() called");

    RCLCPP_INFO(
        node_->get_logger(),
        "Number of waypoints: %zu",
        path_list.size());

    RCLCPP_INFO(
        node_->get_logger(),
        "Current location: x=%.3f, y=%.3f, theta=%.3f",
        current_location.x,
        current_location.y,
        current_location.theta);

    if (path_list.empty())
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "Need at least 1 waypoint to create path.");
      return false;
    }

    if (!compute_path_client_)
    {
      RCLCPP_ERROR(
          node_->get_logger(),
          "ComputePathToPose client is NULL.");
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
        RCLCPP_ERROR(
            node_->get_logger(),
            "ComputePathToPose goal send timeout.");
        return false;
      }

      auto goal_handle = goal_handle_future.get();

      if (!goal_handle)
      {
        RCLCPP_ERROR(
            node_->get_logger(),
            "ComputePathToPose goal was rejected.");
        return false;
      }

      auto result_future =
          compute_path_client_->async_get_result(goal_handle);

      if (result_future.wait_for(std::chrono::seconds(10)) !=
          std::future_status::ready)
      {
        RCLCPP_ERROR(
            node_->get_logger(),
            "ComputePathToPose result timeout.");
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
        RCLCPP_ERROR(
            node_->get_logger(),
            "ComputePathToPose result is NULL.");
        return false;
      }

      auto path = wrapped_result.result->path;

      if (path.poses.empty())
      {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Computed path segment is empty.");
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

    tracking_active_ = false;
    tracking_index_ = 0;

    return true;
  }

  bool SequenceManager::path_tracking()
  {
    if (last_generated_path_.poses.empty())
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "Generated path is empty.");
      return false;
    }

    tracking_active_ = true;
    tracking_index_ = 0;

    RCLCPP_INFO(
        node_->get_logger(),
        "Pure Pursuit tracking started. poses=%zu",
        last_generated_path_.poses.size());

    return true;
  }

  size_t SequenceManager::find_nearest_index(
      const Point &current_location)
  {
    if (last_generated_path_.poses.empty())
      return 0;

    size_t best_idx = tracking_index_;
    double best_dist = std::numeric_limits<double>::max();

    for (size_t i = tracking_index_;
         i < last_generated_path_.poses.size();
         ++i)
    {
      const auto &p = last_generated_path_.poses[i].pose.position;

      double d = std::hypot(
          current_location.x - p.x,
          current_location.y - p.y);

      if (d < best_dist)
      {
        best_dist = d;
        best_idx = i;
      }
    }

    return best_idx;
  }

  size_t SequenceManager::find_lookahead_index(
      size_t nearest_idx,
      const Point &current_location)
  {
    const double lookahead_dist = 1.0;

    size_t best_idx = tracking_index_;

    for (size_t i = tracking_index_;
         i < last_generated_path_.poses.size();
         ++i)
    {
      const auto &p =
          last_generated_path_.poses[i].pose.position;

      double d = std::hypot(
          p.x - current_location.x,
          p.y - current_location.y);

      if (d <= lookahead_dist)
      {
        best_idx = i;
      }
      else
      {
        if (i > tracking_index_)
          break;
      }
    }

    if (best_idx < tracking_index_)
      best_idx = tracking_index_;

    return best_idx;
  }

  geometry_msgs::msg::Twist SequenceManager::update_pure_pursuit_cmd(
      const Point &current_location)
  {
    geometry_msgs::msg::Twist cmd;

    if (!tracking_active_ || last_generated_path_.poses.empty())
    {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      return cmd;
    }

    const double base_linear = 0.12;
    const double angular_gain = 0.9;
    const double max_angular = 0.18;
    const double goal_tolerance = 0.35;
    const double rotate_only_threshold = 0.9;

    const auto &goal =
        last_generated_path_.poses.back().pose.position;

    double goal_dist = std::hypot(
        goal.x - current_location.x,
        goal.y - current_location.y);

    if (goal_dist < goal_tolerance)
    {
      tracking_active_ = false;
      tracking_index_ = 0;

      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;

      RCLCPP_INFO(
          node_->get_logger(),
          "Pure Pursuit goal reached.");

      return cmd;
    }
    size_t nearest_idx = find_nearest_index(current_location);

    if (nearest_idx > tracking_index_)
    {
      tracking_index_ = nearest_idx;
    }

    size_t target_idx =
        find_lookahead_index(tracking_index_, current_location);

    const auto &target =
        last_generated_path_.poses[target_idx].pose.position;

    double target_angle = std::atan2(
        target.y - current_location.y,
        target.x - current_location.x);

    double heading_error =
        normalize_angle(target_angle - current_location.theta);

    double angular =
        angular_gain * heading_error;

    angular =
        std::clamp(angular, -max_angular, max_angular);

    double linear = base_linear;

    if (std::fabs(heading_error) > rotate_only_threshold)
    {
      linear = 0.0;
    }
    else
    {
      double slow_ratio =
          1.0 - std::min(std::fabs(heading_error), 0.8) / 0.8;

      linear =
          base_linear * std::max(0.35, slow_ratio);
    }

    cmd.linear.x = linear;
    cmd.angular.z = angular;

    RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        500,
        "[PURE_PURSUIT_TF] nearest=%zu target=%zu goal_dist=%.3f robot_yaw=%.3f target_angle=%.3f heading_error=%.3f linear=%.3f angular=%.3f",
        nearest_idx,
        target_idx,
        goal_dist,
        current_location.theta,
        target_angle,
        heading_error,
        cmd.linear.x,
        cmd.angular.z);

    return cmd;
  }

  void SequenceManager::check_sound_trigger(
      const Point &current_location)
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
          detected_trigger = 3; //도착
        }
        else if (dist < 5.0)
        {
          detected_trigger = 2; //목적지 부근
        }
      }
      else if (dist < 0.1)
      {
        detected_trigger = 1; //현재위치
        break;
      }
    }

    sound_trigger = detected_trigger;
  }

} // namespace cango_master