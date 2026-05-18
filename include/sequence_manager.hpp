#ifndef CANGO_MASTER_INCLUDE_SEQUENCE_MANAGER_HPP_
#define CANGO_MASTER_INCLUDE_SEQUENCE_MANAGER_HPP_

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <cango_msgs/msg/navigation.hpp>
#include <cango_msgs/msg/robot_control.hpp>
#include <cango_msgs/msg/task_status.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <nav_msgs/msg/path.hpp>

#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <nav2_msgs/srv/is_path_valid.hpp>

#include <tools.hpp>

namespace cango_master
{

class SequenceManager
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  SequenceManager(rclcpp::Node *node, const std::string &yaml_path);
  SequenceManager() = default;

  void reset();

  int sound_trigger = 0;

  void search_path(std::vector<std::string> waypoint_list);

  bool create_full_path(
      const std::vector<Point> &path_list,
      const Point &current_location);

  bool path_tracking();

  void check_sound_trigger(const Point &current_location);

  std::vector<Point> path_list;

private:
  rclcpp::Node *node_ = nullptr;

  void update_status();

  cango_msgs::msg::TaskStatus prev_status;
  cango_msgs::msg::TaskStatus new_status;

  calc_coordinate coordinate_converter;

  rclcpp::CallbackGroup::SharedPtr action_callback_group_;

  rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_path_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr navigation_action_client_;

  nav_msgs::msg::Path last_generated_path_;

  geometry_msgs::msg::PoseStamped get_current_pose(
      const Point &current_location);
};

}  // namespace cango_master

#endif  // CANGO_MASTER_INCLUDE_SEQUENCE_MANAGER_HPP_