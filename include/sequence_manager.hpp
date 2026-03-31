#ifndef CANGO_MASTER_INCLUDE_SEQUENCE_MANAGER_HPP_
#define CANGO_MASTER_INCLUDE_SEQUENCE_MANAGER_HPP_

#include <cango_msgs/msg/navigation.hpp>
#include <cango_msgs/msg/task_status.hpp>
#include <memory>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <tools.hpp>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_plan.hpp"

namespace cango_master {

class SequenceManager {
 public:
  SequenceManager(rclcpp::Node* node, const std::string& yaml_path);
  SequenceManager();
  void reset();

  int sound_trigger = 0;
  // # 0 : None, 1 : %s지점에서 @회전 하고있습니다.
  // # 2 : 목적지 부근입니다. 3 : 목적지에 도착했습니다. @쪽에 %s가 있습니다.
  // # 4 : read custom text in text.

  /// navigation global-setting////
  void search_path(std::vector<std::string> waypoint_list);
  void create_full_path(const std::vector<Point>& path_list,
                        const Point& current_location);
  std::vector<Point> path_list;

  // check sound
  void check_sound_trigger(const Point& current_location);

 private:
  rclcpp::Node* node_;  // 마스터 노드를 가리키는 포인터
  void update_status();
  cango_msgs::msg::TaskStatus prev_status;
  cango_msgs::msg::TaskStatus new_status;

  ///////tools//////
  calc_coordinate coordinate_converter;

  /////nav tools//////
  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr
      planner_client_;
  nav_msgs::msg::Path last_generated_path_;
  rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr planner_service_client_;

  // localization//
  geometry_msgs::msg::PoseStamped get_current_pose(
      const Point& current_location);

  // PATH_ TRACKING
  void path_tracking(Point current_location);
  void next_waypoint(Point& next_pt);
};

}  // namespace cango_master

#endif  // CANGO_MASTER_INCLUDE_SEQUENCE_MANAGER_HPP_