#pragma once

#include <cango_msgs/msg/navigation.hpp>
#include <cango_msgs/msg/task_status.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sequence_manager.hpp>
#include <string>
#include <tools.hpp>
#include <vector>

namespace cango_master {

class CangoMaster : public rclcpp::Node {
 public:
  CangoMaster();
  bool auto_mode = false;

 private:
  void setup();
  void run();
  void reset();

  ////callback functions ///////

  void NaviCB(const cango_msgs::msg::Navigation::ConstSharedPtr& msg);
  void LlmCB(const cango_msgs::msg::LlmRequest::ConstSharedPtr& msg);
  void RobotStatusCB(const cango_msgs::msg::RobotStatus::ConstSharedPtr& msg);
  void timerCallback();

  void task_pub();
  void sound_pub();
  void llm_pub();
  void control_pub();

  bool ask_map_available = false;
  bool map_available = false;
  bool is_request = false;
  bool is_moving = false;
  bool is_user_interrupted = false; 
  bool motor_enable = false;

  cango_msgs::msg::TaskStatus now_status;
  cango_msgs::msg::SoundRequest sound_request;
  cango_msgs::msg::LlmRequest llm_request;
  cango_msgs::msg::RobotControl robot_control;

  ////////// tools //////////
  calc_coordinate coordinate_converter;
  robot_command robot_cmd;
  pd_controller pd_lin, pd_ang;
  SequenceManager sequence_manager;

  ///////////////navigation//////////////////
  std::string goalpoint;
  std::vector<std::string>
      waypoint_list;        //목적지 리스트, sequence manager에서 관리
  Point pcl_location;       //현재 위치, sequence manager
  std::string semantic_location1, semantic_location2;  // semantic map에서의 위치


 private:
  rclcpp::Subscription<cango_msgs::msg::Navigation>::SharedPtr
      navi_subscription;
  rclcpp::Publisher<cango_msgs::msg::TaskStatus>::SharedPtr master_publisher;
  rclcpp::Publisher<cango_msgs::msg::Navigation>::SharedPtr navi_publisher;
  rclcpp::Publisher<cango_msgs::msg::LlmRequest>::SharedPtr llm_publisher;
  rclcpp::Publisher<cango_msgs::msg::RobotControl>::SharedPtr control_publisher;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cango_master
