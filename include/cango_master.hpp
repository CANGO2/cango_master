#pragma once

#include <cango_msgs/msg/navigation.hpp>
#include <cango_msgs/msg/task_status.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sequence_manager.hpp>
#include <string>
#include <tools.hpp>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "cango_msgs/msg/llm_request.hpp"
#include "cango_msgs/msg/robot_control.hpp"
#include "cango_msgs/msg/robot_status.hpp"
#include "cango_msgs/msg/sound_request.hpp"

namespace cango_master {

class CangoMaster : public rclcpp::Node {
 public:
  CangoMaster();
  bool auto_mode = false; // 자율주행 알고리즘 활성화
  bool auto_driving = false; // 실제 자율주행 여부 
  bool robot_up = false; // 로봇이 서야한다는 명령
  bool robot_stand = false; // 로봇이 서있는 상태인지 여부


 private:
  void setup();
  void run();
  void reset();
  void StateChanger();
  void collision_avoid();
  ////callback functions ///////

  void NaviCB(const cango_msgs::msg::Navigation::ConstSharedPtr& msg);
  void LlmCB(const cango_msgs::msg::LlmRequest::ConstSharedPtr& msg);
  void HandCB(const cango_msgs::msg::RobotControl::ConstSharedPtr& msg);
  void RobotStatusCB(const cango_msgs::msg::RobotStatus::ConstSharedPtr& msg);
  void Nav2CB(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timerCallback();

  void task_pub();
  void sound_pub();
  void llm_pub();
  void control_pub();

  bool ask_map_available = false;
  bool map_available = false;
  bool is_moving = false;
  bool is_request = false;
  bool is_user_interrupted = false;
  bool motor_enable = false;
  bool teleop_action_once = false;

  cango_msgs::msg::TaskStatus now_status;
  cango_msgs::msg::SoundRequest sound_request;
  cango_msgs::msg::LlmRequest llm_request;

  ////////// tools //////////
  calc_coordinate coordinate_converter;
  robot_command robot_cmd;
  robot_command nav2_cmd;
  pd_controller pd_lin, pd_ang;
  std::unique_ptr<SequenceManager> sequence_manager;

  ///////////////navigation//////////////////
  std::string goalpoint;
  std::vector<std::string>
      waypoint_list;   //목적지 리스트, sequence manager에서 관리
  Point pcl_location;  //현재 위치, sequence manager
  std::string semantic_location1,
      semantic_location2;  // semantic map에서의 위치

 private:
  rclcpp::Subscription<cango_msgs::msg::Navigation>::SharedPtr
      navi_subscription;
  rclcpp::Subscription<cango_msgs::msg::RobotControl>::SharedPtr
      hand_subscription;
  rclcpp::Subscription<cango_msgs::msg::LlmRequest>::SharedPtr llm_subscription;
  rclcpp::Publisher<cango_msgs::msg::TaskStatus>::SharedPtr master_publisher;
  rclcpp::Publisher<cango_msgs::msg::Navigation>::SharedPtr navi_publisher;
  rclcpp::Publisher<cango_msgs::msg::LlmRequest>::SharedPtr llm_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_publisher;
  rclcpp::Publisher<cango_msgs::msg::RobotControl>::SharedPtr control_publisher;
  rclcpp::Publisher<cango_msgs::msg::SoundRequest>::SharedPtr sound_publisher;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_cmd_subscription;
};

}  // namespace cango_master
