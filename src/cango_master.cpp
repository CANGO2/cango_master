#include "cango_master.hpp"

#include <chrono>
#include <functional>

namespace cango_master {

CangoMaster::CangoMaster() : Node("cango_master") {
  this->setup();

  this->declare_parameter<std::string>("semantic_config_path", "");

  std::string semantic_config_path;
  this->get_parameter("semantic_config_path", semantic_config_path);
  if (!semantic_config_path.empty()) {
    coordinate_converter.load_semantic_map(semantic_config_path);
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Semantic config path is empty! Check your launch file.");
  }
  sequence_manager =
      std::make_unique<SequenceManager>(this, semantic_config_path);
}

void CangoMaster::setup() {
  /// sub
  hand_subscription = this->create_subscription<cango_msgs::msg::RobotControl>(
      "/hand2master", 10,
      std::bind(&CangoMaster::HandCB, this, std::placeholders::_1));
  navi_subscription = this->create_subscription<cango_msgs::msg::Navigation>(
      "/navi2master", 10,
      std::bind(&CangoMaster::NaviCB, this, std::placeholders::_1));
  llm_subscription = this->create_subscription<cango_msgs::msg::LlmRequest>(
      "/llm2master", 10,
      std::bind(&CangoMaster::LlmCB, this, std::placeholders::_1));
  sound_publisher = this->create_publisher<cango_msgs::msg::SoundRequest>(
      "/master2sound", 10);
  master_publisher =
      this->create_publisher<cango_msgs::msg::TaskStatus>("/task_status", 10);
  llm_publisher =
      this->create_publisher<cango_msgs::msg::LlmRequest>("/master2llm", 10);
  navi_publisher =
      this->create_publisher<cango_msgs::msg::Navigation>("/master2navi", 10);
  control_publisher = this->create_publisher<cango_msgs::msg::RobotControl>(
      "/master2control", 10);
  nav2_cmd_subscription = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CangoMaster::Nav2CB, this, std::placeholders::_1));
  teleop_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
      "/collision_cmd", 10);
  timer_ =
      this->create_wall_timer(std::chrono::duration<double>(0.1),
                              std::bind(&CangoMaster::timerCallback, this));
}
void CangoMaster::timerCallback() { run(); }
void CangoMaster::reset() {}
void CangoMaster::run() {
  StateChanger();

  control_pub();

  llm_pub();
}

void CangoMaster::StateChanger() {
  
  //디버깅용!!!
  motor_enable = true;

  
  if (!auto_mode) {
    ask_map_available = false;
    motor_enable = true;
    map_available = false;
    auto_driving = false;
  } else if (auto_mode) {
    teleop_action_once = false;
    if (ask_map_available && !map_available) {
      sequence_manager->search_path(waypoint_list);
      bool check = sequence_manager->create_full_path(
          sequence_manager->path_list, pcl_location);
      if (check) {
        map_available = true;
      }
    } else if (map_available && auto_driving) {
      bool success = sequence_manager->path_tracking(
          pcl_location, sequence_manager->path_list);
      if (success) {
        is_moving = true;
        ask_map_available = false;
        map_available = false;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to start path tracking.");
      }
    }
  }

  /////////////////////////////조종디버깅용////////////////////////////
  // if (is_moving) {
  //   sequence_manager->check_sound_trigger(pcl_location);
  //   if (sequence_manager->sound_trigger != 0) {
  //     sound_pub();
  //   }
  // }
}
void CangoMaster::collision_avoid() {
  if (!auto_mode && motor_enable) {
    if (!teleop_action_once){
    teleop_action_once = true;
    sequence_manager->send_assisted_teleop();}

    geometry_msgs::msg::Twist drive_msg;
    drive_msg.linear.x = robot_cmd.linear_speed;
    drive_msg.linear.y = robot_cmd.side_speed;
    drive_msg.angular.z = robot_cmd.ang_speed;
    teleop_publisher->publish(drive_msg);
  }
}

void CangoMaster::NaviCB(
    const cango_msgs::msg::Navigation::ConstSharedPtr& msg) {
  //목적지, 경로 업데이트
  pcl_location.x = msg->current_location.x;
  pcl_location.y = msg->current_location.y;
  coordinate_converter.pcd2id(pcl_location, semantic_location1,
                              semantic_location2);
}
void CangoMaster::HandCB(
    const cango_msgs::msg::RobotControl::ConstSharedPtr& msg) {
      //테스트용 제거
  // if(msg->robot_up == 1){
  //   auto_mode = false;
  //   robot_up = false;
  //   return;
  // }
  // else if(msg->robot_up == false){
  //   auto_mode = false;
  //   robot_up = true;
  // }
  // if(msg->mode == 1) {
  //   auto_mode = false;
  // } else {
  //   auto_mode = true;
  // }
  auto_mode = false;
  robot_cmd.linear_speed = msg->linear_speed;
  robot_cmd.side_speed = msg->side_speed;
  robot_cmd.ang_speed = msg->ang_speed;
}

void CangoMaster::LlmCB(
    const cango_msgs::msg::LlmRequest::ConstSharedPtr& msg) {
  if (msg->request) {
    is_request = true;
  } else {
    is_request = false;
  }
  if (msg->user_interrupt) {
    is_user_interrupted = true;
  } else {
    is_user_interrupted = false;
  }
  if (msg->user_finish) {
    is_request = false;
    is_user_interrupted = false;
  }
  if (msg->user_start) {
    is_user_interrupted = false;
    auto_driving = true;
    motor_enable = true;
  }
  if (msg->map_search) {
    ask_map_available = true;
  } else {
    ask_map_available = false;
  }
  goalpoint = msg->goalpoint;
  waypoint_list = msg->waypoints;
}

void CangoMaster::RobotStatusCB(
    const cango_msgs::msg::RobotStatus::ConstSharedPtr& msg) {}
void CangoMaster::task_pub() {}
void CangoMaster::sound_pub() {
  cango_msgs::msg::SoundRequest sound_request;
  sound_request.request = true;
  sound_request.ordered_num = sequence_manager->sound_trigger;

  sound_publisher->publish(sound_request);
}

void CangoMaster::Nav2CB(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (auto_mode) {
    nav2_cmd.linear_speed = msg->linear.x;
    nav2_cmd.side_speed = msg->linear.y;  // 옴니휠/메카넘휠인 경우 사용
    nav2_cmd.ang_speed = msg->angular.z;
  }
}

void CangoMaster::llm_pub() {
  llm_request.local_candi1 = semantic_location1;
  llm_request.local_candi2 = semantic_location2;
  if (ask_map_available && map_available) {
    llm_request.map_search = 2;
  } else if (ask_map_available && !map_available) {
    llm_request.map_search = 1;
  } else {
    llm_request.map_search = 0;
  }
  llm_publisher->publish(llm_request);
}

void CangoMaster::control_pub() {
  cango_msgs::msg::RobotControl robot_control;

  if (robot_up && !robot_stand){
    robot_control.robot_up = true;
      control_publisher->publish(robot_control);
      robot_stand = true;
    return;
  }
  if (motor_enable) {
    if (auto_mode) {
      
      robot_control.linear_speed = nav2_cmd.linear_speed;
      robot_control.side_speed = nav2_cmd.side_speed;
      robot_control.ang_speed = nav2_cmd.ang_speed;
    } else {  //수동 모드
      /////////////////////////////조종디버깅용 주석////////////////////////////
      //collision_avoid();
      robot_control.linear_speed = robot_cmd.linear_speed;
      robot_control.side_speed = robot_cmd.side_speed;
      robot_control.ang_speed = robot_cmd.ang_speed;
      std::cout<<"aaaaaaaaaaaaaaa"<<std::endl;
    }
  } else {
    //sequence_manager->cancel_assisted_teleop();
    robot_control.linear_speed = 0.0;
    robot_control.side_speed = 0.0;
    robot_control.ang_speed = 0.0;
  }
  control_publisher->publish(robot_control);
}
}  // namespace cango_master

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cango_master::CangoMaster>();
  rclcpp::executors::SingleThreadedExecutor executor;
  RCLCPP_INFO(node->get_logger(), "Spinning node '%s' with %s",
              node->get_fully_qualified_name(), "SingleThreadedExecutor");
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
