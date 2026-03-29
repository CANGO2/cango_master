#include <chrono>
#include <functional>
#include <include/cango_master.hpp>

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
  navi_subscription = this->create_subscription<cango_msgs::msg::Navigation>(
      "~/navi2master", 10,
      std::bind(&CangoMaster::NaviCB, this, std::placeholders::_1));
  llm_subscription = this->create_subscription<cango_msgs::msg::LlmRequest>(
      "~/llm2master", 10,
      std::bind(&CangoMaster::LlmCB, this, std::placeholders::_1));

  master_publisher =
      this->create_publisher<cango_msgs::msg::TaskStatus>("~/task_status", 10);
  llm_publisher =
      this->create_publisher<cango_msgs::msg::LlmRequest>("~/master2llm", 10);
  navi_publisher =
      this->create_publisher<cango_msgs::msg::Navigation>("~/master2navi", 10);
  control_pub = this->create_publisher<cango_msgs::msg::RobotControl>(
      "~/master2control", 10);
  timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0),
                              std::bind(&CangoMaster::timerCallback, this));
}
void CangoMaster::timerCallback() { run(); }
void CangoMaster::reset() {}
void CangoMaster::run() {
  StateChanger();

  if (is_moter_enable) {
    control_pub();
  }

  llm_pub();
}
void CangoMaster::StateChanger() {
  if (ask_map_available) {
    sequence_manager.search_path(waypoint_list);
    sequence_manager.create_full_path(sequence_manager.path_list, pcl_location);
    // 나온 path가 가능한 길이라고 뜨면,
    // map_available = true;
  }

  if (is_moving) {
    sequence_manager.check_sound_trigger(pcl_location);
    if (sequence_manager.sound_trigger != 0) {
      sound_pub();
    }
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
    is_motor_enable = true;
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
  sound_request.ordered_num = sequence_manager.sound_trigger;

  sound_publisher->publish(sound_request);
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
  //나중에 아예 길이 불가능할경우 3넣는거 만들어야함

  if () llm_publisher->publish(llm_request);
}

void CangoMaster::control_pub() {
  // 나중에 조종이 들어가면 rot_in_place 여부에 따라 조종변화

  if (motor_enable) {
    robot_control.lineer_speed = robot_cmd.lin_vel;
    robot_control.ang_speed = robot_cmd.ang_vel;
  } else {
    robot_control.lineer_speed = 0.0;
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
