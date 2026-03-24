#include <chrono>
#include <functional>

#include <cango_master.hpp>


namespace cango_master {


CangoMaster::CangoMaster() : Node("cango_master") {
  this->setup();
}

void CangoMaster::setup() {

  navi_sub = this->create_subscription<cango_msgs::msg::Navigation>("~/navi2master", 10, std::bind(&CangoMaster::NaviCallback, this, std::placeholders::_1));
  master_pub = this->create_publisher<cango_msgs::msg::TaskStatus>("~/task_status", 10);

  timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&CangoMaster::timerCallback, this));
}

void CangoMaster::NaviCallback(const cango_msgs::msg::Navigation::ConstSharedPtr& msg) {

}

void CangoMaster::timerCallback() {
  RCLCPP_INFO(this->get_logger(), "Timer triggered");
}

}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<cango_master::CangoMaster>();
  rclcpp::executors::SingleThreadedExecutor executor;
  RCLCPP_INFO(node->get_logger(), "Spinning node '%s' with %s", node->get_fully_qualified_name(), "SingleThreadedExecutor");
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
