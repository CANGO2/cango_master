#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <cango_msgs/msg/navigation.hpp>
#include <cango_msgs/msg/task_status.hpp>
#include <tools.hpp>


namespace cango_master {

class CangoMaster : public rclcpp::Node {

 public:

  CangoMaster();

 private:
  void setup();

  void NaviCallback(const cango_msgs::msg::Navigation::ConstSharedPtr& msg);
  void timerCallback();

 private:

  rclcpp::Subscription<cango_msgs::msg::Navigation>::SharedPtr navi_sub;
  rclcpp::Publisher<cango_msgs::msg::TaskStatus>::SharedPtr master_pub;

  rclcpp::TimerBase::SharedPtr timer_;
};


}
