#include <rclcpp/rclcpp.hpp>
#include <cango_msgs/msg/navigation.hpp>
#include <cango_msgs/msg/task_status.hpp>
#include <tools.hpp>

namespace cango_master {   


class SequenceManager{

 public:

  SequenceManager();
 private:
  
  void change_status(enum task_name task_num, int &act);
  void update_status();
  cango_msgs::msg::task_status prev_status;
  cango_msgs::msg::task_status update_status;

};

}


