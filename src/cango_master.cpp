#include "cango_master.hpp"

#include <chrono>
#include <functional>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>

namespace cango_master
{

  CangoMaster::CangoMaster() : Node("cango_master")
  {
    std::this_thread::sleep_for(std::chrono::seconds(5));

    tf_buffer_ =
        std::make_shared<tf2_ros::Buffer>(this->get_clock());

    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->setup();

    this->declare_parameter<double>("sound_trigger_distance", 0.0);
    this->declare_parameter<std::string>("semantic_config_path", "");

    this->get_parameter("sound_trigger_distance", sound_trigger_distance);
    this->get_parameter("semantic_config_path", semantic_config_path);

    if (!semantic_config_path.empty())
    {
      coordinate_converter.load_semantic_map(semantic_config_path);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Semantic config path is wrong!");
    }

    sequence_manager =
        std::make_unique<SequenceManager>(this, semantic_config_path);
  }

  void CangoMaster::setup()
  {
    hand_subscription =
        this->create_subscription<cango_msgs::msg::RobotControl>(
            "/hand2master", 10,
            std::bind(&CangoMaster::HandCB, this, std::placeholders::_1));

    navi_subscription =
        this->create_subscription<cango_msgs::msg::Navigation>(
            "/navi2master", 10,
            std::bind(&CangoMaster::NaviCB, this, std::placeholders::_1));

    safe_subscription =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/obs_distance", 10,
            std::bind(&CangoMaster::SafeCB, this, std::placeholders::_1));

    llm_subscription =
        this->create_subscription<cango_msgs::msg::LlmRequest>(
            "/cango/llm2master", 10,
            std::bind(&CangoMaster::LlmCB, this, std::placeholders::_1));

    sound_publisher =
        this->create_publisher<cango_msgs::msg::SoundRequest>(
            "/cango/master2sound", 10);

    master_publisher =
        this->create_publisher<cango_msgs::msg::TaskStatus>(
            "/task_status", 10);

    llm_publisher =
        this->create_publisher<cango_msgs::msg::LlmRequest>(
            "/cango/master2llm", 10);

    navi_publisher =
        this->create_publisher<cango_msgs::msg::Navigation>(
            "/master2navi", 10);

    control_publisher =
        this->create_publisher<cango_msgs::msg::RobotControl>(
            "/master2control", 10);

    timer_ =
        this->create_wall_timer(
            std::chrono::duration<double>(0.1),
            std::bind(&CangoMaster::timerCallback, this));
  }

  bool CangoMaster::update_robot_pose_from_tf()
  {
    try
    {
      geometry_msgs::msg::TransformStamped tf_msg =
          tf_buffer_->lookupTransform(
              "map",
              "base_link",
              tf2::TimePointZero);

      pcl_location.x = tf_msg.transform.translation.x;
      pcl_location.y = tf_msg.transform.translation.y;

      tf2::Quaternion q(
          tf_msg.transform.rotation.x,
          tf_msg.transform.rotation.y,
          tf_msg.transform.rotation.z,
          tf_msg.transform.rotation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      pcl_location.theta = yaw;

      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Failed to lookup transform map -> base_link: %s",
          ex.what());

      return false;
    }
  }

  void CangoMaster::timerCallback()
  {
    run();
  }

  void CangoMaster::reset()
  {
    if (sequence_manager)
    {
      sequence_manager->reset();
    }

    auto_driving = false;
    is_moving = false;
    ask_map_available = false;
    map_available = false;
  }

  void CangoMaster::run()
  {
    update_robot_pose_from_tf();

    StateChanger();

    control_pub();

    llm_pub();
  }

  void CangoMaster::StateChanger()
  {
    motor_enable = true;

    if (!auto_mode)
    {
      motor_enable = true;
      auto_driving = false;
      is_moving = false;

      if (sequence_manager)
      {
        sequence_manager->reset();
      }
    }

    if (ask_map_available && !map_available)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Map search requested. Creating full path...");

      sequence_manager->search_path(waypoint_list);

      bool check =
          sequence_manager->create_full_path(
              sequence_manager->path_list,
              pcl_location);

      if (check)
      {
        RCLCPP_INFO(this->get_logger(), "Path created successfully.");
        map_available = true;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Path creation failed.");
        map_available = false;
        ask_map_available = false;
      }
    }

    if (auto_mode && map_available && auto_driving)
    {
      bool success = sequence_manager->path_tracking();

      if (success)
      {
        is_moving = true;
        ask_map_available = false;
        map_available = false;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to start path tracking.");
      }
    }

    if (is_moving)
    {
      sequence_manager->check_sound_trigger(pcl_location);

      if (sequence_manager->sound_trigger != 0)
      {
        sound_pub();
      }
    }
  }

  void CangoMaster::NaviCB(
      const cango_msgs::msg::Navigation::ConstSharedPtr &msg)
  {
    /*
      위치와 heading은 /tf map -> base_link에서 직접 받습니다.
      여기서는 semantic 위치 변환만 유지합니다.
      만약 /tf가 안 들어오는 상황에서는 아래 x, y를 fallback으로 쓸 수 있습니다.
    */

    if (pcl_location.x == 0.0 && pcl_location.y == 0.0)
    {
      pcl_location.x = msg->current_location.x;
      pcl_location.y = msg->current_location.y;
    }

    coordinate_converter.pcd2id(
        pcl_location,
        semantic_location1,
        semantic_location2);
  }

  void CangoMaster::HandCB(
      const cango_msgs::msg::RobotControl::ConstSharedPtr &msg)
  {
    bool prev_mode = auto_mode;
    bool prev_robot_up = robot_up;

    auto_mode = (msg->mode == 0);
    robot_up = msg->robot_up;

    bool mode_changed = (prev_mode != auto_mode);
    bool robot_up_changed = (prev_robot_up != robot_up);

    if (mode_changed || robot_up_changed)
    {
      robot_cmd.vibration = true;
      vibration_flag = true;
    }
    else
    {
      robot_cmd.vibration = false;
    }

    robot_cmd.linear_speed = msg->linear_speed;
    robot_cmd.side_speed = msg->side_speed;
    robot_cmd.ang_speed = msg->ang_speed;
  }

  void CangoMaster::LlmCB(
      const cango_msgs::msg::LlmRequest::ConstSharedPtr &msg)
  {
    if (msg->request)
    {
      is_request = true;
    }
    else
    {
      is_request = false;
    }

    if (msg->user_interrupt)
    {
      is_user_interrupted = true;
      auto_driving = false;
      is_moving = false;

      if (sequence_manager)
      {
        sequence_manager->reset();
      }
    }
    else
    {
      is_user_interrupted = false;
      is_moving = true;
    }

    if (msg->user_finish)
    {
      is_request = false;
      is_user_interrupted = false;
      auto_driving = false;
      is_moving = false;

      if (sequence_manager)
      {
        sequence_manager->reset();
      }
    }

    if (msg->user_start)
    {
      is_user_interrupted = false;
      auto_driving = true;
      motor_enable = true;
    }

    if (msg->map_search)
    {
      ask_map_available = true;
    }
    else
    {
      ask_map_available = false;
    }

    goalpoint = msg->goalpoint;
    waypoint_list = msg->waypoints;
  }

  void CangoMaster::RobotStatusCB(
      const cango_msgs::msg::RobotStatus::ConstSharedPtr &msg)
  {
  }

  void CangoMaster::SafeCB(
      const std_msgs::msg::Float32MultiArray::ConstSharedPtr &msg)
  {
    if (msg->data.size() >= 2)
    {
      obs_safety = msg->data[0];
      obs_heading = msg->data[1];
    }
  }

  void CangoMaster::task_pub()
  {
  }

  void CangoMaster::sound_pub()
  {
    cango_msgs::msg::SoundRequest sound_request;

    sound_request.request = true;
    sound_request.ordered_num = sequence_manager->sound_trigger;

    sound_publisher->publish(sound_request);
  }

  void CangoMaster::llm_pub()
  {
    llm_request.local_candi1 = semantic_location1;
    llm_request.local_candi2 = semantic_location2;

    if (ask_map_available && map_available)
    {
      llm_request.map_search = 2;
    }
    else if (ask_map_available && !map_available)
    {
      llm_request.map_search = 1;
    }
    else
    {
      llm_request.map_search = 0;
    }

    llm_request.stand = robot_up;

    llm_publisher->publish(llm_request);
  }

  void CangoMaster::control_pub()
  {
    cango_msgs::msg::RobotControl robot_control;

    if (motor_enable)
    {
      if (auto_mode)
      {
        geometry_msgs::msg::Twist pp_cmd;

        if (sequence_manager)
        {
          pp_cmd =
              sequence_manager->update_pure_pursuit_cmd(pcl_location);
        }

        robot_control.linear_speed =
            pp_cmd.linear.x * robot_cmd.linear_speed * 3;

        robot_control.side_speed = 0.0;

        robot_control.ang_speed =
            -pp_cmd.angular.z * robot_cmd.linear_speed * 3;
      }
      else
      {
        robot_control.linear_speed =
            robot_cmd.linear_speed * obs_safety;

        robot_control.side_speed =
            robot_cmd.side_speed * obs_safety;

        robot_control.ang_speed =
            robot_cmd.ang_speed * obs_safety + obs_heading;
      }
    }
    else
    {
      robot_control.linear_speed = 0.0;
      robot_control.side_speed = 0.0;
      robot_control.ang_speed = 0.0;
    }

    robot_control.mode = auto_mode;
    robot_control.robot_up = robot_up;
    robot_control.vibration = vibration_flag;

    control_publisher->publish(robot_control);

    vibration_flag = false;
  }

}  // namespace cango_master

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<cango_master::CangoMaster>();

  rclcpp::executors::MultiThreadedExecutor executor;

  RCLCPP_INFO(node->get_logger(), "Running with MultiThreadedExecutor");

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}