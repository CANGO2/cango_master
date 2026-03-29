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
#include "nav2_msgs/srv/get_plan.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav_msgs/msg/path.hpp"
namespace cango_master {

class SequenceManager {
 public:
  SequenceManager(rclcpp::Node* node, const std::string& yaml_path);
  SequenceManager();
  void reset();

 private:
  rclcpp::Node* node_;  // 마스터 노드를 가리키는 포인터
  void change_status(enum task_name task_num, int& act);
  void update_status();
  cango_msgs::msg::task_status prev_status;
  cango_msgs::msg::task_status update_status;

  ///////tools//////
  calc_coordinate coordinate_converter;

  /////nav2 tools//////
  rclcpp_action::Client<ComputePath>::SharedPtr planner_client_;
  nav_msgs::msg::Path last_generated_path_;
  rclcpp::Client<nav2_msgs::srv::GetPlan>::SharedPtr planner_service_client_;

  // localization//
  geometry_msgs::msg::PoseStamped get_current_pose();

  /// navigation global-setting////
  void search_path(std::vector<std::string> waypoint_list);
  std::vector<Point> path_list;
  void create_full_path(const std::vector<Point> path_list);

  // PATH_ TRACKING
  int sound_trigger = 0;
// # 0 : None
// # 1 : %s지점에서 @회전 하고있습니다. 
// # 2 : 목적지 부근입니다.
// # 3 : 목적지에 도착했습니다. @쪽에 %s가 있습니다.
// # 4 : read custom text in text.

  void path_tracking(Point current_location);
  void next_waypoint(Point& next_pt);
  
  //check sound
  void check_sound_trigger(const Point &current_location);
};

}  // namespace cango_master

/// 내비관련된거 관리해주는 패키지임!!

// 1. 목적지 리스트 queue처럼 관리했다가 navi에다가 하나씩 쏴주고, 현재위치
// 변하는거 지켜보면서 업데이트하는 역할을 하는애
// 2. 음성알림 시점트리거 역할

// - 현재가 로봇이 대기상태여서 LLM이랑 대화해야할때 어떤요청사항이 필요한지
// 정리
// - LLM에서 사람이 경로 재생성 요청했을때 초기화

// Task 완료 및 실패 보고: Navi에 좌표를 쐈는데 장애물 때문에 못 가는
// 경우(Abort), 마스터에게 "실패함! 다음 계획 어떡함?"이라고 보고하는 기능. 중간
// 지점(Waypoint) 알림: 목적지까지 가는 길에 특정 노드(예: 205호 앞)를 통과할
// 때, 마스터에게 "방금 205호 통과함"이라고 알려주어 마스터가 LLM에게 실시간
// 보고를 할 수 있게 돕는 역할. 음성 우선순위 관리: LLM 대화 도중 긴급
// 상황(배터리 부족 등)이 생기면 기존 음성을 끊고 긴급 음성을 먼저 내보내는
// 트리거 관리.