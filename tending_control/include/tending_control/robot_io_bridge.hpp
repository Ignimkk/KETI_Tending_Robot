// RobotIoBridge — tm_driver(TM5) 서비스/토픽을 감싸는 얇은 C++ 클라이언트 계층.
//
// 상위 계층(inspection_manager)이 TM 세부(SetPositions/SendScript/SetEvent, FeedbackState 등)를
// 직접 다루지 않도록 최소한의 안전한 API 만 노출한다.
//  - 명령: PTP(joint) 이동, 원시 스크립트 전송, 정지(StopAndClearBuffer)
//  - 상태: 관절값/TCP 포즈 스냅샷, 연결/안전 플래그
//
// tm_driver 는 읽기 전용 의존성이며 본 클래스는 tmr_ros2 내부 코드를 수정하지 않는다.

#ifndef TENDING_CONTROL__ROBOT_IO_BRIDGE_HPP_
#define TENDING_CONTROL__ROBOT_IO_BRIDGE_HPP_

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tm_msgs/msg/feedback_state.hpp"
#include "tm_msgs/srv/set_positions.hpp"
#include "tm_msgs/srv/send_script.hpp"
#include "tm_msgs/srv/set_event.hpp"

namespace tending_control
{

class RobotIoBridge
{
public:
  // node: 소유 노드(수명 관리는 호출자). cb_group: 구독/클라이언트를 붙일 콜백 그룹
  // (MultiThreadedExecutor + Reentrant 권장 — 액션 실행 스레드에서 대기 중에도 상태 갱신).
  RobotIoBridge(rclcpp::Node * node, rclcpp::CallbackGroup::SharedPtr cb_group);

  // tm_driver 서비스가 준비될 때까지 대기.
  bool wait_for_services(std::chrono::seconds timeout);

  // 피드백을 한 번이라도 수신했는지.
  bool have_state();

  // TMSCT/TMSVR 양 채널 연결 여부.
  bool connected();

  // 안전 상태(e_stop/robot_error/safetyguard_a 없음). 문제 시 reason 채움.
  bool safety_ok(std::string & reason);

  std::vector<double> joint_pos();
  geometry_msgs::msg::Pose tcp_pose();     // base→TCP (tool_pose 토픽)
  geometry_msgs::msg::Pose flange_pose();  // base→flange (feedback tool0_pose)

  // PTP(joint) 이동 후 목표 근방 도달까지 폴링 대기. vel: rad/s(<=pi).
  bool move_ptp_joint(
    const std::vector<double> & q, double vel, double acc_time_ms,
    double tol_rad, std::chrono::seconds timeout);

  // Cartesian PTP(관절보간, 자세는 TM 내부 IK). pose=[x,y,z,rx,ry,rz] (m, rad).
  // vel: 관절속도 비율(rad/s 상당, SetPositions 규약). 도달은 TCP 포즈로 판정.
  bool move_ptp_tool(
    const std::vector<double> & pose_xyzrpy, double vel, double acc_time_ms,
    double pos_tol_m, double ang_tol_rad, std::chrono::seconds timeout);

  // Cartesian 직선(LINE_T). vel: m/s. 검사거리 일관성이 중요한 궤도에 적합.
  bool move_line_tool(
    const std::vector<double> & pose_xyzrpy, double vel, double acc_time_ms,
    double pos_tol_m, double ang_tol_rad, std::chrono::seconds timeout);

  // 원시 TM 스크립트 전송(예: PTP/Line/ChangeTCP). 완료 대기는 하지 않는다.
  bool send_script(const std::string & id, const std::string & script);

  // 즉시 정지(StopAndClearBuffer).
  bool stop();

private:
  void on_feedback(const tm_msgs::msg::FeedbackState::SharedPtr msg);
  void on_tool_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Node * node_;

  rclcpp::Subscription<tm_msgs::msg::FeedbackState>::SharedPtr feedback_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tool_pose_sub_;

  rclcpp::Client<tm_msgs::srv::SetPositions>::SharedPtr set_positions_cli_;
  rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr send_script_cli_;
  rclcpp::Client<tm_msgs::srv::SetEvent>::SharedPtr set_event_cli_;

  std::mutex mtx_;
  bool have_state_{false};
  bool is_sct_{false};
  bool is_svr_{false};
  bool e_stop_{false};
  bool robot_error_{false};
  bool safetyguard_{false};
  int error_code_{0};
  std::vector<double> joint_pos_;
  geometry_msgs::msg::Pose tcp_pose_;
  geometry_msgs::msg::Pose flange_pose_;

  // 공통: Cartesian 이동 후 TCP 포즈가 목표에 도달할 때까지 폴링 대기.
  bool wait_until_pose(
    const std::vector<double> & target_xyzrpy,
    double pos_tol_m, double ang_tol_rad, std::chrono::seconds timeout);
};

}  // namespace tending_control

#endif  // TENDING_CONTROL__ROBOT_IO_BRIDGE_HPP_
