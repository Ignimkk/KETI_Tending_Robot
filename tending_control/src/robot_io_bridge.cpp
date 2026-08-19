#include "tending_control/robot_io_bridge.hpp"
#include "tending_control/pose_utils.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

using namespace std::chrono_literals;

namespace tending_control
{

RobotIoBridge::RobotIoBridge(rclcpp::Node * node, rclcpp::CallbackGroup::SharedPtr cb_group)
: node_(node)
{
  // tmr_ros2 의 가상(fake) 로봇은 FeedbackState 의 안전 플래그를 채우지 않아
  // e_stop/robot_error/safetyguard_a 가 모두 true, error_code 가 쓰레기값으로 올라온다.
  // 그 결과 safety_ok() 가 항상 실패해 오프라인 검증이 불가능하므로,
  // 시뮬레이션 전용 탈출구를 옵트인으로 제공한다. 실물에서는 절대 켜지 말 것.
  ignore_safety_flags_ = node_->has_parameter("ignore_safety_flags") ?
    node_->get_parameter("ignore_safety_flags").as_bool() :
    node_->declare_parameter<bool>("ignore_safety_flags", false);
  if (ignore_safety_flags_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "ignore_safety_flags=true — 안전 플래그(e_stop/safetyguard/robot_error/연결)를 무시합니다. "
      "가상 로봇 검증 전용입니다. 실물 로봇에서는 절대 사용하지 마십시오.");
  }

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group;

  feedback_sub_ = node_->create_subscription<tm_msgs::msg::FeedbackState>(
    "feedback_states", rclcpp::SensorDataQoS(),
    std::bind(&RobotIoBridge::on_feedback, this, std::placeholders::_1), sub_opts);

  tool_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "tool_pose", rclcpp::SensorDataQoS(),
    std::bind(&RobotIoBridge::on_tool_pose, this, std::placeholders::_1), sub_opts);

  set_positions_cli_ = node_->create_client<tm_msgs::srv::SetPositions>(
    "set_positions", rmw_qos_profile_services_default, cb_group);
  send_script_cli_ = node_->create_client<tm_msgs::srv::SendScript>(
    "send_script", rmw_qos_profile_services_default, cb_group);
  set_event_cli_ = node_->create_client<tm_msgs::srv::SetEvent>(
    "set_event", rmw_qos_profile_services_default, cb_group);
}

void RobotIoBridge::on_feedback(const tm_msgs::msg::FeedbackState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  have_state_ = true;
  is_sct_ = msg->is_sct_connected;
  is_svr_ = msg->is_svr_connected;
  e_stop_ = msg->e_stop;
  robot_error_ = msg->robot_error;
  safetyguard_ = msg->safetyguard_a;
  error_code_ = msg->error_code;
  joint_pos_ = msg->joint_pos;
  if (msg->tool0_pose.size() == 6) {
    flange_pose_ = tending_control::toPoseMsg(
      tending_control::fromTmCartesian(msg->tool0_pose));
  }
}

void RobotIoBridge::on_tool_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  tcp_pose_ = msg->pose;
}

bool RobotIoBridge::wait_for_services(std::chrono::seconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto ready = [&]() {
    return set_positions_cli_->service_is_ready() &&
           send_script_cli_->service_is_ready() &&
           set_event_cli_->service_is_ready();
  };
  while (rclcpp::ok() && !ready()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(100ms);
  }
  return ready();
}

bool RobotIoBridge::have_state()
{
  std::lock_guard<std::mutex> lk(mtx_);
  return have_state_;
}

bool RobotIoBridge::connected()
{
  std::lock_guard<std::mutex> lk(mtx_);
  return have_state_ && is_sct_ && is_svr_;
}

bool RobotIoBridge::safety_ok(std::string & reason)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!have_state_) { reason = "로봇 상태 미수신"; return false; }
  // 상태 미수신은 시뮬에서도 실제 문제이므로 위 검사는 항상 유효하게 둔다.
  if (ignore_safety_flags_) { return true; }
  if (e_stop_) { reason = "비상정지(e_stop) 활성"; return false; }
  if (safetyguard_) { reason = "세이프가드(safetyguard_a) 트리거"; return false; }
  if (robot_error_) { reason = "로봇 에러(code=" + std::to_string(error_code_) + ")"; return false; }
  if (!is_sct_ || !is_svr_) { reason = "TM 통신 미연결"; return false; }
  return true;
}

std::vector<double> RobotIoBridge::joint_pos()
{
  std::lock_guard<std::mutex> lk(mtx_);
  return joint_pos_;
}

geometry_msgs::msg::Pose RobotIoBridge::tcp_pose()
{
  std::lock_guard<std::mutex> lk(mtx_);
  return tcp_pose_;
}

geometry_msgs::msg::Pose RobotIoBridge::flange_pose()
{
  std::lock_guard<std::mutex> lk(mtx_);
  return flange_pose_;
}

bool RobotIoBridge::move_ptp_joint(
  const std::vector<double> & q, double vel, double acc_time_ms,
  double tol_rad, std::chrono::seconds timeout)
{
  auto req = std::make_shared<tm_msgs::srv::SetPositions::Request>();
  req->motion_type = tm_msgs::srv::SetPositions::Request::PTP_J;
  req->positions = q;
  req->velocity = vel;
  req->acc_time = acc_time_ms;
  req->blend_percentage = 0;
  req->fine_goal = true;

  auto future = set_positions_cli_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "set_positions 응답 시간초과");
    return false;
  }
  if (!future.get()->ok) {
    RCLCPP_ERROR(node_->get_logger(), "set_positions 거부됨(스크립트 오류/미연결)");
    return false;
  }

  // 목표 근방 도달까지 폴링 대기.
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok()) {
    std::string reason;
    if (!safety_ok(reason)) {
      RCLCPP_ERROR(node_->get_logger(), "이동 중 안전 이상: %s", reason.c_str());
      stop();
      return false;
    }
    auto cur = joint_pos();
    if (cur.size() == q.size()) {
      double maxerr = 0.0;
      for (size_t i = 0; i < q.size(); ++i) {
        maxerr = std::max(maxerr, std::fabs(cur[i] - q[i]));
      }
      if (maxerr <= tol_rad) {
        return true;
      }
    }
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(node_->get_logger(), "이동 완료 대기 시간초과");
      return false;
    }
    std::this_thread::sleep_for(50ms);
  }
  return false;
}

bool RobotIoBridge::wait_until_pose(
  const std::vector<double> & target_xyzrpy,
  double pos_tol_m, double ang_tol_rad, std::chrono::seconds timeout)
{
  const Eigen::Isometry3d target = tending_control::fromTmCartesian(target_xyzrpy);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok()) {
    std::string reason;
    if (!safety_ok(reason)) {
      RCLCPP_ERROR(node_->get_logger(), "이동 중 안전 이상: %s", reason.c_str());
      stop();
      return false;
    }
    const Eigen::Isometry3d cur = tending_control::fromPoseMsg(tcp_pose());
    const double dp = (cur.translation() - target.translation()).norm();
    const Eigen::AngleAxisd aa(cur.rotation().transpose() * target.rotation());
    const double da = std::fabs(aa.angle());
    if (dp <= pos_tol_m && da <= ang_tol_rad) {
      return true;
    }
    if (std::chrono::steady_clock::now() > deadline) {
      RCLCPP_ERROR(node_->get_logger(),
        "Cartesian 이동 대기 시간초과 (dp=%.4f m, da=%.4f rad)", dp, da);
      return false;
    }
    std::this_thread::sleep_for(50ms);
  }
  return false;
}

bool RobotIoBridge::move_ptp_tool(
  const std::vector<double> & pose_xyzrpy, double vel, double acc_time_ms,
  double pos_tol_m, double ang_tol_rad, std::chrono::seconds timeout)
{
  auto req = std::make_shared<tm_msgs::srv::SetPositions::Request>();
  req->motion_type = tm_msgs::srv::SetPositions::Request::PTP_T;
  req->positions = pose_xyzrpy;
  req->velocity = vel;
  req->acc_time = acc_time_ms;
  req->blend_percentage = 0;
  req->fine_goal = true;

  auto future = set_positions_cli_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "set_positions(PTP_T) 응답 시간초과");
    return false;
  }
  if (!future.get()->ok) {
    RCLCPP_ERROR(node_->get_logger(), "set_positions(PTP_T) 거부됨(IK 실패/미연결)");
    return false;
  }
  return wait_until_pose(pose_xyzrpy, pos_tol_m, ang_tol_rad, timeout);
}

bool RobotIoBridge::move_line_tool(
  const std::vector<double> & pose_xyzrpy, double vel, double acc_time_ms,
  double pos_tol_m, double ang_tol_rad, std::chrono::seconds timeout)
{
  auto req = std::make_shared<tm_msgs::srv::SetPositions::Request>();
  req->motion_type = tm_msgs::srv::SetPositions::Request::LINE_T;
  req->positions = pose_xyzrpy;
  req->velocity = vel;              // m/s
  req->acc_time = acc_time_ms;
  req->blend_percentage = 0;
  req->fine_goal = true;

  auto future = set_positions_cli_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "set_positions(LINE_T) 응답 시간초과");
    return false;
  }
  if (!future.get()->ok) {
    RCLCPP_ERROR(node_->get_logger(), "set_positions(LINE_T) 거부됨(경로 불가/미연결)");
    return false;
  }
  return wait_until_pose(pose_xyzrpy, pos_tol_m, ang_tol_rad, timeout);
}

bool RobotIoBridge::send_script(const std::string & id, const std::string & script)
{
  auto req = std::make_shared<tm_msgs::srv::SendScript::Request>();
  req->id = id;
  req->script = script;
  auto future = send_script_cli_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "send_script 응답 시간초과");
    return false;
  }
  return future.get()->ok;
}

bool RobotIoBridge::stop()
{
  auto req = std::make_shared<tm_msgs::srv::SetEvent::Request>();
  req->func = tm_msgs::srv::SetEvent::Request::STOP;
  req->arg0 = 0;
  req->arg1 = 0;
  auto future = set_event_cli_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "정지(set_event STOP) 응답 시간초과");
    return false;
  }
  return future.get()->ok;
}

}  // namespace tending_control
