// inspection_manager — 엔드밀 검사 상태기계 / RunInspection 액션 / GoToNamedPose 서비스.
//
// 상태기계: INIT → APPROACH → INSPECT(뷰별 stop-and-shoot) → RETREAT → HOME.
// 모션 전략(MVP): 규칙기반(RuleStrategy). "inspect" 기준 포즈에서 지정한 sweep 관절(기본 J6)을
// 각도만큼 증분시켜 뷰 웨이포인트를 생성한다. 시나리오 2(J6 회전 스윕)에 직접 대응하며,
// 시나리오 1(궤도) 및 MoveIt/태스크프레임 전략은 후속 단계에서 전략을 교체·추가한다.
//
// 카메라는 tending_camera 의 capture_image 서비스로 트리거하며, 부착 전에는 stub 이 응답한다.
// 각 촬영 순간의 관절/TCP/각도/이미지경로를 InspectionSample 로 게시 → tending_data 가 영속화.

#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tending_control/robot_io_bridge.hpp"
#include "tending_interfaces/action/run_inspection.hpp"
#include "tending_interfaces/srv/go_to_named_pose.hpp"
#include "tending_interfaces/srv/jog_joint.hpp"
#include "tending_interfaces/srv/capture_image.hpp"
#include "tending_interfaces/msg/inspection_sample.hpp"

using namespace std::chrono_literals;
using RunInspection = tending_interfaces::action::RunInspection;
using GoalHandle = rclcpp_action::ServerGoalHandle<RunInspection>;
using GoToNamedPose = tending_interfaces::srv::GoToNamedPose;
using JogJoint = tending_interfaces::srv::JogJoint;
using CaptureImage = tending_interfaces::srv::CaptureImage;
using InspectionSample = tending_interfaces::msg::InspectionSample;

class InspectionManager : public rclcpp::Node
{
public:
  InspectionManager()
  : rclcpp::Node("inspection_manager")
  {
    // 모션 파라미터
    velocity_ = declare_parameter<double>("velocity", 0.4);         // rad/s
    acc_time_ms_ = declare_parameter<double>("acc_time_ms", 200.0);
    reach_tol_rad_ = declare_parameter<double>("reach_tol_rad", 0.02);
    move_timeout_s_ = declare_parameter<int>("move_timeout_s", 30);
    settle_ms_ = declare_parameter<int>("settle_ms", 400);
    sweep_joint_ = declare_parameter<int>("sweep_joint_index", 5);   // J6
    base_frame_ = declare_parameter<std::string>("base_frame", "base");

    // 명명 포즈 로드: pose_names + poses.<name> (double[6])
    auto names = declare_parameter<std::vector<std::string>>(
      "pose_names", std::vector<std::string>{"home", "init", "approach", "inspect", "retreat"});
    for (const auto & n : names) {
      auto q = declare_parameter<std::vector<double>>("poses." + n, std::vector<double>{});
      if (!q.empty()) {
        poses_[n] = q;
      } else {
        RCLCPP_WARN(get_logger(), "명명 포즈 '%s' 값이 비어있습니다(미설정).", n.c_str());
      }
    }

    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    bridge_ = std::make_unique<tending_control::RobotIoBridge>(this, cb_group_);

    capture_cli_ = create_client<CaptureImage>(
      "capture_image", rmw_qos_profile_services_default, cb_group_);
    sample_pub_ = create_publisher<InspectionSample>("inspection_sample", rclcpp::QoS(50));

    goto_srv_ = create_service<GoToNamedPose>(
      "go_to_named_pose",
      std::bind(&InspectionManager::handle_goto, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_);

    jog_srv_ = create_service<JogJoint>(
      "jog_joint",
      std::bind(&InspectionManager::handle_jog, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_);

    action_srv_ = rclcpp_action::create_server<RunInspection>(
      this, "run_inspection",
      std::bind(&InspectionManager::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&InspectionManager::handle_cancel, this, std::placeholders::_1),
      std::bind(&InspectionManager::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "inspection_manager 준비 완료. tm_driver 서비스 대기 중...");
  }

  bool startup_check()
  {
    if (!bridge_->wait_for_services(10s)) {
      RCLCPP_ERROR(get_logger(), "tm_driver 서비스(set_positions/send_script/set_event) 미검출. "
        "tm_bringup 이 실행 중인지 확인하세요.");
      return false;
    }
    RCLCPP_INFO(get_logger(), "tm_driver 서비스 연결됨.");
    return true;
  }

private:
  // ---- GoToNamedPose ----
  void handle_goto(
    const std::shared_ptr<GoToNamedPose::Request> req,
    std::shared_ptr<GoToNamedPose::Response> res)
  {
    auto it = poses_.find(req->pose_name);
    if (it == poses_.end()) {
      res->ok = false;
      res->message = "알 수 없는 포즈: " + req->pose_name;
      return;
    }
    if (inspecting_.load()) {
      res->ok = false;
      res->message = "검사 실행 중에는 포즈 이동을 받지 않습니다.";
      return;
    }
    std::string reason;
    if (!bridge_->safety_ok(reason)) {
      res->ok = false;
      res->message = "안전 점검 실패: " + reason;
      return;
    }
    double vel = (req->velocity > 0.0) ? req->velocity : velocity_;
    bool ok = move_to(it->second, vel);
    res->ok = ok;
    res->message = ok ? ("이동 완료: " + req->pose_name) : ("이동 실패: " + req->pose_name);
  }

  // ---- JogJoint ----
  // 현재 관절값을 읽어 지정 축에만 delta 를 더한 목표로 PTP 이동. 수동 조작(GUI) 용도.
  void handle_jog(
    const std::shared_ptr<JogJoint::Request> req,
    std::shared_ptr<JogJoint::Response> res)
  {
    if (inspecting_.load()) {
      res->ok = false;
      res->message = "검사 실행 중에는 조그를 받지 않습니다.";
      return;
    }
    if (req->joint < 1 || req->joint > 6) {
      res->ok = false;
      res->message = "관절 번호는 1..6 이어야 합니다: " + std::to_string(req->joint);
      return;
    }
    if (!bridge_->have_state()) {
      res->ok = false;
      res->message = "로봇 상태 미수신(tm_driver 확인)";
      return;
    }
    std::string reason;
    if (!bridge_->safety_ok(reason)) {
      res->ok = false;
      res->message = "안전 점검 실패: " + reason;
      return;
    }

    auto q = bridge_->joint_pos();
    if (q.size() < 6) {
      res->ok = false;
      res->message = "관절값 수신 이상(size=" + std::to_string(q.size()) + ")";
      return;
    }
    q[req->joint - 1] += req->delta;

    const double vel = (req->velocity > 0.0) ? req->velocity : velocity_;
    const bool ok = move_to(q, vel);
    res->ok = ok;
    res->message = ok ?
      ("조그 완료: J" + std::to_string(req->joint)) :
      ("조그 실패: J" + std::to_string(req->joint));
  }

  // ---- RunInspection ----
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const RunInspection::Goal> goal)
  {
    if (goal->num_views == 0) {
      RCLCPP_WARN(get_logger(), "num_views=0 거부");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!poses_.count("inspect")) {
      RCLCPP_ERROR(get_logger(), "'inspect' 기준 포즈 미설정으로 거부");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "검사 취소 요청 수신 → 로봇 정지");
    bridge_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> gh)
  {
    std::thread{std::bind(&InspectionManager::execute, this, gh)}.detach();
  }

  bool move_to(const std::vector<double> & q, double vel)
  {
    return bridge_->move_ptp_joint(
      q, vel, acc_time_ms_, reach_tol_rad_, std::chrono::seconds(move_timeout_s_));
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandle> & gh, const std::string & phase,
    double progress, uint16_t captured)
  {
    auto fb = std::make_shared<RunInspection::Feedback>();
    fb->phase = phase;
    fb->progress = progress;
    fb->captured = captured;
    gh->publish_feedback(fb);
    RCLCPP_INFO(get_logger(), "[%s] progress=%.2f captured=%u", phase.c_str(), progress, captured);
  }

  void execute(const std::shared_ptr<GoalHandle> gh)
  {
    // 검사 중에는 go_to_named_pose/jog_joint 를 거부한다(모션 충돌 방지).
    inspecting_.store(true);
    struct Guard
    {
      std::atomic<bool> & f;
      ~Guard() {f.store(false);}
    } guard{inspecting_};

    const auto goal = gh->get_goal();
    auto result = std::make_shared<RunInspection::Result>();
    result->image_count = 0;

    auto fail = [&](const std::string & msg) {
      bridge_->stop();
      result->success = false;
      result->message = msg;
      gh->abort(result);
      RCLCPP_ERROR(get_logger(), "검사 중단: %s", msg.c_str());
    };

    // 사전 점검
    std::string reason;
    if (!bridge_->have_state()) { fail("로봇 상태 미수신(tm_driver/피드백 확인)"); return; }
    if (!bridge_->safety_ok(reason)) { fail("안전 사전점검 실패: " + reason); return; }

    // INIT
    publish_feedback(gh, "INIT", 0.0, 0);
    if (poses_.count("init") && !move_to(poses_["init"], velocity_)) { fail("init 이동 실패"); return; }

    // APPROACH (approach → inspect 기준)
    publish_feedback(gh, "APPROACH", 0.1, 0);
    if (poses_.count("approach") && !move_to(poses_["approach"], velocity_)) { fail("approach 이동 실패"); return; }
    if (!move_to(poses_["inspect"], velocity_)) { fail("inspect 기준 이동 실패"); return; }

    // INSPECT
    const auto waypoints = make_waypoints(goal);
    const std::string label = goal->machine_id.empty() ? "run" : goal->machine_id;
    for (size_t i = 0; i < waypoints.size(); ++i) {
      if (gh->is_canceling()) {
        bridge_->stop();
        result->success = false;
        result->message = "사용자 취소";
        result->image_count = static_cast<uint16_t>(i);
        gh->canceled(result);
        return;
      }
      if (!move_to(waypoints[i].joints, velocity_)) { fail("검사 웨이포인트 이동 실패"); return; }
      std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms_));

      std::string image_path;
      if (!capture(label, static_cast<uint32_t>(i), image_path)) { fail("이미지 촬영 실패"); return; }

      publish_sample(static_cast<uint32_t>(i), waypoints[i].angle, image_path);
      result->image_count = static_cast<uint16_t>(i + 1);

      double prog = 0.2 + 0.7 * static_cast<double>(i + 1) / static_cast<double>(waypoints.size());
      publish_feedback(gh, "INSPECT", prog, result->image_count);
    }

    // RETREAT / HOME
    publish_feedback(gh, "RETREAT", 0.92, result->image_count);
    if (poses_.count("retreat") && !move_to(poses_["retreat"], velocity_)) { fail("retreat 이동 실패"); return; }
    publish_feedback(gh, "HOME", 0.98, result->image_count);
    if (poses_.count("home") && !move_to(poses_["home"], velocity_)) { fail("home 이동 실패"); return; }

    result->success = true;
    result->dataset_path = dataset_path(label);
    result->message = "검사 완료";
    publish_feedback(gh, "DONE", 1.0, result->image_count);
    gh->succeed(result);
    RCLCPP_INFO(get_logger(), "검사 완료: %u 장, %s", result->image_count, result->dataset_path.c_str());
  }

  struct Waypoint { std::vector<double> joints; double angle; };

  std::vector<Waypoint> make_waypoints(std::shared_ptr<const RunInspection::Goal> goal)
  {
    std::vector<Waypoint> wps;
    const auto & base = poses_.at("inspect");
    const int n = static_cast<int>(goal->num_views);
    const double a0 = goal->angle_start;
    const double a1 = goal->angle_end;
    const double range = a1 - a0;
    const bool full_circle = std::fabs(std::fabs(range) - 2.0 * M_PI) < 1e-3;
    double step = 0.0;
    if (n > 1) {
      step = full_circle ? (range / n) : (range / (n - 1));
    }
    int sj = sweep_joint_;
    if (sj < 0 || sj >= static_cast<int>(base.size())) {
      sj = static_cast<int>(base.size()) - 1;
    }
    for (int i = 0; i < n; ++i) {
      const double angle = a0 + step * i;
      Waypoint w;
      w.joints = base;
      w.joints[sj] = base[sj] + (angle - a0);
      w.angle = angle;
      wps.push_back(std::move(w));
    }
    return wps;
  }

  bool capture(const std::string & label, uint32_t view_index, std::string & image_path)
  {
    if (!capture_cli_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "capture_image 서비스 미준비(tending_camera 확인)");
      return false;
    }
    auto req = std::make_shared<CaptureImage::Request>();
    req->label = label;
    req->view_index = view_index;
    auto future = capture_cli_->async_send_request(req);
    if (future.wait_for(10s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "capture_image 응답 시간초과");
      return false;
    }
    auto resp = future.get();
    image_path = resp->image_path;
    return resp->ok;
  }

  void publish_sample(uint32_t view_index, double angle, const std::string & image_path)
  {
    InspectionSample s;
    s.header.stamp = now();
    s.header.frame_id = base_frame_;
    s.view_index = view_index;
    s.angle = angle;
    s.joint_pos = bridge_->joint_pos();
    s.tcp_pose = bridge_->tcp_pose();
    s.image_path = image_path;
    sample_pub_->publish(s);
  }

  std::string dataset_path(const std::string & label)
  {
    // capture_image 의 output_dir 기본값과 일치(tending_camera 파라미터). 기본 /tmp/tending_dataset.
    return "/tmp/tending_dataset/" + label;
  }

  // params
  double velocity_, acc_time_ms_, reach_tol_rad_;
  int move_timeout_s_, settle_ms_, sweep_joint_;
  std::string base_frame_;
  std::map<std::string, std::vector<double>> poses_;

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::unique_ptr<tending_control::RobotIoBridge> bridge_;
  rclcpp::Client<CaptureImage>::SharedPtr capture_cli_;
  rclcpp::Publisher<InspectionSample>::SharedPtr sample_pub_;
  rclcpp::Service<GoToNamedPose>::SharedPtr goto_srv_;
  rclcpp::Service<JogJoint>::SharedPtr jog_srv_;
  rclcpp_action::Server<RunInspection>::SharedPtr action_srv_;
  std::atomic<bool> inspecting_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InspectionManager>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  node->startup_check();
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
