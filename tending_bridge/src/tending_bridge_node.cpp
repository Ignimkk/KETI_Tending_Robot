// tending_bridge — ROS 2 ↔ Windows GUI(KETI_CNCTendingRobot) 경계 노드.
//
// doc/PROTOCOL.md v1 구현. 개행 구분 JSON 라인 TCP 서버(JsonLineServer)를 열고,
//   ROS → GUI : feedback_states/tool_pose + run_inspection 액션 피드백을 state/event 로 스트리밍
//   GUI → ROS : run_inspection(액션), go_to_named_pose/jog_joint(서비스), cancel, stop
// 을 중계한다.
//
// 설계 원칙
//  - 모션 명령은 모두 inspection_manager 를 통과한다(모션 소유자 단일화). 유일한 예외는
//    비상정지 stop 으로, 지연 최소화를 위해 tm_msgs/SetEvent(STOP) 를 직접 호출한다.
//  - tmr_ros2 는 읽기 전용 의존성이며 내부 코드를 수정하지 않는다.
//  - JsonLineServer 콜백은 IO 스레드에서 온다. 여기서는 절대 블로킹하지 않고
//    콜백 기반 async_send_request/async_send_goal 만 사용한다.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "tm_msgs/msg/feedback_state.hpp"
#include "tm_msgs/srv/set_event.hpp"

#include "tending_interfaces/action/run_inspection.hpp"
#include "tending_interfaces/srv/go_to_named_pose.hpp"
#include "tending_interfaces/srv/jog_joint.hpp"

#include "tending_bridge/json_line_server.hpp"

#include <nlohmann/json.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;

using RunInspection = tending_interfaces::action::RunInspection;
using GoalHandleRunInspection = rclcpp_action::ClientGoalHandle<RunInspection>;
using GoToNamedPose = tending_interfaces::srv::GoToNamedPose;
using JogJoint = tending_interfaces::srv::JogJoint;
using SetEvent = tm_msgs::srv::SetEvent;
using tending_bridge::ClientId;

namespace
{
constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kM2Mm = 1000.0;

int64_t now_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}
}  // namespace

class TendingBridge : public rclcpp::Node
{
public:
  TendingBridge()
  : rclcpp::Node("tending_bridge")
  {
    bind_address_ = declare_parameter<std::string>("bind_address", "0.0.0.0");
    port_ = declare_parameter<int>("port", 5901);
    state_rate_hz_ = declare_parameter<double>("state_rate_hz", 10.0);
    client_timeout_ms_ = declare_parameter<int>("client_timeout_ms", 3000);
    max_clients_ = declare_parameter<int>("max_clients", 2);
    schema_version_ = declare_parameter<int>("schema_version", 1);
    driver_timeout_ms_ = declare_parameter<int>("driver_timeout_ms", 2000);

    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // 감시 타이머는 에지 검출 상태를 들고 있으므로 재진입하면 안 된다.
    watchdog_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_opt;
    sub_opt.callback_group = cb_group_;

    feedback_sub_ = create_subscription<tm_msgs::msg::FeedbackState>(
      "feedback_states", rclcpp::SensorDataQoS(),
      std::bind(&TendingBridge::on_feedback, this, std::placeholders::_1), sub_opt);

    tool_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "tool_pose", rclcpp::SensorDataQoS(),
      std::bind(&TendingBridge::on_tool_pose, this, std::placeholders::_1), sub_opt);

    inspect_cli_ = rclcpp_action::create_client<RunInspection>(this, "run_inspection", cb_group_);
    goto_cli_ = create_client<GoToNamedPose>(
      "go_to_named_pose", rmw_qos_profile_services_default, cb_group_);
    jog_cli_ = create_client<JogJoint>("jog_joint", rmw_qos_profile_services_default, cb_group_);
    set_event_cli_ = create_client<SetEvent>(
      "set_event", rmw_qos_profile_services_default, cb_group_);

    server_.set_log_handler(
      [this](int level, const std::string & msg) {
        if (level >= 2) {
          RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
        } else if (level == 1) {
          RCLCPP_WARN(get_logger(), "%s", msg.c_str());
        } else {
          RCLCPP_INFO(get_logger(), "%s", msg.c_str());
        }
      });
    server_.set_connect_handler([this](ClientId id) {on_client_connect(id);});
    server_.set_line_handler([this](ClientId id, const std::string & l) {on_line(id, l);});

    const auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / (state_rate_hz_ > 0.1 ? state_rate_hz_ : 0.1)));
    state_timer_ = create_wall_timer(
      period, std::bind(&TendingBridge::on_state_timer, this), cb_group_);
    watchdog_timer_ = create_wall_timer(
      200ms, std::bind(&TendingBridge::on_watchdog, this), watchdog_group_);
  }

  bool start_server()
  {
    std::string err;
    if (!server_.start(bind_address_, static_cast<uint16_t>(port_), max_clients_, err)) {
      RCLCPP_ERROR(get_logger(), "TCP 서버 기동 실패: %s", err.c_str());
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "tending_bridge 수신 대기 %s:%d (schema v%d, state %.1f Hz)",
      bind_address_.c_str(), port_, schema_version_, state_rate_hz_);
    return true;
  }

  void shutdown() {server_.stop();}

private:
  // ================= ROS 구독 =================

  void on_feedback(const tm_msgs::msg::FeedbackState::SharedPtr msg)
  {
    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      changed =
        (have_feedback_ != true) ||
        (is_sct_ != msg->is_sct_connected) || (is_svr_ != msg->is_svr_connected) ||
        (e_stop_ != msg->e_stop) || (robot_error_ != msg->robot_error) ||
        (safetyguard_ != msg->safetyguard_a) || (project_run_ != msg->project_run);

      have_feedback_ = true;
      last_feedback_ = std::chrono::steady_clock::now();
      is_sct_ = msg->is_sct_connected;
      is_svr_ = msg->is_svr_connected;
      e_stop_ = msg->e_stop;
      robot_error_ = msg->robot_error;
      safetyguard_ = msg->safetyguard_a;
      project_run_ = msg->project_run;
      error_code_ = static_cast<int>(msg->error_code);
      joint_pos_ = msg->joint_pos;
    }

    if (changed) {
      publish_state();          // 플래그 변화는 주기를 기다리지 않고 즉시 반영
      emit_safety_events(msg);
    }
  }

  void emit_safety_events(const tm_msgs::msg::FeedbackState::SharedPtr & msg)
  {
    // 구독이 Reentrant 그룹에 있어 콜백이 동시 실행될 수 있다. 에지 검출은 직렬화해야
    // 같은 전이에 대해 이벤트가 중복 발행되지 않는다.
    std::lock_guard<std::mutex> lock(event_mtx_);

    if (msg->e_stop && !prev_e_stop_) {
      send_event("error", "SAFETY_ESTOP", "e-stop 감지");
      abort_active_job("e-stop");
    }
    if (msg->safetyguard_a && !prev_guard_) {
      send_event("error", "SAFETY_GUARD", "안전문(safetyguard) 작동");
      abort_active_job("safetyguard");
    }
    if (msg->robot_error && !prev_robot_error_) {
      send_event(
        "error", "ROBOT_ERROR",
        "로봇 에러 (error_code=" + std::to_string(msg->error_code) + ")");
    }
    prev_e_stop_ = msg->e_stop;
    prev_guard_ = msg->safetyguard_a;
    prev_robot_error_ = msg->robot_error;
  }

  void on_tool_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    tcp_pose_ = msg->pose;
    have_tcp_ = true;
  }

  // ================= 상태 송신 =================

  void on_state_timer() {publish_state();}

  void publish_state()
  {
    if (server_.client_count() == 0) {
      return;
    }
    server_.broadcast(build_state().dump());
  }

  json build_state()
  {
    json j;
    j["v"] = schema_version_;
    j["type"] = "state";
    j["ts"] = now_ms();

    std::lock_guard<std::mutex> lock(state_mtx_);

    const bool driver_alive = have_feedback_ &&
      std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_feedback_).count() < driver_timeout_ms_;

    j["link"] = {{"sct", is_sct_}, {"svr", is_svr_}, {"driver", driver_alive}};
    j["safety"] = {
      {"e_stop", e_stop_}, {"robot_error", robot_error_}, {"safetyguard", safetyguard_},
      {"error_code", error_code_}, {"project_run", project_run_}};

    json joints = json::array();
    for (double q : joint_pos_) {
      joints.push_back(q * kRad2Deg);
    }
    j["joint_deg"] = joints;

    if (have_tcp_) {
      tf2::Quaternion q(
        tcp_pose_.orientation.x, tcp_pose_.orientation.y,
        tcp_pose_.orientation.z, tcp_pose_.orientation.w);
      double r = 0.0, p = 0.0, y = 0.0;
      tf2::Matrix3x3(q).getRPY(r, p, y);
      j["tcp"] = {
        {"x", tcp_pose_.position.x * kM2Mm},
        {"y", tcp_pose_.position.y * kM2Mm},
        {"z", tcp_pose_.position.z * kM2Mm},
        {"rx", r * kRad2Deg}, {"ry", p * kRad2Deg}, {"rz", y * kRad2Deg}};
    } else {
      j["tcp"] = {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"rx", 0.0}, {"ry", 0.0}, {"rz", 0.0}};
    }

    if (job_active_) {
      j["job"] = {
        {"active", true}, {"id", job_id_}, {"cmd", job_cmd_},
        {"phase", job_phase_}, {"progress", job_progress_}, {"captured", job_captured_}};
    } else {
      j["job"] = {{"active", false}};
    }
    return j;
  }

  void send_event(const std::string & level, const std::string & code, const std::string & message)
  {
    json j;
    j["v"] = schema_version_;
    j["type"] = "event";
    j["ts"] = now_ms();
    j["level"] = level;
    j["code"] = code;
    j["message"] = message;
    server_.broadcast(j.dump());
  }

  void send_ack(ClientId cid, const std::string & id, bool ok, const std::string & message)
  {
    json j;
    j["v"] = schema_version_;
    j["type"] = "ack";
    j["ts"] = now_ms();
    j["id"] = id;
    j["ok"] = ok;
    j["message"] = message;
    server_.send_to(cid, j.dump());
  }

  void send_result(
    ClientId cid, const std::string & id, const std::string & status, bool success,
    const std::string & message, const std::string & dataset_path = "", int image_count = 0)
  {
    json j;
    j["v"] = schema_version_;
    j["type"] = "result";
    j["ts"] = now_ms();
    j["id"] = id;
    j["status"] = status;
    j["success"] = success;
    j["message"] = message;
    j["dataset_path"] = dataset_path;
    j["image_count"] = image_count;
    server_.send_to(cid, j.dump());
  }

  // ================= 클라이언트 이벤트 =================

  void on_client_connect(ClientId id)
  {
    server_.send_to(id, build_state().dump());   // PROTOCOL §6: 연결 직후 스냅샷 1회
  }

  void on_line(ClientId cid, const std::string & line)
  {
    json j;
    try {
      j = json::parse(line);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "JSON 파싱 실패: %s", e.what());
      send_ack(cid, "", false, "BAD_JSON");
      return;
    }

    const int v = j.value("v", 0);
    if (v != schema_version_) {
      RCLCPP_WARN(get_logger(), "스키마 버전 불일치: 수신 v%d, 기대 v%d", v, schema_version_);
    }
    if (j.value("type", std::string()) != "cmd") {
      return;   // 클라이언트가 보낼 수 있는 것은 cmd 뿐
    }

    const std::string id = j.value("id", std::string());
    const std::string cmd = j.value("cmd", std::string());
    const json args = j.value("args", json::object());

    if (cmd == "ping") {
      send_ack(cid, id, true, "");
    } else if (cmd == "stop") {
      handle_stop(cid, id);
    } else if (cmd == "cancel") {
      handle_cancel(cid, id, args);
    } else if (cmd == "run_inspection") {
      handle_run_inspection(cid, id, args);
    } else if (cmd == "goto_pose") {
      handle_goto_pose(cid, id, args);
    } else if (cmd == "jog") {
      handle_jog(cid, id, args);
    } else {
      send_ack(cid, id, false, "UNKNOWN_CMD");
    }
  }

  // ================= 명령 처리 =================

  // 모션 명령 슬롯 확보. 실패 시 false 와 함께 ack 를 이미 보낸 상태.
  bool claim_job(ClientId cid, const std::string & id, const std::string & cmd)
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    if (job_active_) {
      send_ack(cid, id, false, "BUSY");
      return false;
    }
    job_active_ = true;
    job_id_ = id;
    job_cmd_ = cmd;
    job_client_ = cid;
    job_phase_ = "PENDING";
    job_progress_ = 0.0;
    job_captured_ = 0;
    return true;
  }

  void release_job()
  {
    std::lock_guard<std::mutex> lock(state_mtx_);
    job_active_ = false;
    job_phase_.clear();
    job_progress_ = 0.0;
    job_captured_ = 0;
    inspect_goal_.reset();
  }

  void handle_run_inspection(ClientId cid, const std::string & id, const json & args)
  {
    if (!inspect_cli_->action_server_is_ready()) {
      send_ack(cid, id, false, "SERVICE_UNAVAILABLE");
      return;
    }
    const int num_views = args.value("num_views", 0);
    if (num_views <= 0) {
      send_ack(cid, id, false, "BAD_ARGS");
      return;
    }
    if (!claim_job(cid, id, "run_inspection")) {
      return;
    }
    send_ack(cid, id, true, "");

    RunInspection::Goal goal;
    goal.machine_id = args.value("machine_id", std::string("mc_a"));
    goal.scenario = static_cast<uint8_t>(args.value("scenario", 1));
    goal.angle_start = args.value("angle_start_deg", 0.0) * kDeg2Rad;
    goal.angle_end = args.value("angle_end_deg", 360.0) * kDeg2Rad;
    goal.num_views = static_cast<uint16_t>(num_views);
    goal.inspect_distance = args.value("inspect_distance_mm", 0.0) / kM2Mm;

    rclcpp_action::Client<RunInspection>::SendGoalOptions opt;

    opt.goal_response_callback =
      [this, cid, id](GoalHandleRunInspection::SharedPtr gh) {
        if (!gh) {
          send_result(cid, id, "REJECTED", false, "액션 서버가 goal 을 거부했습니다.");
          release_job();
          return;
        }
        std::lock_guard<std::mutex> lock(state_mtx_);
        inspect_goal_ = gh;
      };

    opt.feedback_callback =
      [this](GoalHandleRunInspection::SharedPtr,
        const std::shared_ptr<const RunInspection::Feedback> fb) {
        {
          std::lock_guard<std::mutex> lock(state_mtx_);
          job_phase_ = fb->phase;
          job_progress_ = fb->progress;
          job_captured_ = fb->captured;
        }
        publish_state();        // 단계 전환은 GUI 에 지연 없이 반영
      };

    opt.result_callback =
      [this, cid, id](const GoalHandleRunInspection::WrappedResult & wr) {
        std::string status = "ABORTED";
        switch (wr.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: status = "SUCCEEDED"; break;
          case rclcpp_action::ResultCode::CANCELED: status = "CANCELED"; break;
          default: status = "ABORTED"; break;
        }
        const bool ok = wr.result && wr.result->success;
        send_result(
          cid, id, status, ok,
          wr.result ? wr.result->message : std::string("결과 없음"),
          wr.result ? wr.result->dataset_path : std::string(),
          wr.result ? static_cast<int>(wr.result->image_count) : 0);
        release_job();
        publish_state();
      };

    inspect_cli_->async_send_goal(goal, opt);
  }

  void handle_goto_pose(ClientId cid, const std::string & id, const json & args)
  {
    if (!goto_cli_->service_is_ready()) {
      send_ack(cid, id, false, "SERVICE_UNAVAILABLE");
      return;
    }
    const std::string pose_name = args.value("pose_name", std::string());
    if (pose_name.empty()) {
      send_ack(cid, id, false, "BAD_ARGS");
      return;
    }
    if (!claim_job(cid, id, "goto_pose")) {
      return;
    }
    send_ack(cid, id, true, "");
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      job_phase_ = "MOVING";
    }

    auto req = std::make_shared<GoToNamedPose::Request>();
    req->pose_name = pose_name;
    req->velocity = args.value("velocity_deg_s", 0.0) * kDeg2Rad;

    goto_cli_->async_send_request(
      req,
      [this, cid, id](rclcpp::Client<GoToNamedPose>::SharedFuture future) {
        auto res = future.get();
        send_result(
          cid, id, res->ok ? "SUCCEEDED" : "ABORTED", res->ok, res->message);
        release_job();
        publish_state();
      });
  }

  void handle_jog(ClientId cid, const std::string & id, const json & args)
  {
    if (!jog_cli_->service_is_ready()) {
      send_ack(cid, id, false, "SERVICE_UNAVAILABLE");
      return;
    }
    const int joint = args.value("joint", 0);
    if (joint < 1 || joint > 6) {
      send_ack(cid, id, false, "BAD_ARGS");
      return;
    }
    if (!claim_job(cid, id, "jog")) {
      return;
    }
    send_ack(cid, id, true, "");
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      job_phase_ = "JOG";
    }

    auto req = std::make_shared<JogJoint::Request>();
    req->joint = static_cast<uint8_t>(joint);
    req->delta = args.value("delta_deg", 0.0) * kDeg2Rad;
    req->velocity = args.value("velocity_deg_s", 0.0) * kDeg2Rad;

    jog_cli_->async_send_request(
      req,
      [this, cid, id](rclcpp::Client<JogJoint>::SharedFuture future) {
        auto res = future.get();
        send_result(cid, id, res->ok ? "SUCCEEDED" : "ABORTED", res->ok, res->message);
        release_job();
        publish_state();
      });
  }

  void handle_cancel(ClientId cid, const std::string & id, const json & args)
  {
    const std::string target = args.value("target_id", std::string());
    GoalHandleRunInspection::SharedPtr gh;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      if (!job_active_ || (!target.empty() && target != job_id_)) {
        send_ack(cid, id, false, "NOT_RUNNING");
        return;
      }
      gh = inspect_goal_;
    }
    send_ack(cid, id, true, "");

    if (gh) {
      inspect_cli_->async_cancel_goal(gh);   // 종결은 result_callback 에서 CANCELED 로 통지
    } else {
      // goto_pose/jog 는 서비스라 중도 취소가 없다 → 정지로 대신한다.
      do_stop("사용자 취소");
    }
  }

  void handle_stop(ClientId cid, const std::string & id)
  {
    send_ack(cid, id, true, "");
    do_stop("사용자 비상정지");
  }

  // 비상정지: SetEvent(STOP) 직접 호출 + 활성 goal cancel. PROTOCOL §5.1.
  void do_stop(const std::string & reason)
  {
    RCLCPP_WARN(get_logger(), "정지 요청: %s", reason.c_str());

    if (set_event_cli_->service_is_ready()) {
      auto req = std::make_shared<SetEvent::Request>();
      req->func = SetEvent::Request::STOP;
      req->arg0 = 0;
      req->arg1 = 0;
      set_event_cli_->async_send_request(req);
    } else {
      RCLCPP_ERROR(get_logger(), "set_event 서비스 미준비 — STOP 을 전달하지 못했습니다.");
      send_event("error", "LOG", "set_event 미준비로 STOP 전달 실패");
    }

    GoalHandleRunInspection::SharedPtr gh;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      gh = inspect_goal_;
    }
    if (gh) {
      inspect_cli_->async_cancel_goal(gh);
    }
  }

  // 안전 이벤트로 인한 중단. 사용자가 요청한 것이 아니므로 이벤트도 함께 남긴다.
  void abort_active_job(const std::string & reason)
  {
    bool active = false;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      active = job_active_;
    }
    if (active) {
      do_stop("안전 이벤트: " + reason);
    }
  }

  // ================= 감시 =================

  void on_watchdog()
  {
    // 1) tm_driver 피드백 끊김 감지
    bool alive = false;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      alive = have_feedback_ &&
        std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_feedback_).count() < driver_timeout_ms_;
    }
    if (alive != prev_driver_alive_) {
      if (alive) {
        send_event("info", "DRIVER_OK", "tm_driver 피드백 수신 재개");
      } else {
        send_event("error", "DRIVER_LOST", "tm_driver 피드백 끊김");
        abort_active_job("tm_driver 피드백 끊김");
      }
      prev_driver_alive_ = alive;
    }

    // 2) 데드맨 — 검사 중 GUI 무음이면 정지 (PROTOCOL §5.3)
    std::chrono::milliseconds silence{0};
    if (!server_.last_rx_elapsed(silence)) {
      return;   // 접속 클라이언트 없음: 유휴 상태로 간주
    }
    bool active = false;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      active = job_active_;
    }
    if (active && silence.count() > client_timeout_ms_ && !deadman_fired_) {
      deadman_fired_ = true;
      send_event(
        "error", "DEADMAN_STOP",
        "GUI 무음 " + std::to_string(silence.count()) + "ms — 자동 정지");
      do_stop("데드맨 타임아웃");
    } else if (silence.count() <= client_timeout_ms_) {
      deadman_fired_ = false;
    }
  }

  // ================= 멤버 =================

  std::string bind_address_;
  int port_{5901};
  double state_rate_hz_{10.0};
  int client_timeout_ms_{3000};
  int max_clients_{2};
  int schema_version_{1};
  int driver_timeout_ms_{2000};

  tending_bridge::JsonLineServer server_;

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::CallbackGroup::SharedPtr watchdog_group_;
  rclcpp::Subscription<tm_msgs::msg::FeedbackState>::SharedPtr feedback_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tool_pose_sub_;
  rclcpp_action::Client<RunInspection>::SharedPtr inspect_cli_;
  rclcpp::Client<GoToNamedPose>::SharedPtr goto_cli_;
  rclcpp::Client<JogJoint>::SharedPtr jog_cli_;
  rclcpp::Client<SetEvent>::SharedPtr set_event_cli_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::mutex state_mtx_;
  bool have_feedback_{false};
  bool have_tcp_{false};
  std::chrono::steady_clock::time_point last_feedback_;
  bool is_sct_{false}, is_svr_{false};
  bool e_stop_{false}, robot_error_{false}, safetyguard_{false}, project_run_{false};
  int error_code_{0};
  std::vector<double> joint_pos_{0, 0, 0, 0, 0, 0};
  geometry_msgs::msg::Pose tcp_pose_;

  bool job_active_{false};
  std::string job_id_, job_cmd_, job_phase_;
  double job_progress_{0.0};
  int job_captured_{0};
  ClientId job_client_{0};
  GoalHandleRunInspection::SharedPtr inspect_goal_;

  // 이벤트 에지 검출용. prev_e_stop_/prev_guard_/prev_robot_error_ 는 event_mtx_ 로 보호.
  // prev_driver_alive_/deadman_fired_ 는 watchdog 타이머 단일 경로에서만 접근한다.
  std::mutex event_mtx_;
  bool prev_e_stop_{false}, prev_guard_{false}, prev_robot_error_{false};
  bool prev_driver_alive_{false};
  bool deadman_fired_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TendingBridge>();
  if (!node->start_server()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  node->shutdown();
  rclcpp::shutdown();
  return 0;
}
