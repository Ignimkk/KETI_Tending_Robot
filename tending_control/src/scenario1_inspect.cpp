// scenario1_inspect — 시나리오 1(손목 장착) 임의 좌표 툴 검사 제어 로직.
//
// 두 가지 제어 방식을 control_mode 로 선택한다(세 번째 MoveIt 방식은 tending_moveit 패키지):
//   (1) rule   : 하드코딩/룰베이스. 툴 축을 base 좌표로 직접 지정, 궤도 포즈를 open-loop
//                Cartesian 명령(PTP_T/LINE_T)으로 전송. 충돌회피/도달성 검사 없음. 가장 단순·취약.
//   (3) hybrid : 제안 방식. 툴 축을 '태스크 프레임(cnc_frame)' 기준으로 정의하고 YAML 로 외부화,
//                궤도를 태스크 프레임 상대로 생성 → base 로 변환. 각 뷰를 safe_region 박스로 검증한
//                뒤 Cartesian 실행. 기계 교체 시 YAML 만 바꾸면 재사용(코드 무변경).
//
// dry_run=true(기본): 로봇 명령 없이 생성된 궤도 포즈만 출력(오프라인 검증/카메라 부재 안전).
// dry_run=false     : 실제 tm_driver 로 이동.

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "tending_control/robot_io_bridge.hpp"
#include "tending_control/pose_utils.hpp"

using namespace std::chrono_literals;
using tending_control::ToolAxis;

class Scenario1Inspect : public rclcpp::Node
{
public:
  Scenario1Inspect()
  : rclcpp::Node("scenario1_inspect")
  {
    control_mode_ = declare_parameter<std::string>("control_mode", "rule");   // rule | hybrid
    dry_run_ = declare_parameter<bool>("dry_run", true);
    use_line_ = declare_parameter<bool>("use_line", false);

    // 궤도 파라미터
    axis_point_ = declare_parameter<std::vector<double>>("axis_point", {0.4, 0.0, 0.4});
    axis_dir_ = declare_parameter<std::vector<double>>("axis_dir", {0.0, 0.0, -1.0});
    inspect_distance_ = declare_parameter<double>("inspect_distance", 0.10);
    angle_start_ = declare_parameter<double>("angle_start", 0.0);
    angle_end_ = declare_parameter<double>("angle_end", 2.0 * M_PI);
    num_views_ = declare_parameter<int>("num_views", 8);
    flip_up_ = declare_parameter<bool>("flip_up", false);

    // 모션 파라미터
    velocity_ = declare_parameter<double>("velocity", 0.4);          // PTP_T: rad/s 상당
    line_velocity_ = declare_parameter<double>("line_velocity", 0.05);// LINE_T: m/s
    acc_time_ms_ = declare_parameter<double>("acc_time_ms", 200.0);
    pos_tol_m_ = declare_parameter<double>("pos_tol_m", 0.003);
    ang_tol_rad_ = declare_parameter<double>("ang_tol_rad", 0.02);
    move_timeout_s_ = declare_parameter<int>("move_timeout_s", 30);
    settle_ms_ = declare_parameter<int>("settle_ms", 300);

    // hybrid: 태스크 프레임(cnc_frame) + 안전영역
    cnc_xyz_ = declare_parameter<std::vector<double>>("cnc_frame.xyz", {0.0, 0.0, 0.0});
    cnc_rpy_ = declare_parameter<std::vector<double>>("cnc_frame.rpy", {0.0, 0.0, 0.0});
    safe_min_ = declare_parameter<std::vector<double>>("safe_region.min_xyz", {-1.0, -1.0, -1.0});
    safe_max_ = declare_parameter<std::vector<double>>("safe_region.max_xyz", {1.0, 1.0, 1.0});

    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    bridge_ = std::make_unique<tending_control::RobotIoBridge>(this, cb_group_);
  }

  void run()
  {
    RCLCPP_INFO(get_logger(), "시나리오1 검사 시작 — mode=%s, dry_run=%s, views=%d",
      control_mode_.c_str(), dry_run_ ? "true" : "false", num_views_);

    // 태스크 축(base 기준) 결정.
    ToolAxis axis = resolve_axis();

    // 궤도 포즈 생성.
    auto poses = tending_control::scenario1Orbit(
      axis, inspect_distance_, angle_start_, angle_end_, num_views_, flip_up_);
    auto angles = tending_control::orbitAngles(angle_start_, angle_end_, num_views_);

    if (dry_run_) {
      RCLCPP_INFO(get_logger(), "[dry_run] 생성된 궤도 포즈(base→TCP):");
      for (size_t i = 0; i < poses.size(); ++i) {
        const bool in_safe = in_safe_region(poses[i].translation());
        RCLCPP_INFO(get_logger(), "  view %zu (θ=%.1f°): %s%s",
          i, angles[i] * 180.0 / M_PI, tending_control::prettyPose(poses[i]).c_str(),
          (control_mode_ == "hybrid" && !in_safe) ? "  [!! safe_region 이탈]" : "");
      }
      RCLCPP_INFO(get_logger(), "[dry_run] 완료 — 실제 이동은 dry_run:=false 로 실행.");
      return;
    }

    // 실제 이동 준비.
    if (!bridge_->wait_for_services(10s)) {
      RCLCPP_ERROR(get_logger(),
        "tm_driver 서비스 미검출. tmr_ros2 로 로봇을 먼저 bringup 하세요 "
        "(예: tm5-900_run_move_group.launch.py 또는 tm_bringup.launch.py).");
      return;
    }
    // 첫 상태 피드백 수신까지 대기(구독 시작 직후 레이스 방지).
    const auto sdead = std::chrono::steady_clock::now() + 5s;
    while (rclcpp::ok() && !bridge_->have_state()) {
      if (std::chrono::steady_clock::now() > sdead) {
        RCLCPP_ERROR(get_logger(), "로봇 상태(feedback_states) 미수신.");
        return;
      }
      std::this_thread::sleep_for(50ms);
    }
    std::string reason;
    if (!bridge_->safety_ok(reason)) {
      RCLCPP_ERROR(get_logger(), "사전 안전점검 실패: %s", reason.c_str());
      return;
    }

    for (size_t i = 0; i < poses.size(); ++i) {
      if (!rclcpp::ok()) return;
      const Eigen::Vector3d p = poses[i].translation();

      // hybrid: 안전영역 검증(룰베이스에는 없는 보호).
      if (control_mode_ == "hybrid" && !in_safe_region(p)) {
        RCLCPP_ERROR(get_logger(),
          "view %zu 목표가 safe_region 밖 → 중단 (%s)", i,
          tending_control::prettyPose(poses[i]).c_str());
        bridge_->stop();
        return;
      }

      auto tm = tending_control::toTmCartesian(poses[i]);
      RCLCPP_INFO(get_logger(), "view %zu (θ=%.1f°) 이동: %s",
        i, angles[i] * 180.0 / M_PI, tending_control::prettyPose(poses[i]).c_str());

      bool ok = use_line_
        ? bridge_->move_line_tool(tm, line_velocity_, acc_time_ms_,
            pos_tol_m_, ang_tol_rad_, std::chrono::seconds(move_timeout_s_))
        : bridge_->move_ptp_tool(tm, velocity_, acc_time_ms_,
            pos_tol_m_, ang_tol_rad_, std::chrono::seconds(move_timeout_s_));
      if (!ok) {
        RCLCPP_ERROR(get_logger(), "view %zu 이동 실패 → 중단", i);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms_));
      // (카메라 셋업 후) 여기서 capture 트리거. 현재는 로봇 단독 동작 검증.
    }
    RCLCPP_INFO(get_logger(), "시나리오1 검사 궤도 완료 (%zu 뷰).", poses.size());
  }

private:
  ToolAxis resolve_axis()
  {
    ToolAxis axis;
    Eigen::Vector3d pt(axis_point_[0], axis_point_[1], axis_point_[2]);
    Eigen::Vector3d dir(axis_dir_[0], axis_dir_[1], axis_dir_[2]);

    if (control_mode_ == "hybrid") {
      // axis_point/dir 를 cnc_frame(태스크 프레임) 기준으로 해석 → base 로 변환.
      Eigen::Isometry3d T = tending_control::fromTmCartesian(
        {cnc_xyz_[0], cnc_xyz_[1], cnc_xyz_[2], cnc_rpy_[0], cnc_rpy_[1], cnc_rpy_[2]});
      axis.point = T * pt;
      axis.dir = T.rotation() * dir;
      RCLCPP_INFO(get_logger(), "[hybrid] 태스크프레임 적용 후 축점(base)=[%.3f %.3f %.3f]",
        axis.point.x(), axis.point.y(), axis.point.z());
    } else {
      axis.point = pt;
      axis.dir = dir;
    }
    return axis;
  }

  bool in_safe_region(const Eigen::Vector3d & p) const
  {
    return p.x() >= safe_min_[0] && p.x() <= safe_max_[0] &&
           p.y() >= safe_min_[1] && p.y() <= safe_max_[1] &&
           p.z() >= safe_min_[2] && p.z() <= safe_max_[2];
  }

  std::string control_mode_;
  bool dry_run_, use_line_, flip_up_;
  std::vector<double> axis_point_, axis_dir_;
  double inspect_distance_, angle_start_, angle_end_;
  int num_views_;
  double velocity_, line_velocity_, acc_time_ms_, pos_tol_m_, ang_tol_rad_;
  int move_timeout_s_, settle_ms_;
  std::vector<double> cnc_xyz_, cnc_rpy_, safe_min_, safe_max_;

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::unique_ptr<tending_control::RobotIoBridge> bridge_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Scenario1Inspect>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin([&exec]() { exec.spin(); });
  node->run();
  rclcpp::shutdown();
  spin.join();
  return 0;
}
