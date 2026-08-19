// scenario1_inspect_moveit — 시나리오 1 검사의 MoveIt 2 기반(충돌회피) 제어 로직. [제어방식 (2)]
//
// tending_control::pose_utils 로 궤도(base→flange 목표) 포즈를 생성하고, MoveGroupInterface 로
// 각 뷰에 계획·실행한다. 계획기(OMPL/CHOMP/Pilz)와 planner_id 는 파라미터로 선택.
// 충돌 환경(상부 척/CNC)은 PlanningScene 에 추가하면 자동 회피된다(TODO: description 반영).
//
// 실행 전제: tm5-700_moveit_config 의 move_group 이 떠 있어야 한다.
//   ros2 launch tm5-700_moveit_config tm5-700_run_move_group.launch.py   (실물/시뮬)
//
// 참고: 이 노드는 move_group 이 필요하므로 오프라인 dry_run 은 제공하지 않는다
//       (룰/하이브리드의 오프라인 검증은 tending_control::scenario1_inspect 사용).

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "tending_control/pose_utils.hpp"

using moveit::planning_interface::MoveGroupInterface;
using tending_control::ToolAxis;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "scenario1_inspect_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = node->get_logger();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin([&exec]() { exec.spin(); });

  // ---- 파라미터 ----
  auto get_s = [&](const std::string & n, const std::string & d) {
    return node->has_parameter(n) ? node->get_parameter(n).as_string() : d; };
  auto get_d = [&](const std::string & n, double d) {
    return node->has_parameter(n) ? node->get_parameter(n).as_double() : d; };
  auto get_i = [&](const std::string & n, int d) {
    return node->has_parameter(n) ? static_cast<int>(node->get_parameter(n).as_int()) : d; };
  auto get_v = [&](const std::string & n, std::vector<double> d) {
    return node->has_parameter(n) ? node->get_parameter(n).as_double_array() : d; };

  const std::string group = get_s("group", "tmr_arm");
  const std::string pipeline = get_s("planning_pipeline", "ompl");     // ompl|chomp|pilz_industrial_motion_planner
  const std::string planner_id = get_s("planner_id", "RRTConnectkConfigDefault");
  const std::string ref_frame = get_s("reference_frame", "base");
  const std::string ee_link = get_s("ee_link", "flange");
  const double vel_scale = get_d("velocity_scaling", 0.1);
  const double acc_scale = get_d("accel_scaling", 0.1);
  const double plan_time = get_d("planning_time", 5.0);

  ToolAxis axis;
  auto ap = get_v("axis_point", {0.4, 0.0, 0.4});
  auto ad = get_v("axis_dir", {0.0, 0.0, -1.0});
  axis.point = Eigen::Vector3d(ap[0], ap[1], ap[2]);
  axis.dir = Eigen::Vector3d(ad[0], ad[1], ad[2]);
  const double distance = get_d("inspect_distance", 0.10);
  const double a0 = get_d("angle_start", 0.0);
  const double a1 = get_d("angle_end", 2.0 * M_PI);
  const int n = get_i("num_views", 8);
  const bool flip_up = node->has_parameter("flip_up") && node->get_parameter("flip_up").as_bool();

  // ---- MoveGroup 설정 ----
  MoveGroupInterface mg(node, group);
  mg.setPlanningPipelineId(pipeline);
  mg.setPlannerId(planner_id);
  mg.setPoseReferenceFrame(ref_frame);
  mg.setEndEffectorLink(ee_link);
  mg.setMaxVelocityScalingFactor(vel_scale);
  mg.setMaxAccelerationScalingFactor(acc_scale);
  mg.setPlanningTime(plan_time);

  RCLCPP_INFO(logger, "MoveIt 검사: group=%s pipeline=%s planner=%s ref=%s ee=%s views=%d",
    group.c_str(), pipeline.c_str(), planner_id.c_str(), ref_frame.c_str(), ee_link.c_str(), n);

  auto poses = tending_control::scenario1Orbit(axis, distance, a0, a1, n, flip_up);
  auto angles = tending_control::orbitAngles(a0, a1, n);

  int ok_count = 0;
  for (size_t i = 0; i < poses.size(); ++i) {
    if (!rclcpp::ok()) break;
    auto target = tending_control::toPoseMsg(poses[i]);
    mg.setPoseTarget(target, ee_link);

    MoveGroupInterface::Plan plan;
    RCLCPP_INFO(logger, "view %zu (θ=%.1f°) 계획 중... %s",
      i, angles[i] * 180.0 / M_PI, tending_control::prettyPose(poses[i]).c_str());
    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "view %zu 계획 실패(도달불가/충돌). 다음 뷰로 건너뜀.", i);
      continue;
    }
    if (mg.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "view %zu 실행 실패 → 중단.", i);
      break;
    }
    ++ok_count;
    // (카메라 셋업 후) 여기서 capture 트리거.
  }

  RCLCPP_INFO(logger, "MoveIt 검사 완료: %d/%zu 뷰 성공.", ok_count, poses.size());
  rclcpp::shutdown();
  spin.join();
  return 0;
}
