// pose_utils — 시나리오 1(손목 장착) 검사 포즈 기하 + TM/ROS 좌표 변환 유틸.
//
// 시나리오 1: 카메라 광축이 툴(엔드밀) 축에 수직. 카메라(=TCP)가 툴 축을 중심으로
// 반경 d(검사거리) 원을 그리며 각도 θ 만큼 궤도 이동하고, 광축은 항상 축을 향한다.
//
// 좌표 규약(tmr_ros2 와 일치):
//  - 위치: meter, 회전: tf2 setRPY(rx,ry,rz) = Rz*Ry*Rx (tool_pose 토픽 / SetPositions PTP_T 동일)
//  - TM 스크립트/SetPositions PTP_T positions = [x,y,z,rx,ry,rz] (m, rad) — 드라이버가 mm/deg 로 변환

#ifndef TENDING_CONTROL__POSE_UTILS_HPP_
#define TENDING_CONTROL__POSE_UTILS_HPP_

#include <vector>

#include <Eigen/Geometry>
#include "geometry_msgs/msg/pose.hpp"

namespace tending_control
{

// 툴(엔드밀) 축: 축 위의 한 점(검사 높이)과 단위 방향 벡터(예: 아래로 [0,0,-1]).
struct ToolAxis
{
  Eigen::Vector3d point{0.0, 0.0, 0.0};
  Eigen::Vector3d dir{0.0, 0.0, -1.0};
};

// 시나리오 1 궤도상의 한 뷰 포즈(base→TCP)를 계산.
//  axis     : 툴 축(base 기준)
//  distance : 축~카메라 반경(m)
//  angle    : 궤도 각도(rad)
//  flip_up  : 이미지 상하 반전(카메라 up 방향을 축의 반대로)
// 반환: base→TCP(=카메라 광학) 변환. Z_TCP 는 축을 향하고, Y_TCP 는 축 방향(±).
Eigen::Isometry3d scenario1ViewPose(
  const ToolAxis & axis, double distance, double angle, bool flip_up = false);

// 궤도 전체 뷰 포즈 목록 생성. angle_start..angle_end 를 num_views 로 등각 분할.
// full-circle(≈2π)이면 끝점 중복을 피하기 위해 step=range/num_views.
std::vector<Eigen::Isometry3d> scenario1Orbit(
  const ToolAxis & axis, double distance,
  double angle_start, double angle_end, int num_views, bool flip_up = false);

// 각 뷰의 궤도 각도값(생성 규칙과 동일). scenario1Orbit 와 짝.
std::vector<double> orbitAngles(double angle_start, double angle_end, int num_views);

// 변환 → TM SetPositions PTP_T/LINE_T positions [x,y,z,rx,ry,rz] (m, rad).
std::vector<double> toTmCartesian(const Eigen::Isometry3d & T);

// Eigen ↔ geometry_msgs::Pose
geometry_msgs::msg::Pose toPoseMsg(const Eigen::Isometry3d & T);
Eigen::Isometry3d fromPoseMsg(const geometry_msgs::msg::Pose & p);

// TM 6-value pose [x,y,z,rx,ry,rz] (m,rad) ↔ Eigen (feedback tool0_pose/tool_pose 파싱용)
Eigen::Isometry3d fromTmCartesian(const std::vector<double> & xyzrpy);

// 보기 좋은 문자열: "xyz=[..] m, rpy=[..] deg"
std::string prettyPose(const Eigen::Isometry3d & T);

}  // namespace tending_control

#endif  // TENDING_CONTROL__POSE_UTILS_HPP_
