#include "tending_control/pose_utils.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace tending_control
{

Eigen::Isometry3d scenario1ViewPose(
  const ToolAxis & axis, double distance, double angle, bool flip_up)
{
  const Eigen::Vector3d a = axis.dir.normalized();

  // 축에 수직인 기준 반경 방향 u0 (축과 나란하지 않은 임의 벡터로 생성).
  Eigen::Vector3d seed = (std::fabs(a.z()) < 0.9)
    ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
  Eigen::Vector3d u0 = (seed - seed.dot(a) * a).normalized();
  Eigen::Vector3d v0 = a.cross(u0).normalized();

  // 각도 θ 에서의 반경 방향과 카메라 위치.
  Eigen::Vector3d r = std::cos(angle) * u0 + std::sin(angle) * v0;
  Eigen::Vector3d pos = axis.point + distance * r;

  // 카메라 좌표축: Z=축을 향함(-r), Y=축 방향(±a), X=Y×Z.
  Eigen::Vector3d z_cam = -r;
  Eigen::Vector3d y_cam = flip_up ? -a : a;
  // y 를 z 에 직교화(수치 안정).
  y_cam = (y_cam - y_cam.dot(z_cam) * z_cam).normalized();
  Eigen::Vector3d x_cam = y_cam.cross(z_cam).normalized();

  Eigen::Matrix3d R;
  R.col(0) = x_cam;
  R.col(1) = y_cam;
  R.col(2) = z_cam;

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = R;
  T.translation() = pos;
  return T;
}

std::vector<double> orbitAngles(double angle_start, double angle_end, int num_views)
{
  std::vector<double> angles;
  if (num_views <= 0) return angles;
  const double range = angle_end - angle_start;
  const bool full = std::fabs(std::fabs(range) - 2.0 * M_PI) < 1e-3;
  double step = 0.0;
  if (num_views > 1) {
    step = full ? (range / num_views) : (range / (num_views - 1));
  }
  for (int i = 0; i < num_views; ++i) {
    angles.push_back(angle_start + step * i);
  }
  return angles;
}

std::vector<Eigen::Isometry3d> scenario1Orbit(
  const ToolAxis & axis, double distance,
  double angle_start, double angle_end, int num_views, bool flip_up)
{
  std::vector<Eigen::Isometry3d> poses;
  for (double a : orbitAngles(angle_start, angle_end, num_views)) {
    poses.push_back(scenario1ViewPose(axis, distance, a, flip_up));
  }
  return poses;
}

std::vector<double> toTmCartesian(const Eigen::Isometry3d & T)
{
  Eigen::Quaterniond q(T.rotation());
  tf2::Quaternion tq(q.x(), q.y(), q.z(), q.w());
  double rx, ry, rz;
  tf2::Matrix3x3(tq).getRPY(rx, ry, rz);
  const Eigen::Vector3d p = T.translation();
  return {p.x(), p.y(), p.z(), rx, ry, rz};
}

geometry_msgs::msg::Pose toPoseMsg(const Eigen::Isometry3d & T)
{
  geometry_msgs::msg::Pose msg;
  const Eigen::Vector3d p = T.translation();
  msg.position.x = p.x();
  msg.position.y = p.y();
  msg.position.z = p.z();
  Eigen::Quaterniond q(T.rotation());
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  return msg;
}

Eigen::Isometry3d fromPoseMsg(const geometry_msgs::msg::Pose & p)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  T.linear() = q.normalized().toRotationMatrix();
  return T;
}

Eigen::Isometry3d fromTmCartesian(const std::vector<double> & xyzrpy)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  if (xyzrpy.size() != 6) return T;
  tf2::Quaternion tq;
  tq.setRPY(xyzrpy[3], xyzrpy[4], xyzrpy[5]);
  Eigen::Quaterniond q(tq.w(), tq.x(), tq.y(), tq.z());
  T.linear() = q.toRotationMatrix();
  T.translation() = Eigen::Vector3d(xyzrpy[0], xyzrpy[1], xyzrpy[2]);
  return T;
}

std::string prettyPose(const Eigen::Isometry3d & T)
{
  auto v = toTmCartesian(T);
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(4);
  ss << "xyz=[" << v[0] << ", " << v[1] << ", " << v[2] << "] m, rpy=["
     << v[3] * 180.0 / M_PI << ", " << v[4] * 180.0 / M_PI << ", "
     << v[5] * 180.0 / M_PI << "] deg";
  return ss.str();
}

}  // namespace tending_control
