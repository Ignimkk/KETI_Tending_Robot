// ee_pose_query — base 기준 EE 포즈 + 관절값 추적 유틸리티.
//
// 용도: MoveIt RViz 의 interactive marker 로 로봇을 수동으로 옮겨가며(수동 티칭),
//   이 유틸이 1초(기본)마다 EE 포즈와 joint state 를 "정리된 형태"로 출력한다.
//   rule 기반 제어의 목표 지점(TM Cartesian 포즈 / axis_point / 관절 포즈)을 그대로
//   복사해 scenario1.yaml, poses.yaml 에 붙여넣을 수 있다.
//
// ★ 포즈 소스 = TF (base → ee_frame). robot_state_publisher 가 joint_states 로부터
//   publish 하는 TF 를 쓰므로 RViz 표시와 정확히 일치하고, fake 로봇(tm_driver 가
//   Cartesian 피드백을 계산하지 않는 경우)에서도 올바른 값을 얻는다.
//   관절값 = /joint_states.
//
// 사용:
//   ros2 run tending_control ee_pose_query                              # 1Hz 연속
//   ros2 run tending_control ee_pose_query --ros-args -p once:=true
//   ros2 run tending_control ee_pose_query --ros-args -p ee_frame:=flange -p rate_hz:=2.0
//
// 좌표 규약: 위치 m, 회전 rpy = tf2 getRPY (tool_pose / SetPositions PTP_T 와 동일 규약).

#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include "tending_control/pose_utils.hpp"

namespace
{
std::string fmtVec(const std::vector<double> & v, const char * fmt)
{
  std::string s = "[";
  char buf[48];
  for (size_t i = 0; i < v.size(); ++i) {
    std::snprintf(buf, sizeof(buf), fmt, v[i]);
    s += buf;
    if (i + 1 < v.size()) s += ", ";
  }
  s += "]";
  return s;
}
}  // namespace

class EePoseQuery : public rclcpp::Node
{
public:
  EePoseQuery()
  : rclcpp::Node("ee_pose_query")
  {
    once_ = declare_parameter<bool>("once", false);
    double rate = declare_parameter<double>("rate_hz", 1.0);
    base_frame_ = declare_parameter<std::string>("base_frame", "base");
    ee_frame_ = declare_parameter<std::string>("ee_frame", "flange");
    joint_order_ = declare_parameter<std::vector<std::string>>(
      "joint_order",
      {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr m) {
        for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) {
          joint_map_[m->name[i]] = m->position[i];
        }
        have_joints_ = !joint_map_.empty();
      });

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, rate));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&EePoseQuery::tick, this));

    RCLCPP_INFO(get_logger(),
      "ee_pose_query 시작 (base='%s' → ee='%s', rate=%.1fHz, once=%s). 수신 대기...",
      base_frame_.c_str(), ee_frame_.c_str(), rate, once_ ? "true" : "false");
  }

private:
  std::vector<double> ordered_joints() const
  {
    std::vector<double> q;
    for (const auto & n : joint_order_) {
      auto it = joint_map_.find(n);
      if (it != joint_map_.end()) q.push_back(it->second);
    }
    return q;
  }

  void tick()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF %s→%s 대기 중: %s", base_frame_.c_str(), ee_frame_.c_str(), e.what());
      return;
    }

    const Eigen::Isometry3d T = tf2::transformToEigen(tf);
    const auto tm6 = tending_control::toTmCartesian(T);
    const std::vector<double> xyz(tm6.begin(), tm6.begin() + 3);

    std::string o = "\n────────── EE state: '" + base_frame_ + "' → '" + ee_frame_ + "' ──────────";
    if (have_joints_) {
      auto q = ordered_joints();
      std::vector<double> deg(q.size());
      for (size_t i = 0; i < q.size(); ++i) deg[i] = q[i] * 180.0 / M_PI;
      o += "\n joints[rad] : " + fmtVec(q, "%+7.4f");
      o += "\n joints[deg] : " + fmtVec(deg, "%+7.2f");
    }
    o += "\n EE pose     : " + tending_control::prettyPose(T);
    o += "\n ── rule 목표 복붙용 (m, rad) ──";
    o += "\n  pose[x,y,z,rx,ry,rz] : " + fmtVec(tm6, "%+.4f");
    o += "\n  axis_point [x,y,z]   : " + fmtVec(xyz, "%+.4f");
    if (have_joints_) {
      o += "\n  poses.<name> (joints): " + fmtVec(ordered_joints(), "%+.4f");
    }
    o += "\n──────────────────────────────────────────────────";
    RCLCPP_INFO(get_logger(), "%s", o.c_str());

    if (once_) {
      rclcpp::shutdown();
    }
  }

  bool once_;
  std::string base_frame_, ee_frame_;
  std::vector<std::string> joint_order_;
  bool have_joints_{false};
  std::map<std::string, double> joint_map_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EePoseQuery>());
  rclcpp::shutdown();
  return 0;
}
