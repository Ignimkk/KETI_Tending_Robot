// pose_utils 기하 오프라인 검증 (로봇 불필요).
#include <cmath>
#include <gtest/gtest.h>

#include "tending_control/pose_utils.hpp"

using namespace tending_control;

TEST(PoseUtils, ViewPoseDistanceAndLookAt)
{
  ToolAxis axis;
  axis.point = Eigen::Vector3d(0.4, 0.0, 0.4);
  axis.dir = Eigen::Vector3d(0.0, 0.0, -1.0);  // 아래로 매달린 엔드밀
  const double d = 0.10;

  for (double theta : {0.0, M_PI / 3, M_PI, 3.0 * M_PI / 2}) {
    auto T = scenario1ViewPose(axis, d, theta, false);
    const Eigen::Vector3d pos = T.translation();

    // 1) 축까지 반경 거리가 d 와 일치(축이 수직이므로 xy 평면 거리).
    Eigen::Vector3d rel = pos - axis.point;
    Eigen::Vector3d a = axis.dir.normalized();
    Eigen::Vector3d radial = rel - rel.dot(a) * a;  // 축에 수직인 성분
    EXPECT_NEAR(radial.norm(), d, 1e-9);

    // 2) 카메라 Z축(광축)이 축을 향함: -Z 가 반경 방향과 정렬.
    Eigen::Vector3d z_cam = T.rotation().col(2);
    EXPECT_NEAR((-z_cam).dot(radial.normalized()), 1.0, 1e-9);

    // 3) 회전행렬이 정규직교(det=+1).
    EXPECT_NEAR(T.rotation().determinant(), 1.0, 1e-9);
  }
}

TEST(PoseUtils, TmCartesianRoundTrip)
{
  ToolAxis axis;
  axis.point = Eigen::Vector3d(0.3, -0.1, 0.5);
  axis.dir = Eigen::Vector3d(0.0, 0.0, -1.0);
  auto T = scenario1ViewPose(axis, 0.12, M_PI / 4, false);

  auto tm = toTmCartesian(T);         // [x,y,z,rx,ry,rz]
  auto T2 = fromTmCartesian(tm);

  EXPECT_TRUE(T.translation().isApprox(T2.translation(), 1e-9));
  // 회전 왕복(오차각 작음)
  Eigen::AngleAxisd aa(T.rotation().transpose() * T2.rotation());
  EXPECT_NEAR(std::fabs(aa.angle()), 0.0, 1e-6);
}

TEST(PoseUtils, OrbitFullCircleNoDuplicateEndpoint)
{
  auto angles = orbitAngles(0.0, 2.0 * M_PI, 8);
  ASSERT_EQ(angles.size(), 8u);
  EXPECT_NEAR(angles.front(), 0.0, 1e-12);
  // full-circle 는 끝점(2π)을 포함하지 않아야 함(중복 방지).
  EXPECT_LT(angles.back(), 2.0 * M_PI - 1e-6);
  EXPECT_NEAR(angles[1] - angles[0], 2.0 * M_PI / 8, 1e-12);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
