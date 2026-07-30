cmake_minimum_required(VERSION 3.5)
project(tending_calibration)

find_package(ament_cmake REQUIRED)

# [P1 스캐폴드] 카메라 부착 후 구현 예정:
#   - handeye_collect : 여러 포즈에서 (플랜지 포즈 + 타깃 이미지) 수집
#   - handeye_solve   : cv::calibrateHandEye 로 flange->camera 산출, handeye.yaml 저장
#   - 의존: rclcpp, geometry_msgs, tf2, OpenCV(cv_bridge), tm_msgs
# 현재는 빌드 대상 없음(placeholder).

install(DIRECTORY doc
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
