cmake_minimum_required(VERSION 3.5)
project(tending_bringup)

find_package(ament_cmake REQUIRED)

# [골격] 카메라·링라이트 부착 TM5 셀 bringup 런치를 여기에 추가 예정.
#   로봇/MoveIt/RViz 브링업은 tmr_ros2 공식 런치(tm5-900_run_move_group 등)를 그대로 사용하고,
#   본 패키지는 카메라 부착 확장 description 을 얹은 bringup 만 담당한다.
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
