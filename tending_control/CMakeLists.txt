cmake_minimum_required(VERSION 3.5)
project(tending_control)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tm_msgs REQUIRED)
find_package(tending_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# 핵심 라이브러리: 기하 유틸(pose_utils) + tm_driver 래퍼(robot_io_bridge)
add_library(tending_control_core
  src/pose_utils.cpp
  src/robot_io_bridge.cpp)
ament_target_dependencies(tending_control_core
  rclcpp geometry_msgs tm_msgs tf2 tf2_geometry_msgs)
target_link_libraries(tending_control_core Eigen3::Eigen)
target_include_directories(tending_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 검사 상태기계/액션
add_executable(inspection_manager src/inspection_manager.cpp)
target_link_libraries(inspection_manager tending_control_core)
ament_target_dependencies(inspection_manager
  rclcpp rclcpp_action geometry_msgs sensor_msgs tm_msgs tending_interfaces)

# 시나리오1 제어 로직 (rule / hybrid)
add_executable(scenario1_inspect src/scenario1_inspect.cpp)
target_link_libraries(scenario1_inspect tending_control_core Eigen3::Eigen)
ament_target_dependencies(scenario1_inspect rclcpp geometry_msgs tm_msgs)

# EE 포즈 추출 유틸 (TF base→ee + /joint_states)
add_executable(ee_pose_query src/ee_pose_query.cpp)
target_link_libraries(ee_pose_query tending_control_core Eigen3::Eigen)
ament_target_dependencies(ee_pose_query
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_eigen)

install(TARGETS tending_control_core
  EXPORT export_tending_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS inspection_manager scenario1_inspect ee_pose_query
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# 다운스트림(tending_moveit 등)에서 pose_utils 재사용을 위한 export
ament_export_include_directories(include)
ament_export_targets(export_tending_control HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs tm_msgs tf2 tf2_geometry_msgs Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_utils test/test_pose_utils.cpp)
  target_link_libraries(test_pose_utils tending_control_core Eigen3::Eigen)
  ament_target_dependencies(test_pose_utils geometry_msgs tf2)
endif()

ament_package()
