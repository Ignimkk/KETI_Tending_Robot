cmake_minimum_required(VERSION 3.5)
project(tending_moveit)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)   # MoveIt 2 는 C++17
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tending_control REQUIRED)

add_executable(scenario1_inspect_moveit src/scenario1_inspect_moveit.cpp)
ament_target_dependencies(scenario1_inspect_moveit
  rclcpp geometry_msgs moveit_ros_planning_interface moveit_core tending_control)

install(TARGETS scenario1_inspect_moveit
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
