cmake_minimum_required(VERSION 3.5)
project(tending_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tm_msgs REQUIRED)
find_package(tending_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

# TendingSystem(C#) GUI 연동 경계 노드. 와이어 프로토콜은 doc/PROTOCOL.md 참조.
#   - json_line_server : 개행 구분 JSON 라인 TCP 서버 (ROS 비의존, 순수 POSIX)
#   - tending_bridge   : ROS 상태/액션 ↔ 프로토콜 중계
add_executable(tending_bridge
  src/tending_bridge_node.cpp
  src/json_line_server.cpp
)
target_include_directories(tending_bridge PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(tending_bridge
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2
  tm_msgs
  tending_interfaces
)
target_link_libraries(tending_bridge nlohmann_json::nlohmann_json)

install(TARGETS tending_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config doc
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(PROGRAMS scripts/mock_ui_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
