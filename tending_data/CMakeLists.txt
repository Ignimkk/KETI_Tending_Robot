cmake_minimum_required(VERSION 3.5)
project(tending_data)

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
find_package(tending_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(dataset_recorder src/dataset_recorder.cpp)
ament_target_dependencies(dataset_recorder
  rclcpp
  tending_interfaces
  geometry_msgs
)

install(TARGETS dataset_recorder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
