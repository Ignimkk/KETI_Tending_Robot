cmake_minimum_required(VERSION 3.5)
project(tending_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
