
cmake_minimum_required(VERSION 3.8)
project(mycobot_bringup)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()