cmake_minimum_required(VERSION 3.8)
project(mycobot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install (
  DIRECTORY config meshes urdf 
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  
endif()

ament_package()
