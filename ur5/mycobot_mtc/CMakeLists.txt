cmake_minimum_required(VERSION 3.8)
project(mycobot_mtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =================================================================================
# Find All Required Dependencies
# #include directives in your C++ code.
# =================================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# =================================================================================
# Define the Executable with All Source Files
# =================================================================================
add_executable(mtc_node
    src/mtc_task_main.cpp
    src/mtc_task_callbacks.cpp
    src/mtc_task_scene.cpp
    src/mtc_task_creation.cpp
    src/mtc_task_execute.cpp
)

# =================================================================================
# Link ament/ROS 2 and other libraries to your executable
# linker to connect node with the libraries it needs to run.
# =================================================================================
ament_target_dependencies(mtc_node
    rclcpp
    moveit_ros_planning_interface
    moveit_task_constructor_core
    geometric_shapes
    tf2_ros
    tf2_geometry_msgs
    tf2_eigen
    std_msgs
    visualization_msgs
)


# =================================================================================
# Set Include Directories
# Adding the include directories from all the dependencies we found.
# =================================================================================
target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${moveit_task_constructor_core_INCLUDE_DIRS}
  ${geometric_shapes_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

target_compile_features(mtc_node PUBLIC cxx_std_17) # Set the C++ standard


# =================================================================================
# Installation Rules
# This section tells `colcon build`/`build` what files to install.
# =================================================================================

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS mtc_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
