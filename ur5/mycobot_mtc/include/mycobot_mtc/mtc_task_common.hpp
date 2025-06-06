#ifndef MYCOBOT_MTC__MTC_TASK_COMMON_HPP_
#define MYCOBOT_MTC__MTC_TASK_COMMON_HPP_

#include <rclcpp/rclcpp.hpp>

// === Standard Library Headers ===
#include <chrono>
#include <memory>
#include <mutex>
#include <vector>
#include <string>
#include <array>
#include <limits> // Required for numeric_limits
#include <math.h> // For PI


// === MoveIt and Geometry Headers ===
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

// === TF2 Headers ===
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

// === ROS Message Type Headers ===
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// === Namespace Aliases and Constants ===
namespace mtc = moveit::task_constructor;
using namespace std::chrono_literals;

// Logger definition
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mycobot_mtc");

#endif  // MYCOBOT_MTC__MTC_TASK_COMMON_HPP_