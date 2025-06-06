#include "mycobot_mtc/mtc_task_node.hpp"
#include <thread>

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  RCLCPP_INFO(LOGGER, "Initializing MTCTaskNode...");

  // --- Initialize Subscriptions ---
  circles_data_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "camera/circles_to_world", 10, std::bind(&MTCTaskNode::circles_data_callback, this, std::placeholders::_1));

  camera_data_storage_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "camera/storage_to_world", 10, std::bind(&MTCTaskNode::storage_data_callback, this, std::placeholders::_1));

  spawn_workpieces_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "button1", 10, std::bind(&MTCTaskNode::spawn_workpieces_callback, this, std::placeholders::_1));

  run_mtc_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "button2", 10, std::bind(&MTCTaskNode::run_mtc_callback, this, std::placeholders::_1));

  // --- Initialize Publishers ---
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "mtc_node/visualization_marker_array", 10);

  // --- Initialize TF Components ---
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // --- Initialize State and Pose Variables ---
  target_place_pose_.header.frame_id = "world";
  target_place_pose_.pose.orientation.w = 1.0;

  // Perform initial TF lookup for the storage pose
  start_storage_pose_ = get_start_storage_pose();

  RCLCPP_INFO(LOGGER, "MTCTaskNode initialized. Waiting for button presses and camera data.");
}

// Node interface implementation
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

// Main function to run the node
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  RCLCPP_INFO(LOGGER, "MTC node is spinning and waiting for events...");

  spin_thread->join();

  rclcpp::shutdown();
  return 0;
}