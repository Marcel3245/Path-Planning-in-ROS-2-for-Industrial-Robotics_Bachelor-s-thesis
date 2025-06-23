#include "mycobot_mtc/mtc_task_node.hpp"
#include <thread>

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  RCLCPP_INFO(LOGGER, "Initializing MTCTaskNode...");

  // --- Initialize Subscriptions ---
  subscriber_workpiece_position = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "camera/workpiece/position", 10, std::bind(&MTCTaskNode::workpiece_callback, this, std::placeholders::_1));

  // --- Initialize Publishers ---
  publisher_terminal_info = node_->create_publisher<std_msgs::msg::String>("terminal/info", 10);
  publisher_robot_active = node_->create_publisher<std_msgs::msg::Bool>("robot/active", 10);
  publisher_camera_active = node_->create_publisher<std_msgs::msg::Bool>("camera/active", 10);


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