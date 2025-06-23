#ifndef MYCOBOT_MTC__MTC_TASK_NODE_HPP_
#define MYCOBOT_MTC__MTC_TASK_NODE_HPP_

#include "mycobot_mtc/mtc_task_common.hpp"

/**
 * @class MTCTaskNode
 * @brief Manages MoveIt Task Constructor tasks for a pick-and-place operation.
 *
 * This class sets up the ROS 2 node, subscribes to camera data and button inputs,
 * manages the planning scene, and creates/executes MTC tasks.
 */

class MTCTaskNode
{
public:
    /**
    * @brief Constructor for the MTCTaskNode.
    * @param options ROS 2 NodeOptions for configuring the node.
    */
    MTCTaskNode(const rclcpp::NodeOptions& options);

    /**
    * @brief Provides access to the underlying ROS 2 node interface for the executor.
    * @return A shared pointer to the node base interface.
    */
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

private:
    // --- Core MTC and Planning Scene Methods ---
    /** @brief Creates the full MTC pick-and-place task pipeline. */
    mtc::Task createTask();

    /** @brief Executes the planning and execution of the MTC task. */
    void doTask();

    /** @brief Sets up the planning scene with collision workpiece. */
    void spawnWorkpiece();

    /** @brief Looks up a transform from the TF tree. */    
    std::optional<geometry_msgs::msg::Pose> get_start_storage_pose();

    // --- Subscription Callback Methods ---
    void workpiece_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    
    // --- ROS 2 and MTC Core Components ---
    rclcpp::Node::SharedPtr node_;
    mtc::Task task_;

    // --- Subscriptions and Publishers ---
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_workpiece_position;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_terminal_info;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_robot_active;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_camera_active;

    // State flags
    float workpiece_position[3] = {0.0, 0.0, 0.0};
};

#endif // MYCOBOT_MTC__MTC_TASK_NODE_HPP_