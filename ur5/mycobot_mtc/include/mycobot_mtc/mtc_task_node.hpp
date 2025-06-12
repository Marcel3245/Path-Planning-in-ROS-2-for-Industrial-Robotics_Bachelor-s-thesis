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
    mtc::Task createTask(int workpiece_id, int target_place);

    /** @brief Executes the planning and execution of the MTC task. */
    void doTask();

    /** @brief Sets up the planning scene with collision objects. */
    void setupPlanningScene();

    /** @brief Looks up a transform from the TF tree. */    
    std::optional<geometry_msgs::msg::Pose> get_start_storage_pose();

    // --- Subscription Callback Methods ---
    void circles_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void storage_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void spawn_workpieces_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void run_mtc_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // --- ROS 2 and MTC Core Components ---
    rclcpp::Node::SharedPtr node_;
    mtc::Task task_;

    // --- Subscriptions and Publishers ---
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr circles_data_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr camera_data_storage_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spawn_workpieces_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr run_mtc_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // --- TF2 Components ---
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // --- State and Data Storage ---
    std::mutex data_mutex_; // Mutex to protect all shared data below
    
    // State flags
    bool workpieces_spawned_ = false;
    bool task_planned_executed_ = false;
    bool circles_data_received_ = false;
    bool storage_pose_data_received_ = false;

    // Data from callbacks (protected by data_mutex_)
    float data_array_[6][4] = {0}; // Moved from global scope
    float x_mid_storage_ = 0.0;
    float y_mid_storage_ = 0.0;
    float z_storage_ = 0.0;
    float alfa_ = 0.0; // Yaw angle for storage

    // Positions of the workpieces 
    float workpieces_positions_[6][3] = {0};
    
    // Poses used in the task
    std::optional<geometry_msgs::msg::Pose> start_storage_pose_;
};

#endif // MYCOBOT_MTC__MTC_TASK_NODE_HPP_