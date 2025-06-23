#include "mycobot_mtc/mtc_task_node.hpp"

void MTCTaskNode::workpiece_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    workpiece_position[0] = msg->data[0];
    workpiece_position[1] = msg->data[1];
    workpiece_position[2] = msg->data[2];

    spawnWorkpiece();

    // Create a message for the camera status
    auto camera_msg = std_msgs::msg::Bool();
    camera_msg.data = false;
    publisher_camera_active->publish(camera_msg);

    // Create a message for the robot status
    auto robot_msg = std_msgs::msg::Bool();
    robot_msg.data = true; 
    publisher_robot_active->publish(robot_msg);

    doTask();
}