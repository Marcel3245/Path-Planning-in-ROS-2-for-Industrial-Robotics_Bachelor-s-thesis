#include "mycobot_mtc/mtc_task_node.hpp"

void MTCTaskNode::spawnWorkpiece() 
{
    RCLCPP_INFO(LOGGER, "Setting up planning scene...");
    moveit::planning_interface::PlanningSceneInterface psi;
    moveit_msgs::msg::CollisionObject workpiece;
    workpiece.id = "workpiece";
    workpiece.header.frame_id = "world";

    shape_msgs::msg::SolidPrimitive cylinder1, cylinder2;
    cylinder1.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder2.type = shape_msgs::msg::SolidPrimitive::CYLINDER;

    // Cylinder 1: Height = 0.033, Radius = 0.008
    cylinder1.dimensions.resize(2);
    cylinder1.dimensions[0] = 0.033;
    cylinder1.dimensions[1] = 0.008;

    // Cylinder 2: Height = 0.01, Radius = 0.021
    cylinder2.dimensions.resize(2);
    cylinder2.dimensions[0] = 0.01;
    cylinder2.dimensions[1] = 0.021;

    // Create poses
    geometry_msgs::msg::Pose cylinder1_pose;
    cylinder1_pose.orientation.w = 1.0;
    cylinder1_pose.position.z = (cylinder1.dimensions[0] / 2.0) + (cylinder2.dimensions[0] / 2.0); 

    geometry_msgs::msg::Pose cylinder2_pose;
    cylinder2_pose.orientation.w = 1.0;
    cylinder2_pose.position.z = 0;  // Move this cylinder downward

    // Add to collision object
    workpiece.primitives.push_back(cylinder1);
    workpiece.primitive_poses.push_back(cylinder1_pose);

    workpiece.primitives.push_back(cylinder2);
    workpiece.primitive_poses.push_back(cylinder2_pose);

    geometry_msgs::msg::Pose workpiece_pose;

    workpiece_pose.position.x = workpiece_position[0];
    workpiece_pose.position.y = workpiece_position[1];
    workpiece_pose.position.z = 0.7475;
    workpiece_pose.orientation.w = 1.0;


    workpiece.pose = workpiece_pose;
    workpiece.operation = moveit_msgs::msg::CollisionObject::ADD;

    psi.applyCollisionObject(workpiece); 
}