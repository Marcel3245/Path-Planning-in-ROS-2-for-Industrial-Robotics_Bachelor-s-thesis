#include "mycobot_mtc/mtc_task_node.hpp"

std::optional<geometry_msgs::msg::Pose> MTCTaskNode::get_start_storage_pose()
{
    const std::string target_frame = "world";
    const std::string source_frame = "start_storage";

    geometry_msgs::msg::TransformStamped transform_stamped;

    try {
        transform_stamped = tf_buffer_->lookupTransform(
            target_frame, source_frame,
            tf2::TimePointZero, 5s);

        geometry_msgs::msg::Pose pose;

        pose.position.x = transform_stamped.transform.translation.x;
        pose.position.y = transform_stamped.transform.translation.y;
        pose.position.z = transform_stamped.transform.translation.z;

        pose.orientation = transform_stamped.transform.rotation;

        RCLCPP_INFO(LOGGER, "TF Lookup successful for %s to %s. Position: (%.3f, %.3f, %.3f), Orientation: (%.3f, %.3f, %.3f, %.3f)",
                    source_frame.c_str(), target_frame.c_str(),
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        return pose;

    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(LOGGER, "Could not transform %s to %s: %s",
                    source_frame.c_str(), target_frame.c_str(), ex.what());
        return std::nullopt;
    }
}


void MTCTaskNode::setupPlanningScene()
{
    RCLCPP_INFO(LOGGER, "Setting up planning scene...");
    moveit::planning_interface::PlanningSceneInterface psi;

    if (start_storage_pose_) {
        const geometry_msgs::msg::Pose& storage_pose = start_storage_pose_.value();

        tf2::Quaternion q(
            storage_pose.orientation.x,
            storage_pose.orientation.y,
            storage_pose.orientation.z,
            storage_pose.orientation.w);

        double yaw = tf2::getYaw(q);

        std::vector<float> x_diff;
        std::vector<float> y_diff;
        float z_offset;

        const double PI = M_PI;
        bool is_yaw_0_or_180 = std::abs(yaw) < 0.1 || std::abs(yaw - PI) < 0.1 || std::abs(yaw + PI) < 0.1;
        bool is_yaw_plus_minus_90 = std::abs(yaw - PI/2.0) < 0.1 || std::abs(yaw + PI/2.0) < 0.1;


        if (is_yaw_0_or_180) {
            RCLCPP_INFO(LOGGER, "Detected TF Yaw close to 0 or 180 degrees (%.2f rad) for smaller objects layout.", yaw);
            x_diff = {-0.0325, -0.0325, -0.0325, 0.0325, 0.0325, 0.0325};
            y_diff = {-0.06, 0, 0.06, -0.06, 0, 0.06};
            z_offset = 0.925; // This Z offset seems related to the TF's height
        } else if (is_yaw_plus_minus_90) {
             RCLCPP_INFO(LOGGER, "Detected TF Yaw close to 90 or -90 degrees (%.2f rad) for smaller objects layout.", yaw);
             x_diff = {-0.06, 0, 0.06, -0.06, 0, 0.06};
             y_diff = {-0.0325, -0.0325, -0.0325, 0.0325, 0.0325, 0.0325};
             z_offset = 0.925; // This Z offset seems related to the TF's height
        }
         else {
             RCLCPP_WARN(LOGGER, "Detected TF Yaw is %.2f rad, which is not close to 0, 90, 180, or -90 degrees. Cannot determine smaller object layout based on TF.", yaw);
             // Do not add smaller objects if TF orientation is unexpected
             return;
        }

        if (x_diff.size() != 6 || y_diff.size() != 6) {
             RCLCPP_ERROR(LOGGER, "Internal error: Diff vector sizes are incorrect (%zu, %zu). Cannot add smaller collision objects.", x_diff.size(), y_diff.size());
             return;
        }

        std::vector<moveit_msgs::msg::CollisionObject> objects_to_add;
        objects_to_add.reserve(6); // Reserve space for efficiency

        for (size_t i = 0; i < 6; ++i) {
            moveit_msgs::msg::CollisionObject object;
            geometry_msgs::msg::Pose object_pose;

            object.id = "object_" + std::to_string(i);
            object.header.frame_id = "world";

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
            object.primitives.push_back(cylinder1);
            object.primitive_poses.push_back(cylinder1_pose);

            object.primitives.push_back(cylinder2);
            object.primitive_poses.push_back(cylinder2_pose);


            // Calculate position relative to the TF frame pose
            if (is_yaw_plus_minus_90) {
                object_pose.position.x = storage_pose.position.x + x_diff[i];
                object_pose.position.y = storage_pose.position.y + y_diff[i];
            } else { // Assuming is_yaw_0_or_180
                 object_pose.position.x = storage_pose.position.x - y_diff[i];
                 object_pose.position.y = storage_pose.position.y + x_diff[i];
            }

            object_pose.position.z = (cylinder2.dimensions[0] / 2.0) + z_offset; // Z relative to TF Z base + cylinder half height

            object_pose.orientation = storage_pose.orientation; // Inherit orientation from TF

            object.pose = object_pose;

            object.operation = moveit_msgs::msg::CollisionObject::ADD;

            objects_to_add.push_back(object);

            RCLCPP_INFO(LOGGER, "Adding object %zu at (%.3f, %.3f, %.3f) based on TF.",
                        i, object_pose.position.x, object_pose.position.y, object_pose.position.z);
        }

        if (!objects_to_add.empty()) {
             psi.applyCollisionObjects(objects_to_add);
             RCLCPP_INFO(LOGGER, "Applied %zu smaller collision objects based on TF.", objects_to_add.size());
        } else {
             RCLCPP_WARN(LOGGER, "No smaller collision objects were generated to apply based on TF.");
        }

    } else {
        RCLCPP_WARN(LOGGER, "Start storage TF pose not available. Cannot add smaller collision objects.");
    }
}