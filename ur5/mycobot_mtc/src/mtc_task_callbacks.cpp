#include "mycobot_mtc/mtc_task_node.hpp"

void MTCTaskNode::circles_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    const size_t expected_elements = 6 * 4;
    if (msg->data.size() < expected_elements) {
        RCLCPP_WARN(LOGGER, "Received circles data array is smaller than expected (got %zu, expected %zu elements)", msg->data.size(), expected_elements);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t i = 0; i < 6; ++i) {
            for (size_t j = 0; j < 4; ++j) {
                size_t index = i * 4 + j;
                if (index < msg->data.size()) {
                    data_array_[i][j] = msg->data[index];
                } else {
                     RCLCPP_ERROR(LOGGER, "Unexpected index out of bounds during circle data copy.");
                     break;
                }
            }
        }
        circles_data_received_ = true; // Indicate circles data received
    } // End mutex scope

    // Marker visualization remains the same (for circles)
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.resize(6);

    for (size_t i = 0; i < 6; ++i) {
        marker_array.markers[i].header.frame_id = "world";
        marker_array.markers[i].header.stamp = node_->now();
        marker_array.markers[i].ns = "circles_" + std::to_string(i);
        marker_array.markers[i].id = i;
        marker_array.markers[i].type = visualization_msgs::msg::Marker::CYLINDER;
        marker_array.markers[i].action = visualization_msgs::msg::Marker::ADD;
        marker_array.markers[i].pose.position.x = data_array_[i][0];
        marker_array.markers[i].pose.position.y = data_array_[i][1];
        marker_array.markers[i].pose.position.z = data_array_[i][2];
        marker_array.markers[i].pose.orientation.x = 0.0;
        marker_array.markers[i].pose.orientation.y = 0.0;
        marker_array.markers[i].pose.orientation.z = 0.0;
        marker_array.markers[i].pose.orientation.w = 1.0;
        marker_array.markers[i].scale.x = 0.043; // Diameter
        marker_array.markers[i].scale.y = 0.043; // Diameter
        marker_array.markers[i].scale.z = 0.001; // Height
        if (data_array_[i][3] == 0){
            marker_array.markers[i].color.r = 0.0f;
            marker_array.markers[i].color.g = 1.0f;
            marker_array.markers[i].color.b = 0.0f;
        } else {
            marker_array.markers[i].color.r = 1.0f;
            marker_array.markers[i].color.g = 0.0f;
            marker_array.markers[i].color.b = 0.0f;
        }
        marker_array.markers[i].color.a = 1.0f;
        marker_array.markers[i].lifetime = rclcpp::Duration(0, 0);
    }

    marker_pub_->publish(marker_array);
}


void MTCTaskNode::storage_data_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Expected data format: [x, y, z, yaw]
    if (msg->data.size() < 4) {
        storage_pose_data_received_ = false; // Data not valid
        return;
    }
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // Store the received pose data in class members
        x_mid_storage_ = msg->data[0];
        y_mid_storage_ = msg->data[1];
        z_storage_ = msg->data[2];
        alfa_ = msg->data[3]; 
        storage_pose_data_received_ = true; // Indicate valid storage data received
    } // End mutex scope

    // --- Visualization Part ---
    visualization_msgs::msg::Marker storage_marker;
    storage_marker.header.frame_id = "world";
    storage_marker.header.stamp = node_->now();
    storage_marker.ns = "storage_object";
    storage_marker.id = 0; // Unique ID for this marker namespace
    storage_marker.action = visualization_msgs::msg::Marker::ADD;
    storage_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE; // Use MESH_RESOURCE
    storage_marker.mesh_resource = "package://mycobot_gazebo/models/storage/collision/storage.stl"; // Path to the mesh

    // Set the pose using the received camera data
    {
         std::lock_guard<std::mutex> lock(data_mutex_); // Lock again to access stored data
         storage_marker.pose.position.x = x_mid_storage_;
         storage_marker.pose.position.y = y_mid_storage_;
         storage_marker.pose.position.z = z_storage_; // Using the Z from camera data

         // Convert yaw (alfa_) to quaternion
         tf2::Quaternion quaternion;
         quaternion.setRPY(0, 0, alfa_); // Set roll=0, pitch=0, yaw=alfa_
         storage_marker.pose.orientation.x = quaternion.x();
         storage_marker.pose.orientation.y = quaternion.y();
         storage_marker.pose.orientation.z = quaternion.z();
         storage_marker.pose.orientation.w = quaternion.w();
    }

    // Set scale (often 1,1,1 if the mesh has correct units)
    storage_marker.scale.x = 1.0;
    storage_marker.scale.y = 1.0;
    storage_marker.scale.z = 1.0;

    // Set color (e.g., a light grey)
    storage_marker.color.r = 0.8f;
    storage_marker.color.g = 0.8f;
    storage_marker.color.b = 0.8f;
    storage_marker.color.a = 0.7f; // Slightly transparent

    // Set lifetime so it doesn't disappear immediately
    storage_marker.lifetime = rclcpp::Duration(0, 0); // Stays for 2 seconds, then refreshed

    // Publish the marker (wrap in MarkerArray)
    visualization_msgs::msg::MarkerArray marker_array_storage;
    marker_array_storage.markers.push_back(storage_marker);
    marker_pub_->publish(marker_array_storage);
}

// Callback for button1 (Spawn Workpieces/Setup Scene)
void MTCTaskNode::spawn_workpieces_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        if (!workpieces_spawned_) {
            // Require storage pose data before setting up the scene
            if (!storage_pose_data_received_) {
                 RCLCPP_WARN(LOGGER, "Received button1 press, but storage pose data has not been received yet. Cannot setup planning scene.");
                 return;
            }
            RCLCPP_INFO(LOGGER, "Received button1 press (true). Attempting to setup planning scene...");
            setupPlanningScene();
            workpieces_spawned_ = true;
            task_planned_executed_ = false;
        } else {
            RCLCPP_INFO(LOGGER, "Received button1 press (true), but workpieces already spawned.");
            task_planned_executed_ = false;
        }
    }
}

// Callback for button2 (Run MTC)
void MTCTaskNode::run_mtc_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
     if (msg->data) {
        if (!workpieces_spawned_) {
            RCLCPP_WARN(LOGGER, "Received button2 press (true), but workpieces are not yet spawned.");
            return;
        }
         if (task_planned_executed_) {
             RCLCPP_INFO(LOGGER, "Received button2 press (true), but MTC task already completed this cycle.");
             return;
         }

        {
             std::lock_guard<std::mutex> lock(data_mutex_);
             if (!circles_data_received_) {
                 RCLCPP_WARN(LOGGER, "Received button2 press (true), but no circles data has been received yet. Cannot set target place pose or pick objects.");
                 return;
             }
        }

        RCLCPP_INFO(LOGGER, "Received button2 press (true). Running MTC task...");
        doTask();
     }
}