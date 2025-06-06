#include "mycobot_mtc/mtc_task_node.hpp"

void MTCTaskNode::doTask()
{
    // Declare variables outside the mutex scope so they are accessible later
    geometry_msgs::msg::PoseStamped target_place_pose_;
    int closest_workpiece_id;

    { // Scope for the mutex lock
        std::lock_guard<std::mutex> lock(data_mutex_); 
        if (!circles_data_received_ || !storage_pose_data_received_ || !start_storage_pose_) {
            RCLCPP_ERROR(LOGGER, "Cannot run MTC task, required data is missing.");
            return;
        }

        // --- Find the furthest target ---
        float max_target_distance = 0; 
        int furthest_target_index = -1;   

        for (int i = 0; i < 6; i++)
        {
            float current_distance = sqrt(pow(data_array_[i][0], 2) + pow(data_array_[i][1], 2));
            if (current_distance > max_target_distance) 
            {
                max_target_distance = current_distance;
                furthest_target_index = i;
            }
        }

        if (furthest_target_index == -1) {
            RCLCPP_ERROR(LOGGER, "Could not determine the furthest target.");
            return;
        }

        // --- Find the closest workpiece ---
        float min_workpiece_distance = std::numeric_limits<float>::max();
        int closest_workpiece_index = -1;

        for (int i = 0; i < 6; i++) 
        {
            float current_distance = sqrt(pow(workpieces_positions_[i][0], 2) + pow(workpieces_positions_[i][1], 2));
            if (current_distance < min_workpiece_distance) 
            {
                min_workpiece_distance = current_distance;
                closest_workpiece_index = i;
            }
        }
        
        if (closest_workpiece_index == -1) {
            RCLCPP_ERROR(LOGGER, "Could not determine the closest workpiece.");
            return;
        }

        // Assign the found index to the variable in the outer scope
        closest_workpiece_id = closest_workpiece_index;

        // Populate the pose message
        target_place_pose_.header.frame_id = "world"; 
        target_place_pose_.header.stamp = node_->now(); 
        target_place_pose_.pose.position.x = data_array_[furthest_target_index][0];
        target_place_pose_.pose.position.y = data_array_[furthest_target_index][1];
        target_place_pose_.pose.position.z = data_array_[furthest_target_index][2] + 0.005 + 0.001; // Adjusted height
        target_place_pose_.pose.orientation.w = 1.0; 

    } // Mutex is released here

    task_ = createTask(closest_workpiece_id, target_place_pose_);

    try
    {
        task_.init();
    }
    catch (mtc::InitStageException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e.what()); 
        return; 
    }

    RCLCPP_INFO(LOGGER, "Task initialized. Planning..."); 

    if (!task_.plan(5)) 
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
    }

    RCLCPP_INFO(LOGGER, "Task planning succeeded. Publishing solution..."); 
    task_.introspection().publishSolution(*task_.solutions().front()); 

    RCLCPP_INFO(LOGGER, "Executing task..."); 
    auto result = task_.execute(*task_.solutions().front());
    
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) { 
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with code: " << result.val); 
        return; 
    }

    RCLCPP_INFO(LOGGER, "Task executed successfully."); 
    task_planned_executed_ = true; 
}