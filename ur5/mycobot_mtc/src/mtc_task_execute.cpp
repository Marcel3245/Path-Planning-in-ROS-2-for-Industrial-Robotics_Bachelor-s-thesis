#include "mycobot_mtc/mtc_task_node.hpp"

void MTCTaskNode::doTask()
{
    // Declare variables outside the mutex scope so they are accessible later
    geometry_msgs::msg::PoseStamped target_place_pose_;
    std::list<int> furthest_target_ids;
    std::list<int> closest_workpiece_ids;

    { // Scope for the mutex lock
        std::lock_guard<std::mutex> lock(data_mutex_); 
        if (!circles_data_received_ || !storage_pose_data_received_ || !start_storage_pose_) {
            RCLCPP_ERROR(LOGGER, "Cannot run MTC task, required data is missing.");
            return;
        }

        // --- Find the furthest targets in descending order ---
        std::vector<std::vector<float>> targets_computed_data_array; 
        targets_computed_data_array.resize(2);

        for (int i = 0; i < 6; i++)
        {
            if(data_array_[i][3] == 0) {
                float distance = sqrt(pow(data_array_[i][0], 2) + pow(data_array_[i][1], 2));
                if (distance != 0) {
                    targets_computed_data_array[0].push_back(static_cast<int>(i));
                    targets_computed_data_array[1].push_back(distance); 
                }
                else 
                {
                    continue;
                }
            } 
            else 
            {
                continue;
            }
        }

        for (int step = 0; step < targets_computed_data_array[1].size() - 1; step++) 
        {
            for (int i = 0; i < targets_computed_data_array[1].size() - step - 1; i++) 
            {
                if (targets_computed_data_array[1][i] < targets_computed_data_array[1][i + 1]) 
                {
                    std::swap(targets_computed_data_array[0][i], targets_computed_data_array[0][i + 1]);
                    std::swap(targets_computed_data_array[1][i], targets_computed_data_array[1][i + 1]);
                }
            }
        }
        
        for (int i = 0; i < targets_computed_data_array[0].size(); i++) 
        {
            furthest_target_ids.push_back(static_cast<int>(targets_computed_data_array[0][i]));
        }        


        // --- Find the closest workpieces in ascending order ---
        std::vector<std::vector<float>> workpieces_computed_data_array;

        for (int i = 0; i < 6; i++) 
        {
            float current_distance = sqrt(pow(workpieces_positions_[i][0], 2) + pow(workpieces_positions_[i][1], 2));
            if (current_distance != 0) {
                workpieces_computed_data_array[0].push_back(static_cast<int>(i));
                workpieces_computed_data_array[1].push_back(current_distance);
            } 
            else 
            {
                continue;
            }
        }

        for (int step = 0; step < workpieces_computed_data_array[1].size() - 1; step++) 
        {
            for (int i = 0; i < workpieces_computed_data_array[1].size() - step - 1; i++) 
            {
                if (workpieces_computed_data_array[1][i] > workpieces_computed_data_array[1][i + 1]) 
                {
                    std::swap(workpieces_computed_data_array[0][i], workpieces_computed_data_array[0][i + 1]);
                    std::swap(workpieces_computed_data_array[1][i], workpieces_computed_data_array[1][i + 1]);
                }
            }
        }
        for (int i = 0; i < workpieces_computed_data_array[0].size(); i++) 
        {
            closest_workpiece_ids.push_back(static_cast<int>(workpieces_computed_data_array[0][i]));
        }
    } // Mutex is released here

    task_ = createTask(closest_workpiece_ids, furthest_target_ids);

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