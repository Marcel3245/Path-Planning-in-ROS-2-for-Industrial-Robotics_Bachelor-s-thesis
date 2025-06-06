#include "mycobot_mtc/mtc_task_node.hpp"

void MTCTaskNode::doTask()
{
  { 
        std::lock_guard<std::mutex> lock(data_mutex_); 

        if (!circles_data_received_) { 
            RCLCPP_ERROR(LOGGER, "No camera data received yet. Cannot set target place pose for MTC."); 
            return;
        }
        if (std::abs(data_array_[5][0]) < 1e-6 && std::abs(data_array_[5][1]) < 1e-6 && std::abs(data_array_[5][2]) < 1e-6) { 
            RCLCPP_ERROR(LOGGER, "Camera data for object 5 appears invalid (all zeros). Cannot set target place pose for MTC.");
            return; 
        }

        target_place_pose_.header.frame_id = "world"; 
        target_place_pose_.header.stamp = node_->now(); 
        target_place_pose_.pose.position.x = data_array_[5][0];
        target_place_pose_.pose.position.y = data_array_[5][1];
        target_place_pose_.pose.position.z = data_array_[5][2] + 0.005 + 0.001; // Adjusted height
        target_place_pose_.pose.orientation.w = 1.0; 

        RCLCPP_INFO(LOGGER, "Setting target place pose for THIS task run to (%.3f, %.3f, %.3f)",
                    target_place_pose_.pose.position.x, 
                    target_place_pose_.pose.position.y, 
                    target_place_pose_.pose.position.z);
    }

    task_ = createTask();

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
