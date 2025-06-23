#include "mycobot_mtc/mtc_task_node.hpp"

void MTCTaskNode::doTask()
{

    task_ = createTask();

    try {
        task_.init();
    } catch (mtc::InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e.what());
        return; 
    }

    RCLCPP_INFO(LOGGER, "Task initialized. Planning...");

    if (!task_.plan(5)) {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed for workpiece ");
        return; 
    }

    RCLCPP_INFO(LOGGER, "Task planning succeeded. Publishing solution...");
    task_.introspection().publishSolution(*task_.solutions().front());
    task_.execute(*task_.solutions().front());
}   