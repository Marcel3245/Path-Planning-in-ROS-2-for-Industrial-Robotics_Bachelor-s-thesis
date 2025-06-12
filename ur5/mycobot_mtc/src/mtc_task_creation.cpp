#include "mycobot_mtc/mtc_task_node.hpp"


mtc::Task MTCTaskNode::createTask(int workpiece_id, int target_place)
{
// ======================================================================================================== //
// ========================================= PICK AND PLACE =============================================== //
// ======================================================================================================== //
  mtc::Task task;
  task.stages()->setName("Pick and Place Task for the object_" + std::to_string(workpiece_id));
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm_controller";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "ee_link";
  const auto& eef = "eef_gripper";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", eef);
  task.setProperty("ik_frame", hand_frame);

  std::string object_name = "object_" + std::to_string(workpiece_id);
  mtc::Stage* current_state_ptr = nullptr; 

  geometry_msgs::msg::PoseStamped target_place_pose_;

  target_place_pose_.header.frame_id = "world";
  target_place_pose_.header.stamp = node_->now(); 
  target_place_pose_.pose.position.x = data_array_[target_place][0];
  target_place_pose_.pose.position.y = data_array_[target_place][1];
  target_place_pose_.pose.position.z = workpieces_positions_[target_place][2] + 0.001;
  target_place_pose_.pose.orientation.w = 1.0; 

  // ================================ GET STARTING POSITION (CURRENT) ================================ //
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.1);
  cartesian_planner->setMaxAccelerationScalingFactor(0.1);
  cartesian_planner->setStepSize(.01);

  // ======================================== OPEN GRIPPER ============================================== //
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  // ======================================== START HOME ============================================== //
  auto stage_set_home = std::make_unique<mtc::stages::MoveTo>("set home", interpolation_planner);
  stage_set_home->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_set_home->setGoal("home");
  task.add(std::move(stage_set_home));

  // ======================================== MOVE TO PICK ============================================== //
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>("move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // Forward attach_object_stage to place pose generator
  mtc::Stage* attach_object_stage = nullptr;

  // ========================================= PICK OBJECT =============================================== //
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });
  // ======================================== APPROACH OBJECT ============================================== //
    {
      auto stage =
        std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
  // ========================================== GRASP OBJECT ================================================ //
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(object_name);
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q_initial = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) * 
                                    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q_initial.matrix();
      grasp_frame_transform.translation().z() = 0.12;

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
  }
  // ======================================== ALLOW COLLISION ============================================== //
  // ================================= *to connect gripper with object ===================================== //
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand+object)");
      stage->allowCollisions(object_name,
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage));
    }
  // ========================================= CLOSE GRIPPER =============================================== //
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("closed");
      grasp->insert(std::move(stage));
    }
  // ========================================= ATTACHE OBJECT =============================================== //
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_name, hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }
  // =========================================== LIFT OBJECT ================================================ //
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 0.05;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object, support_surface)");
      stage->allowCollisions(object_name,
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            false);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }
  // ======================================== MOVE TO PLACE ============================================== //
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }
  // ======================================== PLACE OBJECT ============================================== //
  {
      auto place = std::make_unique<mtc::SerialContainer>("place object");
      task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
      place->properties().configureInitFrom(mtc::Stage::PARENT,
                                            { "eef", "group", "ik_frame" });

  // ======================================== LOWER OBJECT ============================================== //
      {
        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "place_pose");
        stage->setObject(object_name);

        stage->setPose(target_place_pose_);
        stage->setMonitoredStage(attach_object_stage);  

        auto wrapper =
            std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(2);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(object_name);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        place->insert(std::move(wrapper));
      }
  // ======================================== OPEN GRIPPER ============================================== //
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
        stage->setGroup(hand_group_name);
        stage->setGoal("open");
        place->insert(std::move(stage));
      }
  // ========================================= DETACH OBJECT ============================================= //
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
        stage->detachObject(object_name, hand_frame);
        // attach_object_stage = stage.get();
        place->insert(std::move(stage));
      }
    task.add(std::move(place));
  }
  // ========================================= RETREAT FROM OBJECT ======================================= //
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat from " + object_name, cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.1, 0.3);
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "retreat_from_object");

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 0.05;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }
  // =========================================== GO HOME ================================================ //
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  return task;
}