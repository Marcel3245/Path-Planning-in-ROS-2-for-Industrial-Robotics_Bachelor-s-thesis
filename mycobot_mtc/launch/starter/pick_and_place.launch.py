from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from yaml import safe_load
import os


def load_yaml(directory):
    with open(directory, 'r') as file:
        doc = safe_load(file)  
    return doc

def generate_launch_description():
     # General Path
    package_name_moveit_config = 'mycobot_moveit_config'
    package_name_description_config = 'mycobot_description'
    moveit_config_path = os.path.join(FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config), 'config')
    description_config_path = os.path.join(FindPackageShare(package=package_name_description_config).find(package_name_description_config), 'config')


    # General Path Dierectories
    joint_limits_path = os.path.join(description_config_path, 'ur5/joint_limits.yaml')
    kinematics_path = os.path.join(moveit_config_path, 'ur5/kinematics.yaml')
    physical_params = os.path.join(description_config_path, 'ur5/physical_parameters.yaml')
    visual_params = os.path.join(description_config_path, 'ur5/visual_parameters.yaml')
    controllers_path = os.path.join(moveit_config_path, 'controllers.yaml')
    omnpl_planning_path = os.path.join(moveit_config_path, 'ompl_planning.yaml')
    pilz_planning_path = os.path.join(moveit_config_path, 'pilz_industrial_motion_planner_planning.yaml')
    pilz_cartesian_limits_path = os.path.join(moveit_config_path, 'ur5/pilz_cartesian_limits.yaml')
    ros2_controllers_path = os.path.join(moveit_config_path, "ros2_controllers.yaml")
    
    
    
    # UR5 Description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            os.path.join(FindPackageShare(package=package_name_description_config).find(package_name_description_config), 'urdf/ur5_robotiq85_gripper.urdf.xacro'),
            " ",
            "name:=", "ur",
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "use_fake_hardware:=", "True"
            " ",
            "fake_sensor_commands:=", "False",
        ]
    )
    robot_description = {"robot_description": robot_description_content}


    # Robot SRDF
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            os.path.join(FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config), 'srdf/ur_gripper.srdf.xacro'),
            " ",
            "name:=", "ur",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    
    
    # OMPL Planner
    ompl_planning_pipeline_config = {
            "planning_pipelines": {
                "default_planning_pipeline": "ompl",
                "pipeline_names": ["pilz","ompl"],
            },
            "pilz": {
                "planning_plugins": ["pilz_industrial_motion_planner/CommandPlanner"],
                "request_adapters": ["default_planning_request_adapters/ResolveConstraintFrames",
                                "default_planning_request_adapters/ValidateWorkspaceBounds",
                                "default_planning_request_adapters/CheckStartStateBounds",
                                "default_planning_request_adapters/CheckStartStateCollision"],
                "response_adapters": [
                                "default_planning_response_adapters/ValidateSolution",
                                "default_planning_response_adapters/DisplayMotionPath"],
            },
            "ompl": {
                "planning_plugins": ["ompl_interface/OMPLPlanner"],
                "request_adapters": ["default_planning_request_adapters/ResolveConstraintFrames",
                                "default_planning_request_adapters/ValidateWorkspaceBounds",
                                "default_planning_request_adapters/CheckStartStateBounds",
                                "default_planning_request_adapters/CheckStartStateCollision"],
                "response_adapters": ["default_planning_response_adapters/AddTimeOptimalParameterization",
                                "default_planning_response_adapters/ValidateSolution",
                                "default_planning_response_adapters/DisplayMotionPath"],
                "start_state_max_bounds_error": 0.1
            },
    }
    pilz_planning_yaml = load_yaml(pilz_planning_path)
    ompl_planning_pipeline_config["pilz"].update(pilz_planning_yaml)
    
    ompl_planning_yaml = load_yaml(omnpl_planning_path)
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)
    

    
    # Trajectory Execution
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    }
    
    
    # Moveit Controllers
    moveit_simple_controllers_yaml = load_yaml(controllers_path)
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    
    
    # Planning Scene Monitor Parameters
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True
    }
    

    # MoveIt!2 Configuration
    joint_limits_param = {"robot_description_planning": load_yaml(joint_limits_path)}  
    kinematics_params = {"robot_description_kinematics": load_yaml(kinematics_path)} 
    cartesian_limits = {"robot_description_planning": load_yaml(pilz_cartesian_limits_path)}


    # Move group capabilities
    move_group_capabilities = {
        "capabilities": " ".join([
            "move_group/ExecuteTaskSolutionCapability",
            "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService",
         ])
    }
    
    
    moveit_config = [
            robot_description,
            robot_description_semantic,
            kinematics_params,
            joint_limits_param,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            cartesian_limits,
            move_group_capabilities,
    ]
    
    ld = LaunchDescription()
    # Spawner Node for Joint State Broadcaster
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawner Node for Arm Controller
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawner Node for Gripper Controller
    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # MTC Demo node
    pick_place_demo = Node(
        package="mycobot_mtc",
        executable="mtc_node",
        output="screen",
        parameters=moveit_config,
    )

    # 1. Start the Joint State Broadcaster immediately
    ld.add_action(jsb_spawner)

    # 2. Start the Arm Controller after the Joint State Broadcaster spawner finishes
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_spawner], 
        )
    ))
    
    # 3. Start the Gripper Controller after the Arm Controller spawner finishes
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_spawner,
            on_exit=[gripper_spawner],
        )
    ))
    
    # 4. Start the MTC planner
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_spawner,
            on_exit=[pick_place_demo],
        )
    ))


    return ld