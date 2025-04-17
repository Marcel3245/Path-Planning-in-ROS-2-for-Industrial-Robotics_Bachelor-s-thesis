import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from yaml import safe_load

def load_yaml(directory):
    with open(directory, 'r') as file:
        doc = safe_load(file)  
    return doc


def generate_launch_description():
    # General arguments
    use_sim = LaunchConfiguration('use_sim')
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='True',
        description='Start robot in Gazebo simulation.')


    # General Path
    package_name_moveit_config = 'mycobot_moveit_config'
    package_name_description_config = 'mycobot_description'
    package_name_gazebo = 'mycobot_gazebo'

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
            "sim_gazebo:=", "True",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    
   ## ============================================================= MOVE GROUP INITIALIZATION ============================================================== ##
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
        "trajectory_execution.allowed_start_tolerance": 0.01,
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
 
    
    # MoveIt!2 Node Launch    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
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
            {"use_sim_time": use_sim},
        ],
    )
    

   ## ============================================================= CONTROLLERS INITIALIZATION ============================================================= ##
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
    
    fixed_tf_broadcast = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'desk', 'default']
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
        
        
    # Controller Manager
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="both",
    )



   ## ================================================================ GAZEBO INITIALIZATION =============================================================== ##
    world_file = os.path.join(get_package_share_directory(package_name_gazebo),'worlds','empty.world')
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(package_name_gazebo, 'worlds'), ':' +
            str(Path(package_name_description_config).parent.resolve())
            ]
        )
    
    
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={'gz_args': ['-r -s ', world_file ], 'on_exit_shutdown': 'true'}.items()
        #                              -r -s -v4
    )
    

    spawn_ur5 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ur5',
            '-string', robot_description_content,
        ],
        output='screen',
    )

    bridge_params = os.path.join(
        get_package_share_directory(package_name_gazebo),
        'config',
        'ros_gz_bridge.yaml'
    )
    
    gazebo_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )
    
   ## ================================================================ RVIZ INITIALIZATION ================================================================= ##
    rviz_config_path = PathJoinSubstitution([FindPackageShare("mycobot_moveit_config"), "rviz", "view_robot.rviz"])
    rviz2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_path],
                    output='screen',
                    parameters=[
                        robot_description,
                        robot_description_semantic,
                        kinematics_params,
                        joint_limits_param,
                    ]
                ),
            ],
        ),
    )



    return LaunchDescription([
        gazebo_resource_path,
        fixed_tf_broadcast,
        robot_state_publisher,
        declare_use_sim,
        jsb_spawner,
        arm_spawner,
        gripper_spawner,
        gazebo_launch,
        spawn_ur5,
        gazebo_ros_bridge,
        rviz2,
        move_group_node,
        ros2_control_node,
    ])