import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from yaml import safe_load

def load_yaml(directory):
    with open(directory, 'r') as file:
        doc = safe_load(file)  
    return doc

def declare_arguments():
    return [
        DeclareLaunchArgument( 'use_sim', default_value='true', description='Start robot in Gazebo simulation.'),
        DeclareLaunchArgument( 'robot_ip', default_value='0.0.0.0', description='IP address of the robot.'),
        DeclareLaunchArgument( 'use_fake_hardware', default_value='false', description='Use fake hardware interface.'),
        DeclareLaunchArgument( 'sim_gazebo', default_value='true', description='Use Gazebo simulation.'),
        DeclareLaunchArgument( 'sim_ignition', default_value='false', description='Use Ignition simulation.'),
        DeclareLaunchArgument( 'use_sim', default_value='true', description='Start robot in Gazebo simulation.'),
        DeclareLaunchArgument( 'safety_limits', default_value='true', description='Enables the safety limits controller if true.'),
        DeclareLaunchArgument( 'safety_pos_margin', default_value='0.15', description='The margin to lower and upper limits in the safety controller.'),
        DeclareLaunchArgument( 'safety_k_position', default_value='20', description='k-position factor in the safety controller.',),
        DeclareLaunchArgument( 'depth_camera', default_value='false', description='Enable depth camera simulation.'),
    ]

def generate_launch_description():
    
    # General Path
    package_name_moveit = 'mycobot_moveit'
    package_name_description = 'mycobot_description'
    package_name_gazebo = 'mycobot_gazebo'
    package_name_gripper_description = 'mycobot_gripper_description'
    package_name_mtc = 'mycobot_mtc'
    
    moveit_path = FindPackageShare(package=package_name_moveit).find(package_name_moveit)
    gazebo_path = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    description_path = FindPackageShare(package=package_name_description).find(package_name_description)
    gripper_description_path = FindPackageShare(package=package_name_gripper_description).find(package_name_gripper_description)
    mtc_path = FindPackageShare(package=package_name_mtc).find(package_name_mtc)
    
    moveit_config_path = os.path.join(moveit_path, 'config')
    gazebo_config_path = os.path.join(gazebo_path, 'config')
    description_config_path = os.path.join(description_path, 'config')

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
            os.path.join(description_path, 'urdf/ur5_robotiq85_gripper.urdf.xacro'),
            " ",
            "name:=", "ur",
            " ",
            "robot_ip:=", LaunchConfiguration("robot_ip"),
            " ",
            "use_fake_hardware:=", LaunchConfiguration("use_fake_hardware"),
            " ",
            "sim_gazebo:=", LaunchConfiguration("sim_gazebo"),
            " ",
            "sim_ignition:=", LaunchConfiguration("sim_ignition"),
            " ",
            "safety_limits:=", LaunchConfiguration("safety_limits"),
            " ",
            "safety_pos_margin:=", LaunchConfiguration("safety_pos_margin"),
            " ",
            "safety_k_position:=", LaunchConfiguration("safety_k_position"),
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    
   ## ============================================================= MOVE GROUP INITIALIZATION ============================================================== ##
    # Robot SRDF
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            os.path.join(moveit_path, 'srdf/ur_gripper.srdf.xacro'),
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
        "trajectory_execution.allowed_execution_duration_scaling": 1.5,
        "trajectory_execution.allowed_goal_duration_margin": 2.0,
        "trajectory_execution.trajectory_duration_monitoring": False,
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
        {"use_sim_time": LaunchConfiguration("use_sim")},
    ]
    
    # MoveIt!2 Node Launch    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=moveit_config,
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
        arguments=['gripper_action_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    fixed_tf_broadcast = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0.92', '0', '0', '0', 'world', 'base_link']
    )

    camera_model_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_camera_model', 
        output='screen',
        arguments=['-0.35', '0.47', '2.01', '-1.57', '1.57', '0.0', 'world', 'camera_model']
        # arguments=['4', '0', '1.0', '3.14', '0', '0', 'world', 'camera_model']
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
        condition=IfCondition(LaunchConfiguration("use_fake_hardware")),
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="both",
    )


   ## ================================================================ GAZEBO INITIALIZATION =============================================================== ##
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            # Path to the worlds directory within the Gazebo package
            os.path.join(gazebo_path, 'worlds'),
            ':',
            os.path.join(gazebo_path, 'models'),
            ':',
            # Path to the parent directory of the main robot description package
            str(Path(description_path).parent.resolve()),
            ':',
            # Path to the parent directory of the gripper description package 
            str(Path(gripper_description_path).parent.resolve())
        ]
    )
    
    world_path = os.path.join(gazebo_path, 'worlds', 'gazebo.world.sdf')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={
            # Pass as a single string with space separation
            'gz_args': f'-r {world_path}',
            'on_exit_shutdown': 'true'
            }.items()
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

    bridge_params = os.path.join(gazebo_config_path, 'ros_gz_bridge.yaml')
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
    rviz_config_path = os.path.join(mtc_path, 'rviz', 'mtc_rviz.rviz')
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

    ## ================================================================ MTC INITIALIZATION ================================================================= ##
    OpenCV = Node(
        package='mycobot_opencv',
        executable='img_pubsub',
        name='opencv',
        output='screen',
    )
    pick_and_place = Node(
        package="mycobot_mtc",
        executable="mtc_node",
        name="mtc_node",
        output="screen",
        parameters=moveit_config,
    )
    
    
    ld = LaunchDescription(declare_arguments())

    ld.add_action(gazebo_resource_path)
    ld.add_action(fixed_tf_broadcast)
    ld.add_action(robot_state_publisher)
    ld.add_action(camera_model_tf)
    ld.add_action(pick_and_place)
    ld.add_action(OpenCV)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_ur5)
    ld.add_action(gazebo_ros_bridge)
    ld.add_action(rviz2)
    ld.add_action(move_group_node)
    ld.add_action(ros2_control_node)
    ld.add_action(jsb_spawner)
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_spawner],
        )
    ))
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_spawner,
            on_exit=[gripper_spawner],
        )
    ))
    return ld