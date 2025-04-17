from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
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
    
    
    # Rviz Node Launch
    rviz_config_path = PathJoinSubstitution([FindPackageShare("mycobot_mtc"), "config", "rviz", "mtc_robot.rviz"])
    
    
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
            "use_fake_hardware:=", "true"
            " ",
            "fake_sensor_commands:=", "false",
            " ",
            "sim_ignition:=", "True"
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
    

    # MoveIt!2 Configuration
    joint_limits_param = {"robot_description_planning": load_yaml(joint_limits_path)}  
    kinematics_params = {"robot_description_kinematics": load_yaml(kinematics_path)} 
    
    
    # Warehouse Database Config
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": "/home/marcel3245/ros2_ws/warehouse_db.sqlite",
        "port": 33829,
        "scene_name": "",
        "queries_regex": ".*",
    }
    
    
    ld = LaunchDescription()
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_params,
            joint_limits_param,
            warehouse_ros_config,
        ]
    )
    ld.add_action(rviz_node)
    
    
    # Publish Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',  'world', 'base_link'] 
    )
    ld.add_action(static_tf)

    return ld