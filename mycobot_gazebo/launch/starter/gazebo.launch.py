from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from yaml import safe_load
from pathlib import Path
import os


def load_yaml(directory):
    with open(directory, 'r') as file:
        doc = safe_load(file)  
    return doc

def generate_launch_description():
    # General Path
    package_name_gazebo = 'mycobot_gazebo'
    package_name_description = 'mycobot_description'
    description_config_path = os.path.join(FindPackageShare(package=package_name_description).find(package_name_description), 'config')
    
    
    physical_params = os.path.join(description_config_path, 'ur5/physical_parameters.yaml')
    visual_params = os.path.join(description_config_path, 'ur5/visual_parameters.yaml')
    
    
    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(package_name_gazebo, 'worlds'), ':' +
            str(Path(package_name_description).parent.resolve())
            ]
        )
    
    
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                # Combine arguments into a single string
                ('gz_args', [
                            os.path.join(get_package_share_directory(package_name_gazebo), 'worlds', 'empty.world.sdf'), # Assuming empty.world is the correct file
                            ' -r -v 1' # Start paused (-r), verbosity 1 (-v 1)
                            ]
                )
            ]
        )


    # UR5 Description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            os.path.join(FindPackageShare(package=package_name_description).find(package_name_description), 'urdf/ur5_robotiq85_gripper.urdf.xacro'),
            " ",
            "name:=", "ur",
            " ",
            "physical_params:=", physical_params,
            " ",
            "visual_params:=", visual_params,
            " ",
            "use_fake_hardware:=", "True", # Use the sim interface, not fake
            " ",
            "fake_sensor_commands:=", "False", # Doesn't matter if fake_hardware is false
            " ",
            "sim_gazebo:=", "True", # Explicitly false
            " ",
            "sim_ignition:=", "True"
        ]
    )
    robot_description = {"robot_description": robot_description_content}


    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'ur5',
                   '-allow_renaming', 'true'
                   '-use_sim','true'],
    )
    

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )


    return LaunchDescription([
        bridge,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
    ])