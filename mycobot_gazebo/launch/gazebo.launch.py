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
import xacro


def load_yaml(directory):
    with open(directory, 'r') as file:
        doc = safe_load(file)  
    return doc

def generate_launch_description():
    # General Path
    package_name_moveit = FindPackageShare(package='mycobot_moveit_config').find('mycobot_moveit_config')    
    package_name_description = FindPackageShare(package='mycobot_description').find('mycobot_description')
    package_name_gazebo = FindPackageShare(package='mycobot_gazebo').find('mycobot_gazebo')
    moveit_config_path = os.path.join(package_name_moveit, 'config')
    
    
    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(package_name_gazebo, 'worlds'), ':' +
            str(Path(package_name_description).parent.resolve())
            ]
        )
    
    
    
    arguments = LaunchDescription([
                    DeclareLaunchArgument('world', default_value='empty_world',
                            description='Gz sim World'),
            ]
        )
    
    
    
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', [LaunchConfiguration('world'),
                                '.sdf',
                                ' -v 1',
                                ' -r']
                )
            ]
        )


    xacro_file = os.path.join(package_name_description, "urdf", "ur5_robotiq85_gripper.urdf.xacro")
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true',
                                                   'name' : '"ur"'})
    robot_desc = doc.toprettyxml(indent='  ')
    
    params = {'robot_description': robot_desc}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
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
    
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=load_joint_state_controller,
               on_exit=[load_arm_controller,
                        load_gripper_controller],
            )
        ),
        bridge,
        gazebo_resource_path,
        arguments,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
    ])