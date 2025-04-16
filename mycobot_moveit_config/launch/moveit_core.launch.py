import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    launch_dir = os.path.join(
        get_package_share_directory(
            'mycobot_moveit_config'), 'launch')
    
    # use_sim = LaunchConfiguration('use_sim')
    # declare_use_sim = DeclareLaunchArgument(
    #     'use_sim',
    #     default_value='True',
    #     description='Start robot in Gazebo simulation.')
    # ld.add_action(declare_use_sim)


    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/moveit_rviz.launch.py'])
    )
    ld.add_action(rviz_launch)


    # move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/move_group.launch.py'])
    )
    ld.add_action(move_group_launch)


    # Controllers spawner
    load_controllers_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/spawn_controllers.launch.py'])
    )
    ld.add_action(load_controllers_cmd)
    
    return ld