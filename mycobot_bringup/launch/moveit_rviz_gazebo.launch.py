import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    moveit_launch_dir = os.path.join(
        get_package_share_directory(
            'mycobot_moveit_config'), 'launch')
    
    gazebo_launch_dir = os.path.join(
        get_package_share_directory(
            'mycobot_gazebo'), 'launch')


    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([moveit_launch_dir, '/moveit_rviz.launch.py'])
    )
    ld.add_action(rviz_launch)

    # move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([moveit_launch_dir, '/move_group.launch.py'])
    )
    ld.add_action(move_group_launch)
    
    # Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_dir, '/gazebo.launch.py'])
    )
    ld.add_action(gazebo_launch)
    
    return ld