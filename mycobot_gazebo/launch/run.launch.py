import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()
    launch_dir_moveit = os.path.join(
        get_package_share_directory(
            'mycobot_moveit_config'), 'launch')

    launch_dir_gazebo = os.path.join(
        get_package_share_directory(
            'mycobot_gazebo'), 'launch')

    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir_moveit, 'moveit_rviz.launch.py'))
    )
    ld.add_action(rviz_launch)


    # Controllers spawner
    load_controllers_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir_moveit, 'spawn_controllers.launch.py'))
    )
    ld.add_action(load_controllers_cmd)
    

    # move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir_gazebo, 'starter', 'move_group_gazebo.launch.py'))
    )
    ld.add_action(move_group_launch)
    
    
    # Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir_gazebo, 'starter', 'gazebo.launch.py'))
    )
    ld.add_action(gazebo_launch)


    return ld