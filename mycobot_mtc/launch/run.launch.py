import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()
    launch_dir = os.path.join(
        get_package_share_directory(
            'mycobot_moveit_config'), 'launch')
    
    ld = LaunchDescription()
    launch_mtc_dir = os.path.join(
        get_package_share_directory(
            'mycobot_mtc'), 'launch', 'starter')


    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_mtc_dir, '/moveit_rviz.launch.py'])
    )
    ld.add_action(rviz_launch)


    # move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/move_group.launch.py'])
    )
    ld.add_action(move_group_launch)
    
    
    # Controllers spawner and Moveit Task Constructor
    pick_and_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_mtc_dir, '/pick_and_place.launch.py'])
    )
    ld.add_action(pick_and_place_launch)
    
    return ld