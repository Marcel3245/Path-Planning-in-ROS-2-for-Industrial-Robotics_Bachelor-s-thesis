# spawn_controllers.launch.py (Recommended filename)

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
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

    # 1. Start the Joint State Broadcaster immediately
    ld.add_action(jsb_spawner)

    # 2. Start the Arm Controller after the Joint State Broadcaster spawner finishes
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_spawner], 
        )
    ))

    # 3. Start the Gripper Controller after the Arm Controller spawner finishe
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_spawner,
            on_exit=[gripper_spawner],
        )
    ))

    return ld