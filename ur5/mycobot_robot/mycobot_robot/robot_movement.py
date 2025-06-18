import os
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient 

from std_msgs.msg import Float64MultiArray, Bool, String
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory 
from trajectory_msgs.msg import JointTrajectoryPoint


from . import trajectory
class RobotMovement(Node):
    def __init__(self):
        super().__init__('robot_movement')

        # --- ROS Subscribers ---
        self.subscriber_workpiece_position = self.create_subscription(
            Float64MultiArray, 'camera/workpiece/position', self.workpiece_callback, 10
        )
        
        # --- ROS Publishers ---
        self.publisher_terminal = self.create_publisher(String, 'terminal/info', 10)
        self.publisher_robot_movement = self.create_publisher(Bool, 'robot/active', 10)     
        self.publisher_camera_active = self.create_publisher(Bool, 'camera/active', 10)

        # --- ROS 2 Action Client ---
        self.action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
        self.path = trajectory.Trajectory()
        self.get_logger().info("Robot Movement node is ready.")
        
    def workpiece_callback(self, msg):
        if msg is None:
            self.get_logger().error("Received a null pointer from the camera node.")
            return
        
        self.publisher_terminal.publish(String(data="Robot starts to move."))
        self.publisher_camera_active.publish(Bool(data=False))    
        self.publisher_robot_movement.publish(Bool(data=True))
        
        coordinates = list(msg.data)
        self.path.create_path(coordinates)
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.path.joint_names

        # Build the trajectory
        time_from_start_sec = 0
        for i, position in enumerate(self.path.trajectory[:2]):
            point = JointTrajectoryPoint()
            if position[-1] == "PTP":
                position.pop()
                point.positions = position
            elif position[-1] == "LIN":
                position.pop()
                point.positions = position
            
            time_from_start_sec += 3
            point.time_from_start = Duration(sec=time_from_start_sec, nanosec=0)
            goal.trajectory.points.append(point)
                
        self.send_goal(goal)
    
    def send_goal(self, goal):
        self.publisher_terminal.publish(String(data="Waiting for action server..."))
        self.action_client.wait_for_server()     
        self.publisher_terminal.publish(String(data="Action server is ready!"))
        
        self.action_client.send_goal(goal)
        

def main(args=None):
    rclpy.init(args=args)
    robot_movement = RobotMovement()
    rclpy.spin(robot_movement)
    robot_movement.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()