import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64MultiArray

class Workpiece():
    def __init__(self):
        self.name = 'workpiece'
        self.model_path = os.path.join(
            get_package_share_directory('mycobot_gazebo'), 
            'models', 
            'workpiece', 
            'workpiece.sdf'
        )
        self.pose = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    
    def set_random_pose(self):
        self.pose = {
            'x': np.random.uniform(0.97, 1.35),
            'y': np.random.uniform(0.33, 0.67),
            'z': 0.78
        }