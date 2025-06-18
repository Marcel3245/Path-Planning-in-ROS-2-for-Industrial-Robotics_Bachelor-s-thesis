class Trajectory():
    def __init__(self):
        self.trajectory = [[0.45, 0.0, 1.50, 0.0, 0.0, 0.0, "PTP"], [-0.13, -0.35, 1.15, 0.0, 0.0, 0.0, "PTP"], [-0.13, -0.35, 0.95, 0.0, 0.0, 0.0, "LIN"]]
        self.storage_vector = []
        
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        
    def create_path(self, vector):
        # For PTP and Cartesian movement 
        # With predefined path!!!
        vector.append(0.0)
        vector.append(0.0)
        vector.append(0.0)
        vector.append("LIN")
        self.trajectory.insert(0, vector)
        self.trajectory.insert(0, [vector[0], vector[1], vector[2] + 0.2, 0.0, 0.0, 0.0, "LIN"])
        