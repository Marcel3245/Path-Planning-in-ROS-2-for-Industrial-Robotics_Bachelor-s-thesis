import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float64, Bool

class ConveyorBelt(Node):
    def __init__(self):
        super().__init__('conveyor_belt')
        # --- ROS Subscribers ---
        self.subscription_camera = self.create_subscription(Bool, 'camera/belt/move', self.camera_callback, 1)

        # --- ROS Publishers ---
        self.publisher_belt = self.create_publisher(Float64, 'conveyor/velocity', 1)
        self.publisher_camera_active = self.create_publisher(Bool, 'camera/active', 1)
        
    def camera_callback(self, msg):
        if msg.data:
            self.get_logger().info('Belt is moving!')
            self.publisher_belt.publish(Float64(data=1.0)) 
            self.publisher_camera_active.publish(Bool(data=True))
        else:
            self.get_logger().info('Belt stops!')
            self.publisher_belt.publish(Float64(data=0.0))
            
def main(args=None):
    rclpy.init(args=args)
    conveyor_belt = ConveyorBelt()
    rclpy.spin(conveyor_belt)
    conveyor_belt.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()