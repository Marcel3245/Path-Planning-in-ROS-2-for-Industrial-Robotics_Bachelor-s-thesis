import rclpy
from rclpy.node import Node 

from sensor_msgs.msg import Image 
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import String

from cv_bridge import CvBridge
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import cv2
import numpy as np 
import message_filters
import time
from geometry_msgs.msg import PointStamped 
import tf2_geometry_msgs 

class ImageSubscriber(Node):
  def __init__(self):
    super().__init__('image_subscriber')
    self.subscription_rgb_image = self.create_subscription(Image, 'camera/rgb/image', self.image_callback, 10)
  
    self.publisher_belt = self.create_publisher(Bool, 'camera/belt/move', 10)  
    self.publisher_workpiece = self.create_publisher(Float64MultiArray, 'camera/workpiece/position', 10)  
    self.publisher_camera_video = self.create_publisher(Image, 'camera/video', 10)
    self.publisher_terminal = self.create_publisher(String, 'terminal/info', 10)
    
    self.br = CvBridge()
    
    self.workpiece_radius = 0.021 # in meters [m]
    self.workpiece_middle_height = 0.04 # in meters [m]
    self.camera_table_distance = 1.36 # in meters [m]
    self.start_point = (490, 215)
    self.end_point = (740, 505)
    self.workpiece_detected = False
    self.previous_workpiece_center = None # To check if the workpiece is not moving anymore
    self.current_center = None

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    self.terminal_info = False
    
  def image_callback(self, rgb_msg):
    try:
        img = self.br.imgmsg_to_cv2(rgb_msg, "bgr8")
    except Exception as e:
        self.get_logger().error(f"The image can't open: {e}")
        return
      
    img = cv2.rectangle(img, self.start_point, self.end_point, (255, 0, 0), 2)
    
    cropped_img = img[self.start_point[1]:self.end_point[1], self.start_point[0]:self.end_point[0]]
    gray = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=30, minRadius=0, maxRadius=0)
    if circles is not None:
      circle = np.uint16(np.around(circles))
      self.current_center = (circle[0, 0][0] + self.start_point[0], circle[0, 0][1] + self.start_point[1])
      
      # Adjust circle x-coordinate to original image coordinates
      if circle[0, 0][0] > 0 and self.workpiece_detected == False:
        self.publisher_belt.publish(Bool(data=False))
        self.workpiece_detected = True
    
      if self.workpiece_detected:
        if self.previous_workpiece_center is not None and self.previous_workpiece_center == self.current_center:
          image_width = img.shape[1]
          image_height = img.shape[0]
          
          cx_full_pixels = circle[0, 0][0] + self.start_point[0]
          cy_full_pixels = circle[0, 0][1] + self.start_point[1]
          
          f = self.workpiece_radius / circle[0, 0][2]
          
          cx_centered_pixels = cx_full_pixels - image_width/2
          cy_centered_pixels = -(image_height/2 - cy_full_pixels) 
          
          cx_full_meters = cx_centered_pixels * f
          cy_full_meters = cy_centered_pixels * f
          depth_value = self.camera_table_distance + self.workpiece_middle_height
          
          try:
            point_in_camera = PointStamped()
            point_in_camera.header.stamp = rgb_msg.header.stamp  
            point_in_camera.header.frame_id = 'camera_model'
            point_in_camera.point.x = depth_value
            point_in_camera.point.y = - cx_full_meters
            point_in_camera.point.z = cy_full_meters
            
            # Transform the point to the base frame
            transformed_point = self.tf_buffer.transform(point_in_camera, 'world', timeout=rclpy.duration.Duration(seconds=0.1))
            transformed_point.point.x = np.round(transformed_point.point.x, 3)
            transformed_point.point.y = np.round(transformed_point.point.y, 3)
            transformed_point.point.z = np.round(transformed_point.point.z, 3) 
            msg_workpiece_coordinates = Float64MultiArray()
            msg_workpiece_coordinates.data = [transformed_point.point.x, transformed_point.point.y, transformed_point.point.z]
            self.publisher_workpiece.publish(msg_workpiece_coordinates)
            if not self.terminal_info:
              self.publisher_terminal.publish(String(data='Workpiece detected! Coordinates: x: {}, y: {}, z: {}'.format(transformed_point.point.x, transformed_point.point.y, transformed_point.point.z)))
              self.terminal_info = True
            
          except TransformException as ex_pt:
            self.get_logger().error(f'Could not transform point from camera_model to world: {ex_pt}')
          
    self.previous_workpiece_center = self.current_center
    
    # Share and display image
    self.publisher_camera_video.publish(self.br.cv2_to_imgmsg(img, encoding="bgr8"))
    # cv2.imshow("image", img)
    key = cv2.waitKey(1)
    
def main(args=None):
  
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  main()