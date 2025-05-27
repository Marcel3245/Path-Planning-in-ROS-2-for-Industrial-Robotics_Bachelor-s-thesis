import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformException
import cv2 
import numpy as np 
from geometry_msgs.msg import PointStamped 
import tf2_geometry_msgs 

class CircleData:
    def __init__(self, point, status=0):
        self.point = point # geometry_msgs.msg.Point
        self.status = status # 0 for green, 1 for red

class ImageSubscriber(Node):
  def __init__(self):
    super().__init__('image_subscriber')
    self.subscription = self.create_subscription(Image, 'camera/depth/image', self.listener_callback, 1)
    self.br = CvBridge()
    self.object_width = 0.22 # m
    self.z_f = 1.086 # Assuming the camera is at a fixed height - BETTER WITH DEPTH CAMERA        
    self.publisher_img = self.create_publisher(Image, 'camera/compute_image', 1)
    
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    self.publisher_circles_poses = self.create_publisher(Float64MultiArray, 'camera/circles_to_world', 1)
    self.publisher_storage_poses = self.create_publisher(Float64MultiArray, 'camera/storage_to_world', 1)
    
  def publish_frame(self, img):
    self.publisher_img.publish(self.br.cv2_to_imgmsg(img, encoding="bgr8"))
    
  def closing(self, mask):
    kernel = np.ones((7,7),np.uint8) 
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return closing

  def opening(self, mask):
    kernel = np.ones((6,6),np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return opening
  
  def listener_callback(self, data):
    img = self.br.imgmsg_to_cv2(data, "bgr8") 

    # Define Blue
    lower_blue = np.array([100, 100, 156], dtype=np.uint8)
    upper_blue = np.array([150, 255, 205], dtype=np.uint8)
    blue = [lower_blue, upper_blue, 'blue']
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(hsv, blue[0], blue[1])
    mask = self.closing(mask)
    mask = self.opening(mask)

    detected_circle_data = []
    
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours): 
      area = cv2.contourArea(contour) 
      if(area > 2000): 
        x, y, w, h = cv2.boundingRect(contour)  
        
        cropped_img = img[y:y+h, x:x+w]
        grayFrame = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
        _, black_white_img = cv2.threshold(grayFrame, 20, 255, cv2.THRESH_BINARY_INV)
        blure = cv2.medianBlur(black_white_img, 5)
        circles = cv2.HoughCircles(blure, cv2.HOUGH_GRADIENT, 1, 20, param1=20, param2=10, minRadius=10, maxRadius=30)
        
        rotated_rect = cv2.minAreaRect(contour)
        img_center_x = float(rotated_rect[0][0])
        img_center_y = float(rotated_rect[0][1])
        cv2.drawMarker(img, (int(img_center_x), int(img_center_y)), (0, 255, 0), cv2.MARKER_CROSS, 10, 2)
        width = float(rotated_rect[1][0])
        height = float(rotated_rect[1][1])
        
        if height > width:
          f = self.object_width / height
        else:
          f = self.object_width / width
          
          
        box_points = cv2.boxPoints(rotated_rect)
        box_points = np.int0(box_points)
        cv2.drawContours(img, [box_points], 0, (0, 0, 255), 2)
        for i in range(4):
          cv2.putText(img, str(i), (box_points[i][0], box_points[i][1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        point_0 = box_points[0]
        point_1 = box_points[1]
        point_2 = box_points[2]
        point_3 = box_points[3]
        angle = np.arctan2(abs(abs(point_3[1]) - abs(point_2[1])), abs(abs(point_3[0]) - abs(point_2[0]))) * 180 / np.pi
        if point_0[1] > point_2[1]: angle = angle
        elif point_2[1] == point_3[1]: angle = 0
        else: angle = angle - 90
        
        point_in_camera = PointStamped()
        point_in_camera.header.stamp = data.header.stamp
        point_in_camera.header.frame_id = 'camera_model'
        
        center_x = -(img_center_x - img.shape[1]/2) * f
        center_y = (img.shape[0]/2 - img_center_y) * f
        
        try:
          point_in_camera.point.x = self.z_f 
          point_in_camera.point.y = center_x
          point_in_camera.point.z = center_y  
          
          rotated_rect_center = self.tf_buffer.transform(
              point_in_camera,
              'world',
              timeout=rclpy.duration.Duration(seconds=0.1)
          )
          
        except Exception as e:
          self.get_logger().error(f'Error creating PointStamped: {e}')
          continue
        
        if circles is not None:
          circles = np.uint16(np.around(circles))
          for i in circles[0, :]:
            img = cv2.circle(img, (x + i[0], y + i[1]), i[2], (0, 0, 255), 1)
              
            x_f = -(x + i[0] - img.shape[1]/2) * f
            y_f = (img.shape[0]/2 - (y + i[1])) * f

            point_in_camera.point.x = self.z_f 
            point_in_camera.point.y = x_f
            point_in_camera.point.z = y_f
         
            try:
              # Transform the point from 'camera_model' to 'world'
              point_in_world = self.tf_buffer.transform(
                  point_in_camera,
                  'world',
                  timeout=rclpy.duration.Duration(seconds=0.1)
              )

              circle_status = 0 # Default to green

              detected_circle_data.append(CircleData(point=point_in_world.point, status=circle_status))


            except TransformException as ex_pt:
                self.get_logger().error(f'Could not transform point from camera_model to world: {ex_pt}')
    
    if detected_circle_data:
        detected_circle_data.sort(key=lambda cd: (cd.point.y, cd.point.x)) 

        # Prepare the Float64MultiArray message data
        msg_circles = Float64MultiArray()
        msg_circles.data = []
        for circle_data in detected_circle_data:
            msg_circles.data.extend([
                circle_data.point.x,
                circle_data.point.y,
                circle_data.point.z,
                float(circle_data.status)
            ])
        self.publisher_circles_poses.publish(msg_circles)


        msg_storage = Float64MultiArray()
        msg_storage.data.append(rotated_rect_center.point.x) 
        msg_storage.data.append(rotated_rect_center.point.y)
        msg_storage.data.append(0.9001)
        msg_storage.data.append(angle)
        self.publisher_storage_poses.publish(msg_storage)
        
    else:
        self.get_logger().warn("No circles detected to publish.")
          
    # Display image
    cv2.imshow("camera", img)
    self.publish_frame(img)
    
    cv2.waitKey(1)
  
def main(args=None):
  
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()