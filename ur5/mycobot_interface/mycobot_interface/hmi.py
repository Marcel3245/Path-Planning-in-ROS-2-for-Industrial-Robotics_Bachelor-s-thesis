import os
import sys
import threading
import subprocess 
import time 
from datetime import datetime

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Float64MultiArray, Bool, String
from sensor_msgs.msg import Image 

import cv2
import numpy as np
from cv_bridge import CvBridge

from PyQt5.QtWidgets import QApplication, QMainWindow, QTextEdit, QGraphicsScene, QGraphicsPixmapItem, QFrame
from PyQt5.QtCore import pyqtSignal, QObject, Qt, QThread
from PyQt5.QtGui import QImage, QPixmap, QColor
from PyQt5 import uic

from . import resources_rc
from . import workpiece

class RosSignalBridge(QObject):
    terminal_signal = pyqtSignal(str)
    video_signal    = pyqtSignal(QPixmap)
    belt_signal     = pyqtSignal(bool)
    camera_signal   = pyqtSignal(bool)
    robot_signal    = pyqtSignal(bool)

class HMI(Node):
    def __init__(self):
        super().__init__('hmi_node')
        # --- GUI Setup ---
        self.app = QApplication.instance() or QApplication(sys.argv)

        # --- UI Path Finding ---
        self.package_name = 'mycobot_interface' 
        try:
            package_share_directory = get_package_share_directory(self.package_name)
            self.ui_file = os.path.join(package_share_directory, 'human_machine_interface.ui')
            self.get_logger().info(f'Found UI file at: {self.ui_file}')
        except Exception as e:
            self.get_logger().error(f'Could not find UI file: {e}')
            self.ui_file = None
            return
        # --- GUI Initialization ---
        self.window = uic.loadUi(self.ui_file)
        
        # --- Signal Bridge Between ROS and GUI ---
        self.signal_bridge = RosSignalBridge()
        
        # Window terminal setup        
        self.window.textEdit_terminal = QTextEdit(self.window.terminal)
        self.window.textEdit_terminal.setReadOnly(True)
        self.window.textEdit_terminal.setStyleSheet("font-family: Ubuntu Mono; font-size: 12pt;")
        self.signal_bridge.terminal_signal.connect(self.update_terminal_slot)
        
        # Graphics Video View setup
        self.pixmap_item = QGraphicsPixmapItem()
        self.scene = QGraphicsScene(self.window.graphicsView)
        self.scene.addItem(self.pixmap_item)
        self.window.graphicsView.setScene(self.scene)
        self.signal_bridge.video_signal.connect(self.update_video_slot)
                
        # Graphics View setup
        self.gray_color = QColor(Qt.darkGray)
        self.red = QColor(Qt.red)
        self.green = QColor(Qt.green)
        self.diameter = 30
        
        # Conveyor belt setup
        self.window.conveyor_belt_text = QTextEdit(self.window.textEdit_conveyor_belt)
        self.window.conveyor_belt_text.setReadOnly(True)
        self.window.conveyor_belt_text.setStyleSheet("font-family: Ubuntu Mono; font-size: 10pt;")
        self.window.conveyor_belt_text.append("Belt is ready.")
        
        self.window.conveyor_belt_LED = QFrame(self.window.conveyor_belt_led_indicator)
        self.window.conveyor_belt_LED.setFixedSize(self.diameter, self.diameter)
        self.window.conveyor_belt_LED.setFrameShape(QFrame.NoFrame)
        self.window.conveyor_belt_LED.setStyleSheet(f"background-color: {self.gray_color.name()};"
                                                    f" border-radius: {self.diameter // 2}px;")
        self.signal_bridge.belt_signal.connect(self.update_belt_slot)
        
        # Camera setup
        self.window.camera_text = QTextEdit(self.window.textEdit_camera)
        self.window.camera_text.setReadOnly(True)
        self.window.camera_text.setStyleSheet("font-family: Ubuntu Mono; font-size: 10pt;")
        self.window.camera_text.append("Camera is ready.")
        
        self.window.camera_LED = QFrame(self.window.camera_led_indicator)
        self.window.camera_LED.setFixedSize(self.diameter, self.diameter) 
        self.window.camera_LED.setFrameShape(QFrame.NoFrame)
        self.window.camera_LED.setStyleSheet(f"background-color: {self.gray_color.name()};"
                                             f" border-radius: {self.diameter // 2}px;")
        self.signal_bridge.camera_signal.connect(self.update_camera_slot)
        
        # Robot setup
        self.window.robot_text = QTextEdit(self.window.textEdit_robot)
        self.window.robot_text.setReadOnly(True)
        self.window.robot_text.setStyleSheet("font-family: Ubuntu Mono; font-size: 10pt;")
        self.window.robot_text.append("Robot is ready.")
        
        self.window.robot_LED = QFrame(self.window.robot_led_indicator)
        self.window.robot_LED.setFixedSize(self.diameter, self.diameter)
        self.window.robot_LED.setFrameShape(QFrame.NoFrame)
        self.window.robot_LED.setStyleSheet(f"background-color: {self.gray_color.name()};"
                                            f" border-radius: {self.diameter // 2}px;")
        self.signal_bridge.robot_signal.connect(self.update_robot_slot)
        
        # Start the program button setup
        if hasattr(self.window, 'pushButton_run_program'):
            self.window.pushButton_run_program.clicked.connect(self.on_run_program_clicked)
            
        # Restart the program button setup
        if hasattr(self.window, 'pushButton_restart_program'):
            self.window.pushButton_restart_program.clicked.connect(self.on_restart_program_clicked)
            self.window.pushButton_restart_program.setEnabled(False)
            
        # --- ROS Subscribers ---
        self.subscriber_terminal = self.create_subscription(String, 'terminal/info', self.terminal_callback, 10)
        self.subscriber_image = self.create_subscription(Image, 'camera/video', self.main_camera_callback, 10)
        self.subscriber_side_iamge = self.create_subscription(Image, 'camera/rgb/side_image', self.side_camera_callback, 10)
        self.subscriber_belt = self.create_subscription(Bool, 'camera/belt/move', self.belt_callback, 10)
        self.subscriber_camera = self.create_subscription(Bool, 'camera/active', self.camera_callback, 10)
        self.subscriber_robot = self.create_subscription(Bool, 'robot/active', self.robot_callback, 10)
        
        # --- ROS Publisher ---
        self.publisher_belt = self.create_publisher(Bool, 'camera/belt/move', 10)  
        self.publisher_robot = self.create_publisher(Bool, 'robot/active', 10)
        self.publisher_camera_active = self.create_publisher(Bool, 'camera/active', 10)
        
        self.workpiece = workpiece.Workpiece()
        self.bridge = CvBridge()
        
        self.is_robot_active = False

        self.get_logger().info('HMI Node has been started.')

    def update_terminal_slot(self, msg):
        timestamp = datetime.now().strftime('%H:%M:%S')
        formatted_msg = f"[{timestamp}] {msg}\n"
        self.window.textEdit_terminal.append(formatted_msg)
        
        # Automatically scroll to the bottom
        self.window.textEdit_terminal.verticalScrollBar().setValue(
            self.window.textEdit_terminal.verticalScrollBar().maximum()
        )

    def terminal_callback(self, msg):
        # Instead of updating the GUI directly, emit a signal.
        self.signal_bridge.terminal_signal.emit(msg.data)
        
    def update_video_slot(self, pixmap):
        self.pixmap_item.setPixmap(pixmap)
        self.window.graphicsView.fitInView(self.pixmap_item, Qt.KeepAspectRatio)
    
    def _process_and_emit_image(self, msg):
        try: 
            # Perform the non-GUI work (image conversion) in the ROS thread
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)
            pixmap = QPixmap.fromImage(qt_image)
            
            self.signal_bridge.video_signal.emit(pixmap)
        
        except Exception as e:
            error_message = f"Error processing video frame: {str(e)}"
            self.get_logger().error(error_message)
            self.signal_bridge.terminal_signal.emit(error_message)
            
    def main_camera_callback(self, msg):
        if not self.is_robot_active:
            self._process_and_emit_image(msg)
            
    def side_camera_callback(self, msg):
        if self.is_robot_active:
            self._process_and_emit_image(msg)

    def update_belt_slot(self, move):
        if move:
            self.window.conveyor_belt_LED.setStyleSheet(f"background-color: {self.green.name()};"
                                                        f" border-radius: {self.diameter // 2}px;")
            self.window.conveyor_belt_text.clear()
            self.window.conveyor_belt_text.append("Belt is running.\n")
        else:
            self.window.conveyor_belt_LED.setStyleSheet(f"background-color: {self.red.name()};"
                                                        f" border-radius: {self.diameter // 2}px;")
            self.window.conveyor_belt_text.clear()
            self.window.conveyor_belt_text.append("Belt stopped.\n")
            
    def belt_callback(self, msg):
        # Instead of updating the GUI directly, emit a signal.
        self.signal_bridge.belt_signal.emit(msg.data)

    def update_camera_slot(self, msg):
        if msg:
            self.window.camera_LED.setStyleSheet(f"background-color: {self.green.name()};"
                                                 f" border-radius: {self.diameter // 2}px;")
            self.window.camera_text.clear()
            self.window.camera_text.append("Computing coords.")
        else:
            self.window.camera_LED.setStyleSheet(f"background-color: {self.red.name()};"
                                                 f" border-radius: {self.diameter // 2}px;")
            self.window.camera_text.clear()
            self.window.camera_text.append("Camera is off.")
            
    def camera_callback(self, msg):
        # Instead of updating the GUI directly, emit a signal.
        self.signal_bridge.camera_signal.emit(msg.data)

    def update_robot_slot(self, msg):
        if msg:
            self.window.robot_LED.setStyleSheet(f"background-color: {self.green.name()};"
                                                f" border-radius: {self.diameter // 2}px;")
            self.window.robot_text.clear()
            self.window.robot_text.append("Robot is moving!")
        else:
            self.window.robot_LED.setStyleSheet(f"background-color: {self.red.name()};"
                                                f" border-radius: {self.diameter // 2}px;")
            self.window.robot_text.clear()
            self.window.robot_text.append("Robot is not moving.")
            
    def robot_callback(self, msg):
        self.is_robot_active = msg.data
        # Instead of updating the GUI directly, emit a signal.
        self.signal_bridge.robot_signal.emit(msg.data)

    def on_run_program_clicked(self):
        self.window.pushButton_run_program.setEnabled(False) 
        self.window.pushButton_restart_program.setEnabled(True)
        self.terminal_callback(String(data="Running program..."))        
        self.workpiece.set_random_pose()

        model_path = self.workpiece.model_path
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found at {model_path}")
            return

        pose = self.workpiece.pose
        
        # Command to spawn the workpiece in Gazebo
        command = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-file', model_path,
            '-name', self.workpiece.name,
            '-x', str(pose['x']),
            '-y', str(pose['y']),
            '-z', str(pose['z'])
        ]

        try:
            # Use Popen for non-blocking execution. The GUI won't freeze.
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.terminal_callback(String(data=f"Workpiece '{self.workpiece.name}' spawned successfully."))
        except Exception as e:
            self.terminal_callback(String(data=f"Error spawning workpiece: {str(e)}"))
        
        time.sleep(2)
        # Run the belt motor
        self.publisher_robot.publish(Bool(data=False))
        self.publisher_belt.publish(Bool(data=True))
        self.terminal_callback(String(data="Belt motor started."))     
        
    def on_restart_program_clicked(self):
        self.window.pushButton_run_program.setEnabled(True)
        self.window.pushButton_restart_program.setEnabled(False)
        
        self.publisher_belt.publish(Bool(data=False))
        self.publisher_robot.publish(Bool(data=False))
        self.publisher_camera_active.publish(Bool(data=False))
        
        # Remove the workpiece

        self.terminal_callback(String(data="Program restarted.")) 

    def run_gui(self):
        """This function runs the GUI event loop."""
        if self.ui_file:
            self.window.show()
            sys.exit(self.app.exec_())

def main(args=None):
    rclpy.init(args=args)
    hmi_node = HMI()
    ros_thread = threading.Thread(target=rclpy.spin, args=(hmi_node,), daemon=True)
    ros_thread.start()
    hmi_node.run_gui()
    hmi_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()