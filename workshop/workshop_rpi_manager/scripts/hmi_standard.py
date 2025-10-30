#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QLabel, QFrame, QTextEdit, QScrollArea)
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt
from PyQt5.QtGui import QPixmap, QImage, QFont, QColor, QPalette


class ROS2Node(Node):
    """Single ROS2 node for all subscriptions"""
    
    def __init__(self):
        super().__init__('hmi_node')
        self.bridge = CvBridge()
        
        # Store latest data
        self.latest_image = None
        self.latest_safety_state = "unknown"
        
        # Create subscriptions
        self.image_subscription = self.create_subscription(
            Image,
            '/rpi_manager/frame',
            self.image_callback,
            10
        )
        
        self.safety_subscription = self.create_subscription(
            String,
            '/quality_check/safety_state',
            self.safety_callback,
            10
        )
        
        self.get_logger().info('HMI ROS2 node initialized')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def safety_callback(self, msg):
        self.latest_safety_state = msg.data


class ROS2Thread(QThread):
    """Single thread for ROS2 operations"""
    image_received = pyqtSignal(np.ndarray)
    safety_received = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.running = True
        
    def run(self):
        self.node = ROS2Node()
        
        try:
            while self.running:
                rclpy.spin_once(self.node, timeout_sec=0.01)
                self.check_data()
        except Exception as e:
            print(f"ROS2 thread error: {e}")
        finally:
            if self.node:
                self.node.destroy_node()
    
    def check_data(self):
        """Check for new data and emit signals"""
        if self.node and self.node.latest_image is not None:
            self.image_received.emit(self.node.latest_image)
            self.node.latest_image = None
            
        if self.node and self.node.latest_safety_state != "unknown":
            self.safety_received.emit(self.node.latest_safety_state)
            self.node.latest_safety_state = "unknown"
    
    def stop(self):
        self.running = False
        if self.node:
            self.node.destroy_node()
        self.quit()
        self.wait()


class IndustrialHMI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HMI Standard - Surveillance System")
        self.setGeometry(100, 100, 1200, 800)
        
        # Industrial dark style
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2b2b2b;
                color: #ffffff;
            }
            QFrame {
                background-color: #3c3c3c;
                border: 2px solid #555555;
                border-radius: 5px;
            }
            QLabel {
                color: #ffffff;
                font-weight: bold;
            }
            QTextEdit {
                background-color: #1e1e1e;
                color: #ffffff;
                border: 1px solid #555555;
                border-radius: 3px;
                font-family: 'Courier New', monospace;
            }
        """)
        
        # Variables for data
        self.current_safety_state = "unknown"
        self.robot_state = "online"
        
        # Initialize interface
        self.init_ui()
        
        # Initialize ROS2 subscriber
        self.init_ros_subscriber()
        
        # Timer for robot state update
        self.robot_timer = QTimer()
        self.robot_timer.timeout.connect(self.update_robot_status)
        self.robot_timer.start(1000)  # Update every second

    def init_ui(self):
        """Initialize user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main horizontal layout
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # Left panel - Logs and information
        left_panel = self.create_left_panel()
        main_layout.addWidget(left_panel, 1)
        
        # Right panel - Camera image
        right_panel = self.create_right_panel()
        main_layout.addWidget(right_panel, 2)
        
    def create_left_panel(self):
        """Create left panel with logs"""
        panel = QFrame()
        layout = QVBoxLayout(panel)
        layout.setSpacing(10)
        
        # Panel title
        title = QLabel("SURVEILLANCE SYSTEM")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #00ff00; margin: 10px;")
        layout.addWidget(title)
        
        # Safety state
        safety_frame = QFrame()
        safety_layout = QVBoxLayout(safety_frame)
        
        safety_label = QLabel("LAST DETECTED SAFETY STATE:")
        safety_label.setFont(QFont("Arial", 12, QFont.Bold))
        safety_layout.addWidget(safety_label)
        
        self.safety_status = QLabel("UNKNOWN")
        self.safety_status.setFont(QFont("Arial", 14, QFont.Bold))
        self.safety_status.setAlignment(Qt.AlignCenter)
        self.safety_status.setStyleSheet("""
            QLabel {
                padding: 10px;
                border-radius: 5px;
                background-color: #666666;
            }
        """)
        safety_layout.addWidget(self.safety_status)
        
        layout.addWidget(safety_frame)
        
        # Robot state
        robot_frame = QFrame()
        robot_layout = QVBoxLayout(robot_frame)
        
        robot_label = QLabel("ROBOT STATE:")
        robot_label.setFont(QFont("Arial", 12, QFont.Bold))
        robot_layout.addWidget(robot_label)
        
        self.robot_status = QLabel("ONLINE")
        self.robot_status.setFont(QFont("Arial", 14, QFont.Bold))
        self.robot_status.setAlignment(Qt.AlignCenter)
        self.robot_status.setStyleSheet("""
            QLabel {
                padding: 10px;
                border-radius: 5px;
                background-color: #00aa00;
                color: #ffffff;
            }
        """)
        robot_layout.addWidget(self.robot_status)
        
        layout.addWidget(robot_frame)
        
        # System logs area
        logs_label = QLabel("SYSTEM LOGS:")
        logs_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(logs_label)
        
        self.logs_text = QTextEdit()
        self.logs_text.setMaximumHeight(300)
        self.logs_text.setReadOnly(True)
        self.logs_text.append("System initialized...")
        self.logs_text.append("Waiting for camera data...")
        layout.addWidget(self.logs_text)
        
        # Flexible space
        layout.addStretch()
        
        return panel
        
    def create_right_panel(self):
        """Create the right panel with the image"""
        panel = QFrame()
        layout = QVBoxLayout(panel)
        
        # Image panel title
        image_title = QLabel("CAMERA VIEW")
        image_title.setFont(QFont("Arial", 14, QFont.Bold))
        image_title.setAlignment(Qt.AlignCenter)
        image_title.setStyleSheet("color: #00ff00; margin: 10px;")
        layout.addWidget(image_title)
        
        # Label for the image
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("""
            QLabel {
                border: 2px solid #555555;
                border-radius: 5px;
                background-color: #1e1e1e;
                min-height: 400px;
            }
        """)
        self.image_label.setText("Waiting for image...")
        layout.addWidget(self.image_label)
        
        # Image information
        self.image_info = QLabel("Resolution: -- | FPS: --")
        self.image_info.setAlignment(Qt.AlignCenter)
        self.image_info.setStyleSheet("color: #cccccc; font-size: 10px;")
        layout.addWidget(self.image_info)
        
        return panel
        
    def init_ros_subscriber(self):
        """Initialize ROS2 subscriber"""
        # Single thread for ROS2
        self.ros_thread = ROS2Thread()
        self.ros_thread.image_received.connect(self.update_image)
        self.ros_thread.safety_received.connect(self.update_safety_status)
        self.ros_thread.start()
        
    def update_image(self, cv_image):
        """Update the displayed image"""
        try:
            # Resize image for display
            height, width, channel = cv_image.shape
            max_width = 600
            max_height = 400
            
            if width > max_width or height > max_height:
                scale = min(max_width/width, max_height/height)
                new_width = int(width * scale)
                new_height = int(height * scale)
                cv_image = cv2.resize(cv_image, (new_width, new_height))
            
            # Convert to QImage
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            # Create pixmap and display
            pixmap = QPixmap.fromImage(qt_image)
            self.image_label.setPixmap(pixmap)
            
            # Update information
            self.image_info.setText(f"Resolution: {width}x{height} | Format: BGR8")
            
        except Exception as e:
            self.logs_text.append(f"Error updating image: {e}")
            
    def update_safety_status(self, safety_state):
        """Update safety state"""
        self.current_safety_state = safety_state.lower()
        
        if self.current_safety_state == "safe":
            self.safety_status.setText("SAFE")
            self.safety_status.setStyleSheet("""
                QLabel {
                    padding: 10px;
                    border-radius: 5px;
                    background-color: #00aa00;
                    color: #ffffff;
                }
            """)
        elif self.current_safety_state == "unsafe":
            self.safety_status.setText("UNSAFE")
            self.safety_status.setStyleSheet("""
                QLabel {
                    padding: 10px;
                    border-radius: 5px;
                    background-color: #aa0000;
                    color: #ffffff;
                }
            """)
        else:
            self.safety_status.setText("UNKNOWN")
            self.safety_status.setStyleSheet("""
                QLabel {
                    padding: 10px;
                    border-radius: 5px;
                    background-color: #666666;
                    color: #ffffff;
                }
            """)
        
        # Add to log
        self.logs_text.append(f"Safety state updated: {safety_state.upper()}")
        
    def update_robot_status(self):
        """Update robot state (simulation)"""
        # In a real system, this would come from a ROS2 topic
        self.robot_state = "online"
        self.robot_status.setText("ONLINE")
        self.robot_status.setStyleSheet("""
            QLabel {
                padding: 10px;
                border-radius: 5px;
                background-color: #00aa00;
                color: #ffffff;
            }
        """)
        
    def closeEvent(self, event):
        """Clean up resources on close"""
        if hasattr(self, 'ros_thread'):
            self.ros_thread.stop()
            
        event.accept()


def main(args=None):
    # Initialize ROS2 once at the application level
    rclpy.init(args=args)
    
    app = QApplication(sys.argv)
    
    # Global style for the application
    app.setStyle('Fusion')
    
    hmi = IndustrialHMI()
    hmi.show()
    
    try:
        sys.exit(app.exec_())
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()