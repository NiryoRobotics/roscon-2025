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


class ImageSubscriber(QThread):
    """Thread pour recevoir les images de la caméra"""
    image_received = pyqtSignal(np.ndarray)
    
    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.node = None
        
    def run(self):
        rclpy.init()
        self.node = ImageSubscriberNode(self)
        try:
            rclpy.spin(self.node)
        finally:
            self.node.destroy_node()
            rclpy.shutdown()
    
    def stop(self):
        if self.node:
            self.node.destroy_node()


class SafetySubscriber(QThread):
    """Thread pour recevoir l'état de sécurité"""
    safety_received = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.node = None
        
    def run(self):
        rclpy.init()
        self.node = SafetySubscriberNode(self)
        try:
            rclpy.spin(self.node)
        finally:
            self.node.destroy_node()
    
    def stop(self):
        if self.node:
            self.node.destroy_node()


class ImageSubscriberNode(Node):
    def __init__(self, parent_thread):
        super().__init__('hmi_image_subscriber')
        self.parent_thread = parent_thread
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/rpi_manager/frame',
            self.image_callback,
            10
        )
        self.get_logger().info('HMI Image subscriber initialized')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.parent_thread.image_received.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')


class SafetySubscriberNode(Node):
    def __init__(self, parent_thread):
        super().__init__('hmi_safety_subscriber')
        self.parent_thread = parent_thread
        self.subscription = self.create_subscription(
            String,
            '/quality_check/safety_state',
            self.safety_callback,
            10
        )
        self.get_logger().info('HMI Safety subscriber initialized')

    def safety_callback(self, msg):
        self.parent_thread.safety_received.emit(msg.data)


class IndustrialHMI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HMI Standard - Surveillance System")
        self.setGeometry(100, 100, 1200, 800)
        
        # Style industriel sombre
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
        
        # Variables pour les données
        self.current_safety_state = "unknown"
        self.robot_state = "online"
        
        # Initialiser l'interface
        self.init_ui()
        
        # Initialiser les subscribers ROS2
        self.init_ros_subscribers()
        
        # Timer pour mettre à jour l'état du robot
        self.robot_timer = QTimer()
        self.robot_timer.timeout.connect(self.update_robot_status)
        self.robot_timer.start(1000)  # Mise à jour toutes les secondes

    def init_ui(self):
        """Initialiser l'interface utilisateur"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Layout principal horizontal
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # Panneau de gauche - Logs et informations
        left_panel = self.create_left_panel()
        main_layout.addWidget(left_panel, 1)
        
        # Panneau de droite - Image de la caméra
        right_panel = self.create_right_panel()
        main_layout.addWidget(right_panel, 2)
        
    def create_left_panel(self):
        """Créer le panneau de gauche avec les logs"""
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
        
    def init_ros_subscribers(self):
        """Initialize ROS2 subscribers"""
        # Thread for images
        self.image_thread = ImageSubscriber()
        self.image_thread.image_received.connect(self.update_image)
        self.image_thread.start()
        
        # Thread for safety state
        self.safety_thread = SafetySubscriber()
        self.safety_thread.safety_received.connect(self.update_safety_status)
        self.safety_thread.start()
        
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
        if hasattr(self, 'image_thread'):
            self.image_thread.stop()
            self.image_thread.quit()
            self.image_thread.wait()
            
        if hasattr(self, 'safety_thread'):
            self.safety_thread.stop()
            self.safety_thread.quit()
            self.safety_thread.wait()
            
        event.accept()


def main(args=None):
    app = QApplication(sys.argv)
    
    # Global style for the application
    app.setStyle('Fusion')
    
    hmi = IndustrialHMI()
    hmi.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
