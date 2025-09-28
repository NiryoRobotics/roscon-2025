#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os


class DetectionNaiveNode(Node):
    def __init__(self) -> None:
        super().__init__('detection_naive')
        self.get_logger().info('detection_naive node initialised')
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Load YOLO v8 model
        model_path = "/workspaces/roscon-2025/src/assets/safety_check_model.pt"
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model not found: {model_path}")
            self.model = None
        else:
            try:
                self.model = YOLO(model_path)
                self.get_logger().info("YOLO v8 model loaded successfully")
            except Exception as e:
                self.get_logger().error(f"Error loading model: {e}")
                self.model = None
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Impossible to open the camera")
            self.cap = None
        else:
            self.get_logger().info("Camera initialised successfully")
        
        # Create publisher for topic
        self.publisher_ = self.create_publisher(String, '/quality_check/safety_state', 10)
        
        # Create image publisher
        self.image_publisher = self.create_publisher(Image, '/rpi_manager/frame', 1)
        
        self.timer_ = self.create_timer(5.0, self._on_timer)

    def _apply_performative_filters(self, frame):
        # Reduce to 144p resolution (naive approach)
        performative_filters = cv2.resize(frame, (256, 144), interpolation=cv2.INTER_NEAREST)
        
        hsv = cv2.cvtColor(performative_filters, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        mask = cv2.bitwise_or(mask_red, mask_blue)

        gray = cv2.cvtColor(performative_filters, cv2.COLOR_BGR2GRAY)
        gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        purple = np.full_like(performative_filters, (128, 0, 128)) 

        result = np.where(mask[..., None].astype(bool), purple, gray_bgr)
        
        return result

    def _get_safety_prediction(self, filtered_image):
        if self.model is None:
            self.get_logger().warn("Model not available, returning unsafe by default")
            return "unsafe"
        
        try:
            # Execute the YOLO prediction
            results = self.model.predict(filtered_image, verbose=False)
            
            # Extract the best prediction
            if results and results[0].probs is not None:
                top1 = results[0].probs.top1  # index of the best class
                conf = results[0].probs.top1conf.item()
                label = self.model.names[top1]
                
                self.get_logger().info(f"Prediction: {label} (confidence: {conf:.2f})")
                
                # Return 'safe' if the model says 'safe', otherwise 'unsafe'
                return "safe" if label.lower() == "safe" else "unsafe"
            else:
                self.get_logger().warn("No prediction available")
                return "unsafe"
                
        except Exception as e:
            self.get_logger().error(f"Error during prediction: {e}")
            return "unsafe"

    def _on_timer(self) -> None:
        if self.cap is None:
            self.get_logger().error("Camera not available")
            msg = String()
            msg.data = 'unsafe'
            self.publisher_.publish(msg)
            return
        
        try:
            # Capture an image
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Impossible to capture the image")
                msg = String()
                msg.data = 'unsafe'
                self.publisher_.publish(msg)
                return
            
            # Publish the raw camera frame
            try:
                image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_publisher.publish(image_msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish image: {e}")
            
            filtered_image = self._apply_performative_filters(frame)
            
            safety_state = self._get_safety_prediction(filtered_image)
            
            msg = String()
            msg.data = safety_state
            self.publisher_.publish(msg)
            
            self.get_logger().info(f"Published safety_state: {safety_state}")
            
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {e}")
            msg = String()
            msg.data = 'unsafe'
            self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
            self.get_logger().info("Camera released")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionNaiveNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


