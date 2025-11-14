#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from ultralytics import YOLO
import os
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from rpi_manager.srv import GetSafety


class DetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('detection')
        self.get_logger().info('detection node initialised (service: get_safety)')
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        
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
        
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Impossible to open the camera")
            self.cap = None
        else:
            self.get_logger().info("Camera initialised successfully")
        
        self._srv = self.create_service(GetSafety, 'get_safety', self._on_get_safety)

        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.get_logger().info(
                f"Camera buffer size set: {self.cap.get(cv2.CAP_PROP_BUFFERSIZE)}"
            )
        except Exception as e:
            self.get_logger().warn(f"Could not set camera buffer size: {e}")
        
        # Create image publisher
        self.image_publisher = self.create_publisher(Image, '/rpi_manager/frame', 1)
        
        # Create safety state publisher
        self.safety_publisher = self.create_publisher(String, '/quality_check/safety_state', 1)

    def _apply_color_filters(self, frame):
        """Apply color filters to the image"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color ranges
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])

        # Create masks
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Combine masks
        mask = cv2.bitwise_or(mask_red, mask_blue)

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # Keep the color where the mask is present
        result = np.where(mask[..., None].astype(bool), frame, gray_bgr)
        
        return result

    def _get_safety_prediction(self, filtered_image):
        """Use the YOLO model to predict if the image is safe or unsafe"""
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

    def _on_get_safety(self, request: GetSafety.Request, response: GetSafety.Response) -> GetSafety.Response:
        """Service callback to get the safety state"""
        if self.cap is None:
            self.get_logger().error("Camera not available")
            response.state = 'unsafe'
            return response
        
        try:
            # Capture an image
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Impossible to capture the image")
                response.state = 'unsafe'
                return response
            
            # Publish the raw camera frame
            try:
                image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_publisher.publish(image_msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish image: {e}")
            
            # Apply color filters
            filtered_image = self._apply_color_filters(frame)
            
            # Get the safety prediction
            safety_state = self._get_safety_prediction(filtered_image)
            
            # Publish safety state
            try:
                safety_msg = String()
                safety_msg.data = safety_state
                self.safety_publisher.publish(safety_msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish safety state: {e}")
            
            self.get_logger().info(f"Safety state detected: {safety_state}")
            response.state = safety_state
            
        except Exception as e:
            self.get_logger().error(f"Error in the get_safety service: {e}")
            response.state = 'unsafe'
        
        return response

    def destroy_node(self):
        """Clean up resources when the node is destroyed"""
        if self.cap is not None:
            self.cap.release()
            self.get_logger().info("Camera released")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


