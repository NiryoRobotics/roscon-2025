#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
from ultralytics import YOLO
import os


class DetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('detection')
        self.get_logger().info('detection node initialised')
        
        # Load YOLO v8 model
        model_path = "insert model path here"
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
        
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.get_logger().info(
                f"Camera buffer size set: {self.cap.get(cv2.CAP_PROP_BUFFERSIZE)}"
            )
        except Exception as e:
            self.get_logger().warn(f"Could not set camera buffer size: {e}")

        # Create publisher for topic
        self.publisher_ = self.create_publisher(String, '/quality_check/safety_state', 10)
        self.timer_ = self.create_timer(5.0, self._on_timer)

    def _apply_performative_filters(self, frame):
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
        pass

    def _on_timer(self) -> None:
        pass

    def destroy_node(self):
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


