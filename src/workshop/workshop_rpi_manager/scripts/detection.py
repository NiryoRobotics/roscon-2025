#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('detection')
        self.get_logger().info('detection node initialised')
        self.publisher_ = self.create_publisher(String, '/quality_check/safety_state', 10)
        self.timer_ = self.create_timer(5.0, self._on_timer)

    def _on_timer(self) -> None:
        pass


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