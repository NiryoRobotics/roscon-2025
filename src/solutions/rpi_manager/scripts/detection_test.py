#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rpi_manager.srv import GetSafety


class DetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('detection')
        self.get_logger().info('detection node initialised (service: get_safety)')
        self._srv = self.create_service(GetSafety, 'get_safety', self._on_get_safety)

    def _on_get_safety(self, request: GetSafety.Request, response: GetSafety.Response) -> GetSafety.Response:
        response.state = 'safe'
        return response


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


