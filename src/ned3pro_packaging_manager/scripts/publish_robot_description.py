#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
from ament_index_python.packages import get_package_share_directory


class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__('robot_description_publisher')
        
        # Load URDF file
        urdf_file = os.path.join(
            get_package_share_directory("ned3pro_packaging_manager"),
            "config",
            "niryo_ned3pro.urdf.xacro"
        )
        
        with open(urdf_file, 'r') as file:
            urdf_content = file.read()
        
        # Create publisher
        self.publisher = self.create_publisher(String, '/robot_description', 10)
        
        # Publish robot description
        msg = String()
        msg.data = urdf_content
        self.publisher.publish(msg)
        
        self.get_logger().info('Published robot_description to /robot_description topic')


def main():
    rclpy.init()
    node = RobotDescriptionPublisher()
    
    # Keep publishing periodically
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
