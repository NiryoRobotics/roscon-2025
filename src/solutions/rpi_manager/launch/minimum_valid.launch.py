#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for minimum valid solution with detection and hmi_standard."""
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for ROS2 nodes",
    )
    
    # Detection node (standard version)
    detection_node = Node(
        package="rpi_manager",
        executable="detection.py",
        name="detection",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            {
                "model_path": "/workspaces/roscon-2025/src/assets/safety_check_model.pt",
                "camera_index": 0,
            }
        ],
    )
    
    # HMI standard node
    hmi_standard_node = Node(
        package="rpi_manager",
        executable="hmi_standard.py",
        name="hmi_standard",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[
            {
                "image_topic": "/rpi_manager/frame",
                "safety_topic": "/quality_check/safety_state",
            }
        ],
    )

    return LaunchDescription([
        log_level_arg,
        detection_node,
        hmi_standard_node,
    ])
