#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('ned3pro_quality_check_manager')
    
    # Default paths
    default_poses_path = os.path.join(pkg_share, 'config', 'poses.yaml')
    
    # Declare launch arguments
    conveyor_id_arg = DeclareLaunchArgument(
        'conveyor_id',
        default_value='9',
        description='Conveyor ID'
    )
    
    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='60',
        description='Conveyor speed'
    )
    
    sensor_index_arg = DeclareLaunchArgument(
        'sensor_index',
        default_value='4',
        description='Detection sensor index'
    )
    
    digital_state_topic_arg = DeclareLaunchArgument(
        'digital_state_topic',
        default_value='/niryo_robot_rpi/digital_io_state',
        description='Digital I/O state topic'
    )
    
    conveyor_service_arg = DeclareLaunchArgument(
        'conveyor_service',
        default_value='/niryo_robot/conveyor/control_conveyor',
        description='Conveyor control service'
    )
    
    robot_action_arg = DeclareLaunchArgument(
        'robot_action',
        default_value='/niryo_robot_arm_commander/robot_action',
        description='Robot action'
    )
    
    tool_action_arg = DeclareLaunchArgument(
        'tool_action',
        default_value='/niryo_robot_tools_commander/action_server',
        description='Tool action'
    )
    
    tool_id_arg = DeclareLaunchArgument(
        'tool_id',
        default_value='11',
        description='Tool ID'
    )
    
    max_torque_percentage_arg = DeclareLaunchArgument(
        'max_torque_percentage',
        default_value='100',
        description='Maximum torque percentage'
    )
    
    hold_torque_percentage_arg = DeclareLaunchArgument(
        'hold_torque_percentage',
        default_value='100',
        description='Hold torque percentage'
    )
    
    poses_path_arg = DeclareLaunchArgument(
        'poses_path',
        default_value=default_poses_path,
        description='Path to poses configuration file'
    )
    
    # Quality check intern node
    quality_check_intern_node = Node(
        package='ned3pro_quality_check_manager',
        executable='quality_check_node_intern.py',
        name='quality_check_node_intern',
        output='screen',
        parameters=[{
            'conveyor_id': LaunchConfiguration('conveyor_id'),
            'speed': LaunchConfiguration('speed'),
            'sensor_index': LaunchConfiguration('sensor_index'),
            'digital_state_topic': LaunchConfiguration('digital_state_topic'),
            'conveyor_service': LaunchConfiguration('conveyor_service'),
            'robot_action': LaunchConfiguration('robot_action'),
            'tool_action': LaunchConfiguration('tool_action'),
            'tool_id': LaunchConfiguration('tool_id'),
            'max_torque_percentage': LaunchConfiguration('max_torque_percentage'),
            'hold_torque_percentage': LaunchConfiguration('hold_torque_percentage'),
            'poses_path': LaunchConfiguration('poses_path'),
        }]
    )
    
    return LaunchDescription([
        conveyor_id_arg,
        speed_arg,
        sensor_index_arg,
        digital_state_topic_arg,
        conveyor_service_arg,
        robot_action_arg,
        tool_action_arg,
        tool_id_arg,
        max_torque_percentage_arg,
        hold_torque_percentage_arg,
        poses_path_arg,
        quality_check_intern_node,
    ])
