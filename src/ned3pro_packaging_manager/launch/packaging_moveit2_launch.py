import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Get URDF file
    urdf_file = os.path.join(
        get_package_share_directory("niryo_ned_description"),
        "urdf/ned3pro",
        "niryo_ned3pro.urdf.xacro",
    )

    # Build MoveIt2 configuration
    moveit_config = (
        MoveItConfigsBuilder("niryo_ned3pro", package_name="ned3pro_packaging_manager")
        .robot_description(file_path=urdf_file)
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_semantic(file_path="config/niryo_ned3pro.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("ned3pro_packaging_manager"),
                "config",
                "moveit_py_config.yaml"
            )
        )
        .to_moveit_configs()
    )

    # Declare launch arguments
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    
    conveyor_id_arg = DeclareLaunchArgument(
        "conveyor_id",
        default_value="9",
        description="Conveyor ID",
    )
    
    speed_arg = DeclareLaunchArgument(
        "speed",
        default_value="60",
        description="Conveyor speed",
    )
    
    sensor_index_arg = DeclareLaunchArgument(
        "sensor_index",
        default_value="4",
        description="Digital sensor index",
    )
    moveit2_config = (
        MoveItConfigsBuilder("niryo_ned3pro", package_name="ned3pro_packaging_manager")
        .robot_description(
            file_path=urdf_file,
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_semantic(file_path="config/niryo_ned3pro.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit2_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
        
    )

    # MoveIt2 Python node
    packaging_moveit2_node = Node(
        name="packaging_node_moveit2",
        package="ned3pro_packaging_manager",
        executable="packaging_node_moveit2.py",
        output="both",
        parameters=[
            moveit_config.to_dict(),
            {
                "conveyor_id": LaunchConfiguration("conveyor_id"),
                "speed": LaunchConfiguration("speed"),
                "sensor_index": LaunchConfiguration("sensor_index"),
                "digital_state_topic": "/niryo_robot_rpi/digital_io_state",
                "conveyor_service": "/niryo_robot/conveyor/control_conveyor",
            }
        ],
    )




    # RViz configuration
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ned3pro_packaging_manager"), "config", rviz_base]
    )
    
    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Static transform publisher (world to base_link for NED3 Pro)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    return LaunchDescription([
        rviz_config_arg,
        conveyor_id_arg,
        speed_arg,
        sensor_index_arg,
        static_tf,
        #move_group_node,
        rviz_node,
        packaging_moveit2_node,
    ])
