#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.logging import get_logger

from niryo_ned_ros2_interfaces.srv import ControlConveyor
from niryo_ned_ros2_interfaces.msg import DigitalIOState
from geometry_msgs.msg import PoseStamped

# MoveIt2 Python API imports (exactly as in the official example)
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit.core.kinematic_constraints import construct_joint_constraint

import yaml
import os
from ament_index_python.packages import get_package_share_directory


class ConveyorControllerSync:

    def __init__(self, node: Node, service_name: str, conveyor_id: int, speed: int) -> None:
        self._node = node
        self._client = node.create_client(ControlConveyor, service_name)
        self._conveyor_id = conveyor_id
        self._speed = speed
        self._current_state = None

        if not self._client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error(f"Service {service_name} not available !")

    def set_running(self, run: bool) -> None:
        if self._current_state is not None and self._current_state == run:
            return
        req = ControlConveyor.Request()
        req.id = self._conveyor_id
        req.control_on = True
        req.speed = self._speed
        req.direction = 1 if run else 0
        self._node.get_logger().info(f"Conveyor {'RUN' if run else 'STOP'} (direction={req.direction})")
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        self._current_state = run


class PickAndPlaceExecutorMoveIt2:
    def __init__(self, node: Node, group_name: str = "arm") -> None:
        self._node = node
        self._group_name = group_name
        self._logger = get_logger("moveit_py.packaging")
        
        # Initialize MoveIt2 Python API (parameters will be passed via launch file)
        try:
            self._moveit = MoveItPy(node_name="moveit_py_packaging")
            self._robot = self._moveit.get_robot()
            self._arm = self._moveit.get_planning_component(group_name)
            
            self._logger.info("MoveItPy instance created for packaging")
            
        except Exception as e:
            self._logger.error(f"Failed to initialize MoveIt2: {e}")
            raise

    def plan_and_execute(
        self,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
    ):
        """Helper function to plan and execute a motion - exactly as in official example"""
        # plan to goal
        self._logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = self._arm.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = self._arm.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = self._arm.plan()

        # execute the plan
        if plan_result:
            self._logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self._robot.execute(robot_trajectory, controllers=[])
            time.sleep(sleep_time)
            return True
        else:
            self._logger.error("Planning failed")
            return False

    def go_to_joints(self, joints: list[float]) -> bool:
        """Move to joint positions using RobotState - following official example pattern"""
        try:
            # instantiate a RobotState instance using the current robot model
            robot_model = self._moveit.get_robot_model()
            robot_state = RobotState(robot_model)
            
            # Set joint positions for NED3 Pro (6 joints)
            joint_names = [f"joint_{i+1}" for i in range(6)]  # joint_1 to joint_6 for NED3 Pro
            joint_values = dict(zip(joint_names, joints))
            robot_state.joint_positions = joint_values
            
            # set plan start state to current state
            self._arm.set_start_state_to_current_state()
            
            # set goal state to the initialized robot state
            self._logger.info("Set goal state to joint positions")
            self._arm.set_goal_state(robot_state=robot_state)
            
            # plan to goal
            return self.plan_and_execute(sleep_time=1.0)
                
        except Exception as e:
            self._logger.error(f"Error in go_to_joints: {e}")
            return False

    def go_to_pose(self, pose: PoseStamped, pose_link: str = "hand_link") -> bool:
        """Move to pose using PoseStamped message - following official example pattern"""
        try:
            # set plan start state to current state
            self._arm.set_start_state_to_current_state()
            
            # set pose goal with PoseStamped message
            self._arm.set_goal_state(pose_stamped_msg=pose, pose_link=pose_link)
            
            # plan to goal
            return self.plan_and_execute(sleep_time=1.0)
                
        except Exception as e:
            self._logger.error(f"Error in go_to_pose: {e}")
            return False

    def move_to_target_pose(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, 
                           qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0) -> bool:
        """Move to target pose with given position and orientation"""
        pose = PoseStamped()
        pose.header.frame_id = "base_link"  # Base frame for NED3 Pro
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        self._logger.info(f"Moving to pose: position=({x}, {y}, {z}), orientation=({qx}, {qy}, {qz}, {qw})")
        
        return self.go_to_pose(pose)

    def move_to_joint_positions(self, joints: list[float]) -> bool:
        """Move to specific joint positions [0,0,0,0,0,0] - main function for packaging"""
        self._logger.info(f"Moving to joint positions: {joints}")
        return self.go_to_joints(joints)

    def use_multi_pipeline_planning(self, target_joints: list[float] = None) -> bool:
        """Use multi-pipeline planning for better results - following official example"""
        if target_joints is None:
            target_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        try:
            # Set joint positions using constraints (as in official example)
            robot_model = self._moveit.get_robot_model()
            robot_state = RobotState(robot_model)
            
            joint_names = [f"joint_{i+1}" for i in range(6)]
            joint_values = dict(zip(joint_names, target_joints))
            robot_state.joint_positions = joint_values
            
            joint_constraint = construct_joint_constraint(
                robot_state=robot_state,
                joint_model_group=self._moveit.get_robot_model().get_joint_model_group(self._group_name),
            )
            
            # set plan start state to current state
            self._arm.set_start_state_to_current_state()
            
            # set goal state with constraints
            self._arm.set_goal_state(motion_plan_constraints=[joint_constraint])
            
            # initialise multi-pipeline plan request parameters
            multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
                self._moveit, ["ompl_rrtc", "pilz_lin", "chomp_planner"]
            )
            
            # plan to goal with multiple pipelines
            return self.plan_and_execute(
                multi_plan_parameters=multi_pipeline_plan_request_params,
                sleep_time=2.0
            )
            
        except Exception as e:
            self._logger.error(f"Error in multi-pipeline planning: {e}")
            return False


class PackagingNodeMoveIt2(Node):
    def __init__(self):
        super().__init__("packaging_node_moveit2")

        # --- Parameters ---
        self.conveyor_id = self.declare_parameter("conveyor_id", 9).get_parameter_value().integer_value
        self.speed = self.declare_parameter("speed", 60).get_parameter_value().integer_value
        self.sensor_index = self.declare_parameter("sensor_index", 4).get_parameter_value().integer_value
        self.digital_state_topic = self.declare_parameter(
            "digital_state_topic", "/packaging/niryo_robot_rpi/digital_io_state"
        ).get_parameter_value().string_value
        self.conveyor_service = self.declare_parameter(
            "conveyor_service", "/packaging/niryo_robot/conveyor/control_conveyor"
        ).get_parameter_value().string_value

        # --- Helpers ---
        self.conveyor = ConveyorControllerSync(self, self.conveyor_service, self.conveyor_id, self.speed)
        
        # Initialize MoveIt2
        try:
            self.pick_place = PickAndPlaceExecutorMoveIt2(self, "arm")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MoveIt2: {e}")
            self.pick_place = None

        # --- State ---
        self._last_object_detected = None

        # --- Subscription ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(DigitalIOState, self.digital_state_topic, self._on_digital_state, qos)

        self.get_logger().info("packaging_node_moveit2 started: monitoring sensor, controlling conveyor and robot")

    def _on_digital_state(self, msg: DigitalIOState) -> None:
        try:
            value = msg.digital_inputs[self.sensor_index].value
        except (IndexError, AttributeError) as exc:
            self.get_logger().error(f"Invalid digital input index {self.sensor_index}: {exc}")
            return
        self._last_object_detected = not value

    def run_loop(self):
        """Main synchronous loop"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._last_object_detected is None:
                continue

            if not self._last_object_detected:
                # No object → conveyor ON
                self.conveyor.set_running(True)
                continue

            # Object detected → stop conveyor and move robot
            self.conveyor.set_running(False)
            self.get_logger().info("Object detected - conveyor stopped, moving robot to target pose")
            
            if self.pick_place is not None:
                # Move to joint positions [0,0,0,0,0,0] when object detected
                # Using multi-pipeline planning for better results
                success = self.pick_place.use_multi_pipeline_planning([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                if success:
                    self.get_logger().info("Robot successfully moved to target joint positions")
                else:
                    self.get_logger().error("Failed to move robot to target joint positions")
            else:
                self.get_logger().warn("MoveIt2 not available, skipping robot movement")
            
            self._last_object_detected = False  # Reset to wait for the next object


def main():
    rclpy.init()
    node = PackagingNodeMoveIt2()
    try:
        node.run_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
