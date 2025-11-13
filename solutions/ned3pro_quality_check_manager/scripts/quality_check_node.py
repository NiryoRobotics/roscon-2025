#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rpi_manager.srv import GetSafety
from rclpy.action import ActionClient

from niryo_ned_ros2_interfaces.srv import ControlConveyor
from niryo_ned_ros2_interfaces.msg import DigitalIOState
from niryo_ned_ros2_interfaces.action import RobotMove, Tool

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


class PickAndPlaceExecutorSync:

    def __init__(self, node: Node, robot_action: str, tool_action: str, poses: dict, tool_cfg: dict) -> None:
        self._node = node
        self._robot = ActionClient(node, RobotMove, robot_action)
        self._tool = ActionClient(node, Tool, tool_action)
        self._poses = poses
        self._tool_cfg = tool_cfg

        if not self._robot.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error(f"robot server {robot_action} not available !")
        if not self._tool.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error(f"tool server {tool_action} not available !")

    def execute_first_phase(self) -> None:
        """Go pick the object and drop it at low1, open gripper, return to high2."""
        steps = [
            ("move", self._poses["grip"]),
            ("tool", 2, True),  # close
            ("move", self._poses["high1"]),
            ("move", self._poses["high2"]),
            ("move", self._poses["low1"]),
            ("tool", 1, True),  # open
            ("move", self._poses["high2"]),
        ]
        for step in steps:
            if step[0] == "move":
                self._move(step[1])
            elif step[0] == "tool":
                self._tool_cmd(step[1], step[2])

    def execute_second_phase(self, target_pose: list[float]) -> None:
        """From high2, go to low1, close, high2, go to target, open, and return path."""
        steps = [
            ("move", self._poses["low1"]),
            ("tool", 2, True),  # close
            ("move", self._poses["high2"]),
            ("move", target_pose),
            ("tool", 1, True),  # open
            ("move", self._poses["high2"]),
            ("move", self._poses["high1"]),
            ("move", self._poses["grip"]),
        ]
        for step in steps:
            if step[0] == "move":
                self._move(step[1])
            elif step[0] == "tool":
                self._tool_cmd(step[1], step[2])

    def _move(self, joints):
        goal = RobotMove.Goal()
        goal.cmd.cmd_type = 0
        goal.cmd.joints = list(joints)
        send_future = self._robot.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self._node.get_logger().error("RobotMove rejected")
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        self._node.get_logger().info("RobotMove completed")

    def _tool_cmd(self, cmd_type: int, activate: bool):
        goal = Tool.Goal()
        goal.cmd.cmd_type = cmd_type
        goal.cmd.tool_id = self._tool_cfg["id"]
        goal.cmd.activate = activate
        goal.cmd.max_torque_percentage = self._tool_cfg["max"]
        goal.cmd.hold_torque_percentage = self._tool_cfg["hold"]
        send_future = self._tool.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self._node.get_logger().error("Tool command rejected")
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        self._node.get_logger().info(f"Tool cmd completed (cmd_type={cmd_type})")


class QualityCheckNodeSync(Node):
    def __init__(self):
        super().__init__("quality_check_node_sync")

        # --- Parameters ---
        self.conveyor_id = self.declare_parameter("conveyor_id", 9).get_parameter_value().integer_value
        self.speed = self.declare_parameter("speed", 60).get_parameter_value().integer_value
        self.sensor_index = self.declare_parameter("sensor_index", 4).get_parameter_value().integer_value
        self.digital_state_topic = self.declare_parameter(
            "digital_state_topic", "/niryo_robot_rpi/digital_io_state"
        ).get_parameter_value().string_value
        self.conveyor_service = self.declare_parameter(
            "conveyor_service", "/niryo_robot/conveyor/control_conveyor"
        ).get_parameter_value().string_value
        self.robot_action = self.declare_parameter(
            "robot_action", "/niryo_robot_arm_commander/robot_action"
        ).get_parameter_value().string_value
        self.tool_action = self.declare_parameter(
            "tool_action", "/niryo_robot_tools_commander/action_server"
        ).get_parameter_value().string_value
        self.tool_id = self.declare_parameter("tool_id", 11).get_parameter_value().integer_value
        self.max_torque_percentage = self.declare_parameter("max_torque_percentage", 100).get_parameter_value().integer_value
        self.hold_torque_percentage = self.declare_parameter("hold_torque_percentage", 100).get_parameter_value().integer_value

        # --- Poses ---
        default_poses_path = os.path.join(
            get_package_share_directory("ned3pro_quality_check_manager"), "config", "poses.yaml"
        )
        poses_path = self.declare_parameter("poses_path", default_poses_path).get_parameter_value().string_value
        with open(poses_path, "r") as f:
            poses_file = yaml.safe_load(f)
        poses = poses_file.get("poses", {})
        for key in ["grip", "high1", "high2", "low1", "safe", "unsafe"]:
            if key not in poses:
                raise RuntimeError(f"Missing pose '{key}' in {poses_path}")

        # --- Helpers ---
        self.conveyor = ConveyorControllerSync(self, self.conveyor_service, self.conveyor_id, self.speed)
        tool_cfg = {"id": self.tool_id, "max": self.max_torque_percentage, "hold": self.hold_torque_percentage}
        self.pick_place = PickAndPlaceExecutorSync(self, self.robot_action, self.tool_action, poses, tool_cfg)

        # --- State ---
        self._last_object_detected = None
        # Service client for safety state
        self._safety_client = self.create_client(GetSafety, '/get_safety')
        if not self._safety_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Service /get_safety not available at startup; will retry in loop")

        # --- Subscription ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(DigitalIOState, self.digital_state_topic, self._on_digital_state, qos)

        self.get_logger().info("quality_check_node_sync started: monitoring sensor and controlling conveyor")

    def _on_digital_state(self, msg: DigitalIOState) -> None:
        try:
            value = msg.digital_inputs[self.sensor_index].value
        except (IndexError, AttributeError) as exc:
            self.get_logger().error(f"Invalid digital input index {self.sensor_index}: {exc}")
            return
        self._last_object_detected = not value

    def _get_safety_state(self) -> str:
        if not self._safety_client.service_is_ready():
            self._safety_client.wait_for_service(timeout_sec=0.5)
        try:
            req = GetSafety.Request()
            future = self._safety_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            resp = future.result()
            if resp is None or not hasattr(resp, 'state'):
                return 'unsafe'
            return (resp.state or '').lower()
        except Exception as exc:
            self.get_logger().warn(f"get_safety call failed: {exc}")
            return 'unsafe'

    def run_loop(self):
        """Main synchronous loop"""
        rate_hz = 10
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._last_object_detected is None:
                continue

            if not self._last_object_detected:
                # No object → conveyor ON
                self.conveyor.set_running(True)
                continue

            # Object detected → stop conveyor and execute in two phases
            self.conveyor.set_running(False)
            # Phase 1: pick and drop at low1, back to high2
            self.pick_place.execute_first_phase()
            # Phase 2: query service to decide safe/unsafe
            safety = self._get_safety_state()
            target = self.pick_place._poses["safe"] if safety == "safe" else self.pick_place._poses["unsafe"]
            self.get_logger().info(f"Second phase target decided by service: {'SAFE' if safety == 'safe' else 'UNSAFE'}")
            self.pick_place.execute_second_phase(target)
            self.conveyor.set_running(True)
            self._last_object_detected = False  # Reset to wait for the next object


def main():
    rclpy.init()
    node = QualityCheckNodeSync()
    try:
        node.run_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
