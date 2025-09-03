#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient

from niryo_ned_ros2_interfaces.srv import ControlConveyor
from niryo_ned_ros2_interfaces.msg import DigitalIOState
from niryo_ned_ros2_interfaces.action import RobotMove, Tool

import yaml
import os
from ament_index_python.packages import get_package_share_directory


class ConveyorController:
    def __init__(self, node: Node, service_name: str, conveyor_id: int, speed: int) -> None:
        self._node = node
        self._client = node.create_client(ControlConveyor, service_name)
        self._conveyor_id = int(conveyor_id)
        self._speed = int(speed)
        self._current_state = None  # True running, False stopped

    def set_running(self, run: bool, done_cb=None) -> None:
        if self._current_state is not None and self._current_state == run:
            if done_cb:
                done_cb()
            return
        if not self._client.service_is_ready():
            if not self._client.wait_for_service(timeout_sec=0.0):
                self._node.get_logger().warn(f"Waiting for service {self._client.srv_name if hasattr(self._client, 'srv_name') else ''}")
                return
        req = ControlConveyor.Request()
        req.id = self._conveyor_id
        req.control_on = True
        req.speed = self._speed
        req.direction = 1 if run else 0
        self._node.get_logger().info(f"Conveyor {'RUN' if run else 'STOP'} (direction={req.direction})")
        fut = self._client.call_async(req)

        def _after(_):
            self._current_state = run
            if done_cb:
                done_cb()

        fut.add_done_callback(_after)


class PickAndPlaceExecutor:
    def __init__(self, node: Node, robot_action: str, tool_action: str, poses: dict, tool_cfg: dict) -> None:
        self._node = node
        self._robot = ActionClient(node, RobotMove, robot_action)
        self._tool = ActionClient(node, Tool, tool_action)
        self._poses = poses
        self._tool_cfg = tool_cfg
        self._steps = None

    def start(self, done_cb=None) -> None:
        steps = [
            (self._move, self._poses["grip"]),
            (self._tool_cmd, 2, True),  # close
            (self._move, self._poses["high1"]),
            (self._move, self._poses["high2"]),
            (self._move, self._poses["low1"]),
            (self._tool_cmd, 1, True),  # open
            (self._move, self._poses["high2"]),
            (self._move, self._poses["high1"]),
            (self._move, self._poses["grip"]),
        ]
        self._steps = iter(steps)
        self._next(done_cb)

    def _next(self, done_cb):
        try:
            fn, *args = next(self._steps)
        except StopIteration:
            self._steps = None
            if done_cb:
                done_cb()
            return
        fn(*args, done_cb=lambda: self._next(done_cb))

    def _move(self, joints, done_cb=None):
        goal = RobotMove.Goal()
        goal.cmd.cmd_type = 0
        goal.cmd.joints = list(joints)
        send_fut = self._robot.send_goal_async(goal)

        def _sent(f):
            gh = f.result()
            if not gh.accepted:
                self._node.get_logger().error("RobotMove rejected")
                if done_cb:
                    done_cb()
                return
            res_fut = gh.get_result_async()
            res_fut.add_done_callback(lambda _:(self._node.get_logger().info("RobotMove completed"), done_cb and done_cb()))

        send_fut.add_done_callback(_sent)

    def _tool_cmd(self, cmd_type: int, activate: bool, done_cb=None):
        goal = Tool.Goal()
        goal.cmd.cmd_type = int(cmd_type)
        goal.cmd.tool_id = int(self._tool_cfg["id"])
        goal.cmd.activate = bool(activate)
        goal.cmd.max_torque_percentage = int(self._tool_cfg["max"])
        goal.cmd.hold_torque_percentage = int(self._tool_cfg["hold"])
        send_fut = self._tool.send_goal_async(goal)

        def _sent(f):
            gh = f.result()
            if not gh.accepted:
                self._node.get_logger().error("Tool command rejected")
                if done_cb:
                    done_cb()
                return
            res_fut = gh.get_result_async()
            res_fut.add_done_callback(lambda _:(self._node.get_logger().info(f"Tool cmd completed (cmd_type={cmd_type})"), done_cb and done_cb()))

        send_fut.add_done_callback(_sent)


class QualityCheckNode(Node):
    def __init__(self) -> None:
        super().__init__("quality_check_node")

        # Parameters
        self.conveyor_id = self.declare_parameter("conveyor_id", 9).get_parameter_value().integer_value
        self.speed = self.declare_parameter("speed", 60).get_parameter_value().integer_value
        self.sensor_index = self.declare_parameter("sensor_index", 4).get_parameter_value().integer_value
        self.digital_state_topic = self.declare_parameter(
            "digital_state_topic",
            "/quality_check/niryo_robot_rpi/digital_io_state",
        ).get_parameter_value().string_value
        self.conveyor_service = self.declare_parameter(
            "conveyor_service",
            "/quality_check/niryo_robot/conveyor/control_conveyor",
        ).get_parameter_value().string_value
        self.robot_action = self.declare_parameter(
            "robot_action",
            "/quality_check/niryo_robot_arm_commander/robot_action",
        ).get_parameter_value().string_value
        self.tool_action = self.declare_parameter(
            "tool_action",
            "/quality_check/niryo_robot_tools_commander/action_server",
        ).get_parameter_value().string_value
        self.tool_id = self.declare_parameter("tool_id", 11).get_parameter_value().integer_value
        self.max_torque_percentage = self.declare_parameter("max_torque_percentage", 100).get_parameter_value().integer_value
        self.hold_torque_percentage = self.declare_parameter("hold_torque_percentage", 100).get_parameter_value().integer_value

        # Load poses from YAML (config/poses.yaml by default)
        default_poses_path = os.path.join(
            get_package_share_directory("ned3pro_quality_check_manager"),
            "config",
            "poses.yaml",
        )
        poses_path = self.declare_parameter("poses_path", default_poses_path).get_parameter_value().string_value
        with open(poses_path, "r") as f:
            poses_file = yaml.safe_load(f)
        poses = poses_file.get("poses", {})
        required = ["grip", "high1", "high2", "low1"]
        for key in required:
            if key not in poses:
                raise RuntimeError(f"Missing pose '{key}' in {poses_path}")

        # Compose helpers
        self.conveyor = ConveyorController(self, self.conveyor_service, self.conveyor_id, self.speed)
        tool_cfg = {"id": self.tool_id, "max": self.max_torque_percentage, "hold": self.hold_torque_percentage}
        self.pick_place = PickAndPlaceExecutor(self, self.robot_action, self.tool_action, poses, tool_cfg)

        # State
        self._busy = False
        self._last_object_detected = None

        # Subscriptions and timer
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(DigitalIOState, self.digital_state_topic, self._on_digital_state, qos)
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info("quality_check_node started: monitoring sensor and controlling conveyor")

    def _on_digital_state(self, msg: DigitalIOState) -> None:
        try:
            value = msg.digital_inputs[self.sensor_index].value
        except (IndexError, AttributeError) as exc:
            self.get_logger().error(
                f"Invalid digital input index {self.sensor_index} or message structure: {exc}"
            )
            return
        self._last_object_detected = not value

    def _control_loop(self) -> None:
        if self._last_object_detected is None:
            return
        if not self._last_object_detected:
            if not self._busy:
                self.conveyor.set_running(True)
            return
        if self._busy:
            return
        self._busy = True
        self.conveyor.set_running(False, done_cb=lambda: self.pick_place.start(done_cb=self._after_sequence))

    def _after_sequence(self):
        self._busy = False
        self.conveyor.set_running(True)


def main() -> None:
    rclpy.init()
    node = QualityCheckNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

