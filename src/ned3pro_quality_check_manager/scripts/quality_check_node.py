#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient

from niryo_ned_ros2_interfaces.srv import ControlConveyor
from niryo_ned_ros2_interfaces.msg import DigitalIOState
from niryo_ned_ros2_interfaces.action import RobotMove, Tool


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

        # Set up subscriber to digital IO state
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._last_object_detected = None  # Unknown at startup
        self._sub = self.create_subscription(
            DigitalIOState, self.digital_state_topic, self._on_digital_state, qos
        )

        # Service client to control the conveyor
        self._client = self.create_client(ControlConveyor, self.conveyor_service)
        if not self._client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f"Service {self.conveyor_service} not available yet. Will keep trying while running."
            )

        # Action clients for robot and tool
        self._robot_move_ac = ActionClient(
            self,
            RobotMove,
            "/quality_check/niryo_robot_arm_commander/robot_action",
        )
        self._tool_ac = ActionClient(
            self,
            Tool,
            "/quality_check/niryo_robot_tools_commander/action_server",
        )

        # Tool parameters
        self.tool_id = self.declare_parameter("tool_id", 11).get_parameter_value().integer_value
        self.max_torque_percentage = self.declare_parameter("max_torque_percentage", 100).get_parameter_value().integer_value
        self.hold_torque_percentage = self.declare_parameter("hold_torque_percentage", 100).get_parameter_value().integer_value

        # Predefined poses (as joints)
        self.grip_pose = [
            -0.01677261411166551,
            -0.020751964806212577,
            -0.8435874406589392,
            -0.045926770046776255,
            -0.6519463250437423,
            -0.02445103901637724,
        ]
        self.high_pose_1 = [
            -0.015428710587629874,
            -0.005602506898901798,
            0.07335618846132168,
            0.0016266343776782932,
            -1.5968784903812976,
            -0.016781135076949116,
        ]
        self.high_pose_2 = [
            1.592560582567266,
            0.030054569719342356,
            -0.029426251188624398,
            -0.0075772503496351895,
            -1.5800047017145555,
            -0.01831511586483492,
        ]
        self.low_pose_1 = [
            1.4017786420317657,
            -0.23052308760341106,
            -0.6880786043062445,
            -0.05513065477409018,
            -0.667286132922599,
            -0.03979084689523393,
        ]

        # Create a timer to evaluate and command conveyor when state changes
        self._current_direction = None  # 1 forward, 0 stop (as per user spec)
        self._busy = False
        self._steps_iter = None
        self._timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info("quality_check_node started: monitoring sensor and controlling conveyor")

    def _on_digital_state(self, msg: DigitalIOState) -> None:
        try:
            value = msg.digital_inputs[self.sensor_index].value
        except (IndexError, AttributeError) as exc:
            self.get_logger().error(
                f"Invalid digital input index {self.sensor_index} or message structure: {exc}"
            )
            return

        # User-provided logic: object_detected = not value
        object_detected = not value
        self._last_object_detected = object_detected

    def _control_loop(self) -> None:
        # Require a sensor reading first
        if self._last_object_detected is None:
            return

        # No object detected: ensure conveyor runs (unless busy with sequence)
        if not self._last_object_detected:
            if not self._busy:
                self._set_conveyor_state_async(run=True)
            return

        # Object detected: stop conveyor and run pick-and-place once
        if self._busy:
            return
        self._busy = True
        # Stop conveyor first, then start async pick-and-place chain
        self._set_conveyor_state_async(run=False, done_cb=self._start_pick_and_place)

    def _set_conveyor_state_async(self, run: bool, done_cb=None) -> None:
        # Only command when change is needed
        if self._current_direction is not None:
            currently_running = self._current_direction == 1
            if run == currently_running:
                if done_cb:
                    done_cb()
                return
        if not self._client.service_is_ready():
            # Try reconnecting lazily
            if not self._client.wait_for_service(timeout_sec=0.0):
                self.get_logger().warn(f"Waiting for service {self.conveyor_service}")
                return
        req = ControlConveyor.Request()
        req.id = int(self.conveyor_id)
        if run:
            req.control_on = True
            req.speed = int(self.speed)
            req.direction = 1
            next_state = 1
            self.get_logger().info("Sending conveyor RUN command (direction=1)")
        else:
            req.control_on = True
            req.speed = int(self.speed)
            req.direction = 0
            next_state = 0
            self.get_logger().info("Sending conveyor STOP command (direction=0)")
        future = self._client.call_async(req)

        def _after(_):
            self._current_direction = next_state
            self.get_logger().info(
                f"Conveyor command sent: id={req.id} on={req.control_on} speed={req.speed} direction={req.direction}"
            )
            if done_cb:
                done_cb()

        future.add_done_callback(_after)

    def _start_pick_and_place(self) -> None:
        # Ensure action servers are ready (non-blocking check)
        if not self._robot_move_ac.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn("RobotMove action server not ready yet")
        if not self._tool_ac.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn("Tool action server not ready yet")

        # Build async sequence as (callable, args...)
        steps = [
            (self._send_robot_move_async, self.grip_pose),
            (self._send_tool_command_async, 2, True),  # close
            (self._send_robot_move_async, self.high_pose_1),
            (self._send_robot_move_async, self.high_pose_2),
            (self._send_robot_move_async, self.low_pose_1),
            (self._send_tool_command_async, 1, True),   # open
            (self._send_robot_move_async, self.high_pose_2),
            (self._send_robot_move_async, self.high_pose_1),
            (self._send_robot_move_async, self.grip_pose),
        ]
        self._steps_iter = iter(steps)
        self._execute_next_step()

    def _execute_next_step(self) -> None:
        if self._steps_iter is None:
            return
        try:
            step = next(self._steps_iter)
        except StopIteration:
            # Sequence finished
            self._steps_iter = None
            self._busy = False
            self._set_conveyor_state_async(run=True)
            return
        func, *args = step
        func(*args, done_cb=self._execute_next_step)

    def _send_robot_move_async(self, joints, done_cb=None):
        goal = RobotMove.Goal()
        goal.cmd.cmd_type = 0
        goal.cmd.joints = list(joints)
        send_future = self._robot_move_ac.send_goal_async(goal)

        def _sent(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().error("RobotMove goal rejected")
                if done_cb:
                    done_cb()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda _:
                (self.get_logger().info("RobotMove completed"), done_cb and done_cb())
            )

        send_future.add_done_callback(_sent)

    def _send_tool_command_async(self, cmd_type: int, activate: bool, done_cb=None):
        goal = Tool.Goal()
        goal.cmd.cmd_type = int(cmd_type)
        goal.cmd.tool_id = int(self.tool_id)
        goal.cmd.activate = bool(activate)
        goal.cmd.max_torque_percentage = int(self.max_torque_percentage)
        goal.cmd.hold_torque_percentage = int(self.hold_torque_percentage)
        send_future = self._tool_ac.send_goal_async(goal)

        def _sent(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().error("Tool goal rejected")
                if done_cb:
                    done_cb()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda _:
                (self.get_logger().info(f"Tool command completed (cmd_type={cmd_type})"), done_cb and done_cb())
            )

        send_future.add_done_callback(_sent)


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


