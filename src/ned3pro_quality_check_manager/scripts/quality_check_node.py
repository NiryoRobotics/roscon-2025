#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from niryo_ned_ros2_interfaces.srv import ControlConveyor
from niryo_ned_ros2_interfaces.msg import DigitalIOState
from niryo_ned_ros2_interfaces.action import RobotMove, Tool

import yaml
import os
from ament_index_python.packages import get_package_share_directory


# -------------------------------
# Conveyor controller (unchanged)
# -------------------------------
class ConveyorControllerSync:
    def __init__(self, node: Node, service_name: str, conveyor_id: int, speed: int) -> None:
        self._node = node
        self._client = node.create_client(ControlConveyor, service_name)
        self._conveyor_id = conveyor_id
        self._speed = speed
        self._current_state = None
        self._lock = threading.Lock()

        if not self._client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error(f"Service {service_name} not available !")

    def set_running(self, run: bool) -> None:
        with self._lock:
            if self._current_state is not None and self._current_state == run:
                return
            req = ControlConveyor.Request()
            req.id = self._conveyor_id
            req.control_on = True
            req.speed = self._speed
            req.direction = 1 if run else 0
            self._node.get_logger().info(f"Conveyor {'RUN' if run else 'STOP'} (direction={req.direction})")
            future = self._client.call_async(req)

            # Do NOT block the whole process; just wait within this thread for the service result.
            # Using spin_until_future_complete is OK here since MultiThreadedExecutor allows concurrency.
            rclpy.spin_until_future_complete(self._node, future)
            self._current_state = run


# -----------------------------------------
# Asynchronous Pick & Place with callbacks
# -----------------------------------------
class PickAndPlaceExecutorAsync:
    """
    Non-blocking, step-by-step executor.
    - Calls on_first_high() when the first 'high1' move completes.
    - Calls on_grip_closed() when the first tool-close of the cycle completes.
    - Calls on_done() at the end of the full sequence.
    """

    def __init__(self, node: Node, robot_action: str, tool_action: str, poses: dict, tool_cfg: dict,
                 on_first_high, on_grip_closed, on_done) -> None:
        self._node = node
        self._robot = ActionClient(node, RobotMove, robot_action)
        self._tool = ActionClient(node, Tool, tool_action)
        self._poses = poses
        self._tool_cfg = tool_cfg

        if not self._robot.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error(f"Serveur robot {robot_action} not available !")
        if not self._tool.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error(f"Serveur tool {tool_action} not available !")

        # callbacks to communicate with the Node
        self._on_first_high = on_first_high
        self._on_grip_closed = on_grip_closed
        self._on_done = on_done

        # state
        self._running_lock = threading.Lock()
        self._running = False
        self._step_idx = 0
        self._first_high_emitted = False
        self._grip_close_emitted = False

        # Define the same sequence as the sync version
        # NOTE: We identify the "first high" as the completion of the move to 'high1'
        self._steps = [
            ("move", self._poses["grip"]),       # (robot likely already here, but we keep it for parity)
            ("tool", 2, True),                   # close (pick)
            ("move", self._poses["high1"]),      # <-- when this finishes: restart conveyor
            ("move", self._poses["high2"]),
            ("move", self._poses["low1"]),
            ("tool", 1, True),                   # open (place)
            ("move", self._poses["high2"]),
            ("move", self._poses["high1"]),
            ("move", self._poses["grip"]),
        ]

    def is_running(self) -> bool:
        with self._running_lock:
            return self._running

    def start_cycle(self) -> bool:
        with self._running_lock:
            if self._running:
                return False
            self._running = True
            self._step_idx = 0
            self._first_high_emitted = False
            self._grip_close_emitted = False

        self._node.get_logger().info("Pick&Place: START cycle (async)")
        self._advance()
        return True

    # ----- Internal helpers -----

    def _advance(self):
        """Advance to next step asynchronously."""
        with self._running_lock:
            if not self._running:
                return
            if self._step_idx >= len(self._steps):
                self._running = False
                self._node.get_logger().info("Pick&Place: DONE cycle")
                # notify node
                self._on_done()
                return

            step = self._steps[self._step_idx]

        kind = step[0]
        if kind == "move":
            joints = step[1]
            self._send_move(joints)
        elif kind == "tool":
            cmd_type, activate = step[1], step[2]
            self._send_tool(cmd_type, activate)
        else:
            self._node.get_logger().error(f"Unknown step kind: {kind}")
            self._step_idx += 1
            self._advance()

    def _send_move(self, joints):
        goal = RobotMove.Goal()
        goal.cmd.cmd_type = 0
        goal.cmd.joints = list(joints)
        send_future = self._robot.send_goal_async(goal)
        send_future.add_done_callback(self._on_move_goal_accepted)

    def _on_move_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().error("RobotMove rejected")
            # Skip to next step to avoid deadlock
            self._step_idx += 1
            self._advance()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_move_done)

    def _on_move_done(self, future):
        _ = future.result()  # we could inspect result if needed
        # Determine if this step was the move to high1 (first high)
        step = self._steps[self._step_idx]
        is_first_high_move = (step[0] == "move" and step[1] is self._poses["high1"])

        if is_first_high_move and not self._first_high_emitted:
            self._first_high_emitted = True
            # Tell the node we reached first high => it may restart the conveyor
            self._on_first_high()

        self._node.get_logger().info(f"RobotMove completed (step {self._step_idx})")
        self._step_idx += 1
        self._advance()

    def _send_tool(self, cmd_type: int, activate: bool):
        goal = Tool.Goal()
        goal.cmd.cmd_type = cmd_type
        goal.cmd.tool_id = self._tool_cfg["id"]
        goal.cmd.activate = activate
        goal.cmd.max_torque_percentage = self._tool_cfg["max"]
        goal.cmd.hold_torque_percentage = self._tool_cfg["hold"]
        send_future = self._tool.send_goal_async(goal)
        send_future.add_done_callback(self._on_tool_goal_accepted)

    def _on_tool_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().error("Tool command rejected")
            self._step_idx += 1
            self._advance()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_tool_done)

    def _on_tool_done(self, future):
        _ = future.result()
        step = self._steps[self._step_idx]
        # The first tool "close" (cmd_type=2, activate=True) corresponds to grabbing the object for THIS cycle
        if (step[0] == "tool" and step[1] == 2 and step[2] is True and not self._grip_close_emitted):
            self._grip_close_emitted = True
            self._on_grip_closed()

        self._node.get_logger().info(f"Tool cmd completed (step {self._step_idx})")
        self._step_idx += 1
        self._advance()


# -----------------------------
# Asynchronous Quality Check
# -----------------------------
class QualityCheckNodeAsync(Node):
    def __init__(self):
        super().__init__("quality_check_node_async")

        # --- Parameters ---
        self.conveyor_id = self.declare_parameter("conveyor_id", 9).get_parameter_value().integer_value
        self.speed = self.declare_parameter("speed", 60).get_parameter_value().integer_value
        self.sensor_index = self.declare_parameter("sensor_index", 4).get_parameter_value().integer_value
        self.digital_state_topic = self.declare_parameter(
            "digital_state_topic", "/quality_check/niryo_robot_rpi/digital_io_state"
        ).get_parameter_value().string_value
        self.conveyor_service = self.declare_parameter(
            "conveyor_service", "/quality_check/niryo_robot/conveyor/control_conveyor"
        ).get_parameter_value().string_value
        self.robot_action = self.declare_parameter(
            "robot_action", "/quality_check/niryo_robot_arm_commander/robot_action"
        ).get_parameter_value().string_value
        self.tool_action = self.declare_parameter(
            "tool_action", "/quality_check/niryo_robot_tools_commander/action_server"
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
        for key in ["grip", "high1", "high2", "low1"]:
            if key not in poses:
                raise RuntimeError(f"Missing pose '{key}' in {poses_path}")

        # --- Helpers ---
        self.conveyor = ConveyorControllerSync(self, self.conveyor_service, self.conveyor_id, self.speed)
        tool_cfg = {"id": self.tool_id, "max": self.max_torque_percentage, "hold": self.hold_torque_percentage}

        # Async executor with callbacks into this node for coordination
        self.pick_place = PickAndPlaceExecutorAsync(
            self,
            self.robot_action,
            self.tool_action,
            poses,
            tool_cfg,
            on_first_high=self._on_first_high_reached,
            on_grip_closed=self._on_grip_closed_pick,
            on_done=self._on_pickplace_done,
        )

        # --- State ---
        self._lock = threading.RLock()
        self._conveyor_running = False
        self._pending_after_detection = False  # sensor saw an object while conveyor was running
        self._last_object_detected = None      # last known boolean (True=object present)
        self._ever_started = False             # to start conveyor once we receive first valid sensor

        # --- Subscription ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(DigitalIOState, self.digital_state_topic, self._on_digital_state, qos)

        self.get_logger().info("quality_check_node_async started: async actions + multithreaded spin")

    # -------------------------
    # Sensor Handling
    # -------------------------
    def _on_digital_state(self, msg: DigitalIOState) -> None:
        try:
            value = msg.digital_inputs[self.sensor_index].value
        except (IndexError, AttributeError) as exc:
            self.get_logger().error(f"Invalid digital input index {self.sensor_index}: {exc}")
            return

        detected = (not value)  # same semantics as sync version

        with self._lock:
            self._last_object_detected = detected

            # Initialize conveyor ON on first valid message if no object in front
            if not self._ever_started:
                self._ever_started = True
                if not detected:
                    self._set_conveyor(True)

            # If a new object is detected while conveyor is running, stop it and mark pending
            if detected:
                # If conveyor is running, stop immediately
                if self._conveyor_running:
                    self._set_conveyor(False)
                # Mark that we have a piece waiting to be picked by the next cycle's close
                self._pending_after_detection = True

                # If no cycle is running, start one now
                if not self.pick_place.is_running():
                    self.pick_place.start_cycle()

            else:
                # No object: nothing urgent; conveyor will be (re)started by first-high event
                pass

    # -------------------------
    # Events from executor
    # -------------------------
    def _on_first_high_reached(self):
        """Called once per cycle: after the first move to 'high1' completes."""
        with self._lock:
            # Only (re)start conveyor if no current detection requiring stop
            if not self._pending_after_detection:
                self._set_conveyor(True)
            # If pending flag is True, we keep conveyor stopped until the next cycle grip-close

    def _on_grip_closed_pick(self):
        """Called once per cycle: after the first tool-close completes (object is grabbed)."""
        with self._lock:
            # We just grabbed the object that was pending for THIS cycle -> clear pending flag
            self._pending_after_detection = False
            # Do NOT restart conveyor here; pattern is to wait until 'first high' of this cycle
            # (which already happened earlier in this cycle). For the *next* object, the next cycle
            # will repeat: first-high will restart, and sensor can stop again if needed.
            pass

    def _on_pickplace_done(self):
        """Called when the full sequence completes (back to 'grip')."""
        with self._lock:
            # If another object arrived meanwhile (pending flag already set by sensor),
            # immediately start a new cycle; conveyor is already stopped because of the sensor.
            if self._pending_after_detection and not self.pick_place.is_running():
                self.pick_place.start_cycle()

    # -------------------------
    # Conveyor helper
    # -------------------------
    def _set_conveyor(self, run: bool):
        if self._conveyor_running == run:
            return
        self.conveyor.set_running(run)
        self._conveyor_running = run


# --------------
# main() & spin
# --------------
def main():
    rclpy.init()
    node = QualityCheckNodeAsync()
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        executor.add_node(node)
        node.get_logger().info("Spinning with MultiThreadedExecutor (async)...")
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
