#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from niryo_ned_ros2_interfaces.srv import ControlConveyor
from niryo_ned_ros2_interfaces.msg import DigitalIOState


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

        # Create a timer to evaluate and command conveyor when state changes
        self._current_direction = None  # 1 forward, 0 stop (as per user spec)
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

        # Determine desired conveyor direction
        desired_direction = 0 if self._last_object_detected else 1

        # Only command when change is needed
        if desired_direction == self._current_direction:
            return

        if not self._client.service_is_ready():
            # Try reconnecting lazily
            if not self._client.wait_for_service(timeout_sec=0.0):
                self.get_logger().warn_throttle(5.0, f"Waiting for service {self.conveyor_service}")
                return

        req = ControlConveyor.Request()
        req.id = int(self.conveyor_id)
        req.control_on = True
        req.speed = int(self.speed)
        req.direction = int(desired_direction)

        future = self._client.call_async(req)

        def _done_cb(_):
            self._current_direction = desired_direction
            # Optionally, check response success if field exists
            self.get_logger().info(
                f"Conveyor command sent: id={req.id} on={req.control_on} speed={req.speed} direction={req.direction}"
            )

        future.add_done_callback(_done_cb)


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


