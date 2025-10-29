#!/usr/bin/env python3

from operator import truediv
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.logging import get_logger
from rclpy.action import ActionClient

from niryo_ned_ros2_interfaces.srv import ControlConveyor
from niryo_ned_ros2_interfaces.msg import DigitalIOState
from niryo_ned_ros2_interfaces.action import Tool
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

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
        
        # Initialize gripper action client (align with quality check node)
        self._tool_action = ActionClient(node, Tool, "/niryo_robot_tools_commander/action_server")
        self._tool_cfg = {
            "id": 11,  # Gripper tool ID
            "max": 100,  # Max torque percentage
            "hold": 100  # Hold torque percentage (align with working node)
        }
        if not self._tool_action.wait_for_server(timeout_sec=5.0):
            self._logger.error("tool server /niryo_robot_tools_commander/action_server not available !")
        
        # Initialize MoveIt2 Python API (parameters will be passed via launch file)
        try:
            self._moveit = MoveItPy(node_name="moveit_py_packaging")
            self._arm = self._moveit.get_planning_component(group_name)
            self._planning_scene_monitor = self._moveit.get_planning_scene_monitor()
            
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
        """Helper function to plan and execute a motion"""
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
            self._moveit.execute(robot_trajectory, controllers=[])
            time.sleep(sleep_time)
            return True
        else:
            self._logger.error("Planning failed")
            return False

    def _compose_pose_stamped(self, pose_input, frame_id: str = "base_link") -> PoseStamped:
        """Build a PoseStamped from supported inputs.
        Accepted:
        - PoseStamped
        - dict with keys x,y,z,qx,qy,qz,qw
        - list/tuple of 7 floats [x,y,z,qx,qy,qz,qw]
        """
        if isinstance(pose_input, PoseStamped):
            return pose_input

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self._node.get_clock().now().to_msg()

        if isinstance(pose_input, dict):
            pose.pose.position.x = float(pose_input.get("x", 0.0))
            pose.pose.position.y = float(pose_input.get("y", 0.0))
            pose.pose.position.z = float(pose_input.get("z", 0.0))
            pose.pose.orientation.x = float(pose_input.get("qx", 0.0))
            pose.pose.orientation.y = float(pose_input.get("qy", 0.0))
            pose.pose.orientation.z = float(pose_input.get("qz", 0.0))
            pose.pose.orientation.w = float(pose_input.get("qw", 1.0))
            return pose

        if isinstance(pose_input, (list, tuple)) and len(pose_input) == 7:
            x, y, z, qx, qy, qz, qw = pose_input
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.x = float(qx)
            pose.pose.orientation.y = float(qy)
            pose.pose.orientation.z = float(qz)
            pose.pose.orientation.w = float(qw)
            return pose

        raise ValueError("Unsupported pose format. Provide PoseStamped, dict{x,y,z,qx,qy,qz,qw}, or [x,y,z,qx,qy,qz,qw].")




    def use_single_pipeline_planning(self, target=None, pipeline_name: str = "ompl_rrtc", pose_link: str = "tool_link") -> bool:
        """Single-pipeline planning using either joints or Cartesian pose.
        - If target is a list of 6 floats → interpreted as joint targets
        - If target is PoseStamped | dict | list[7] → interpreted as Cartesian pose
        """
        try:
            # set plan start state to current state
            self._arm.set_start_state_to_current_state()

            goal_set = False
            if isinstance(target, (list, tuple)) and len(target) == 6:
                robot_model = self._moveit.get_robot_model()
                robot_state = RobotState(robot_model)
                joint_names = [f"joint_{i+1}" for i in range(6)]
                joint_values = dict(zip(joint_names, target))
                robot_state.joint_positions = joint_values
                joint_constraint = construct_joint_constraint(
                    robot_state=robot_state,
                    joint_model_group=self._moveit.get_robot_model().get_joint_model_group(self._group_name),
                )
                self._arm.set_goal_state(motion_plan_constraints=[joint_constraint])
                goal_set = True
            elif target is not None:
                pose_msg = self._compose_pose_stamped(target)
                self._arm.set_goal_state(pose_stamped_msg=pose_msg, pose_link=pose_link)
                goal_set = True

            if not goal_set:
                raise ValueError("Provide either 6 joint values or a Cartesian pose (PoseStamped/dict/list[7]).")

            params = MultiPipelinePlanRequestParameters(self._moveit, [pipeline_name])
            self._logger.info(f"Using single pipeline: {pipeline_name}")
            return self.plan_and_execute(multi_plan_parameters=params, sleep_time=0.0)

        except Exception as e:
            self._logger.error(f"Error in single-pipeline planning ({pipeline_name}): {e}")
            return False

    def use_multi_pipeline_planning(self, target=None, pose_link: str = "tool_link") -> bool:
        """Multi-pipeline planning using either joints or Cartesian pose.
        - If target is a list of 6 floats → interpreted as joint targets
        - If target is PoseStamped | dict | list[7] → interpreted as Cartesian pose
        """
        try:
            # set plan start state to current state
            self._arm.set_start_state_to_current_state()

            goal_set = False
            if isinstance(target, (list, tuple)) and len(target) == 6:
                robot_model = self._moveit.get_robot_model()
                robot_state = RobotState(robot_model)
                joint_names = [f"joint_{i+1}" for i in range(6)]
                joint_values = dict(zip(joint_names, target))
                robot_state.joint_positions = joint_values
                joint_constraint = construct_joint_constraint(
                    robot_state=robot_state,
                    joint_model_group=self._moveit.get_robot_model().get_joint_model_group(self._group_name),
                )
                self._arm.set_goal_state(motion_plan_constraints=[joint_constraint])
                goal_set = True
            elif target is not None:
                pose_msg = self._compose_pose_stamped(target)
                self._arm.set_goal_state(pose_stamped_msg=pose_msg, pose_link=pose_link)
                goal_set = True

            if not goal_set:
                raise ValueError("Provide either 6 joint values or a Cartesian pose (PoseStamped/dict/list[7]).")

            params = MultiPipelinePlanRequestParameters(self._moveit, ["ompl_rrtc", "pilz_lin", "pliz_ptp"])
            return self.plan_and_execute(multi_plan_parameters=params, sleep_time=0.0)

        except Exception as e:
            self._logger.error(f"Error in multi-pipeline planning: {e}")
            return False

    def _tool_cmd(self, cmd_type: int, activate: bool):
        """Send tool command (gripper open/close) using action server"""
        goal = Tool.Goal()
        goal.cmd.cmd_type = cmd_type
        goal.cmd.tool_id = self._tool_cfg["id"]
        goal.cmd.activate = activate
        goal.cmd.max_torque_percentage = self._tool_cfg["max"]
        goal.cmd.hold_torque_percentage = self._tool_cfg["hold"]
        
        send_future = self._tool_action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future)
        goal_handle = send_future.result()
        
        if not goal_handle.accepted:
            self._logger.error("Tool command rejected")
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        self._logger.info(f"Tool cmd completed (cmd_type={cmd_type}, activate={activate})")
        return True

    def close_gripper(self) -> bool:
        """Close the gripper"""
        self._logger.info("Closing gripper")
        return self._tool_cmd(cmd_type=2, activate=True)  # cmd_type=2: close

    def open_gripper(self) -> bool:
        """Open the gripper"""
        self._logger.info("Opening gripper")
        return self._tool_cmd(cmd_type=1, activate=True)  # cmd_type=1: open

    def add_collision_object(self, object_id: str, object_type: str = "box", 
                           position: tuple = (0.0, 0.0, 0.0), 
                           dimensions: tuple = (0.1, 0.1, 0.1),
                           orientation: tuple = (0.0, 0.0, 0.0, 1.0),
                           frame_id: str = "base_link") -> bool:
        """
        Add a collision object to the planning scene.
        
        Args:
            object_id (str): Unique identifier of the object
            object_type (str): Type d'objet ("box", "cylinder", "sphere")
            position (tuple): Position (x, y, z) in meters
            dimensions (tuple): Dimensions according to the type:
                - box: (length, width, height)
                - cylinder: (radius, height, 0)
                - sphere: (radius, 0, 0)
            orientation (tuple): Orientation quaternion (qx, qy, qz, qw)
            frame_id (str): Reference frame (default: "base_link")
        
        Returns:
            bool: True if the object has been added successfully
        """
        try:
            with self._planning_scene_monitor.read_write() as scene:
                collision_object = CollisionObject()
                collision_object.header.frame_id = frame_id
                collision_object.id = object_id
                
                # Create the primitive according to the type
                primitive = SolidPrimitive()
                
                if object_type.lower() == "box":
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = list(dimensions)
                elif object_type.lower() == "cylinder":
                    primitive.type = SolidPrimitive.CYLINDER
                    primitive.dimensions = [dimensions[0], dimensions[1]]  # radius, height
                elif object_type.lower() == "sphere":
                    primitive.type = SolidPrimitive.SPHERE
                    primitive.dimensions = [dimensions[0]]  # radius only
                else:
                    self._logger.error(f"Unsupported object type: {object_type}")
                    return False
                
                # Create the pose
                pose = Pose()
                pose.position.x = position[0]
                pose.position.y = position[1]
                pose.position.z = position[2]
                pose.orientation.x = orientation[0]
                pose.orientation.y = orientation[1]
                pose.orientation.z = orientation[2]
                pose.orientation.w = orientation[3]
                
                # Add to the collision object
                collision_object.primitives.append(primitive)
                collision_object.primitive_poses.append(pose)
                collision_object.operation = CollisionObject.ADD
                
                # Apply to the scene
                scene.apply_collision_object(collision_object)
                scene.current_state.update()
                
                self._logger.info(f"Collision object '{object_id}' added: {object_type} at {position}")
                return True
                
        except Exception as e:
            self._logger.error(f"Error adding collision object '{object_id}': {e}")
            return False

    def remove_collision_object(self, object_id: str) -> bool:
        """
        Remove a collision object from the planning scene.
        
        Args:
            object_id (str): Identifier of the object to remove
        
        Returns:
            bool: True if the object has been removed successfully
        """
        try:
            with self._planning_scene_monitor.read_write() as scene:
                collision_object = CollisionObject()
                collision_object.id = object_id
                collision_object.operation = CollisionObject.REMOVE
                
                scene.apply_collision_object(collision_object)
                scene.current_state.update()
                
                self._logger.info(f"Collision object '{object_id}' removed")
                return True
                
        except Exception as e:
            self._logger.error(f"Error removing collision object '{object_id}': {e}")
            return False

    def remove_all_collision_objects(self) -> bool:
        """
        Remove all collision objects from the planning scene.
        
        Returns:
            bool: True if all objects have been removed successfully
        """
        try:
            with self._planning_scene_monitor.read_write() as scene:
                scene.remove_all_collision_objects()
                scene.current_state.update()
                
                self._logger.info("All collision objects have been removed")
                return True
                
        except Exception as e:
            self._logger.error(f"Error removing all collision objects: {e}")
            return False

    def check_collision(self, robot_state=None, joint_group_name: str = "arm") -> bool:
        """
        Check if the current robot state is in collision.
        
        Args:
            robot_state: Robot state to check (None for the current state)
            joint_group_name (str): Name of the joint group
        
        Returns:
            bool: True if the robot is in collision
        """
        try:
            with self._planning_scene_monitor.read_only() as scene:
                if robot_state is None:
                    robot_state = scene.current_state
                
                collision_status = scene.is_state_colliding(
                    robot_state=robot_state, 
                    joint_model_group_name=joint_group_name, 
                    verbose=True
                )
                
                self._logger.info(f"Collision state: {'IN COLLISION' if collision_status else 'NO COLLISION'}")
                return collision_status
                
        except Exception as e:
            self._logger.error(f"Error checking collision: {e}")
            return False


class PackagingNodeMoveIt2(Node):
    def __init__(self):
        super().__init__("packaging_node_moveit2")

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

        # --- Helpers ---
        self.conveyor = ConveyorControllerSync(self, self.conveyor_service, self.conveyor_id, self.speed)
        
        # Load poses from YAML file
        self.poses = self._load_poses()
        
        # Initialize MoveIt2
        try:
            self.pick_place = PickAndPlaceExecutorMoveIt2(self, "arm")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MoveIt2: {e}")
            self.pick_place = None

        # --- State ---
        self._last_object_detected = None
        self._loop_count = 0  # Counter to follow the number of passages in the loop

        # --- Subscription ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(DigitalIOState, self.digital_state_topic, self._on_digital_state, qos)

        self.get_logger().info("packaging_node started: monitoring sensor, controlling conveyor and robot")

        #self.pick_place.add_collision_object(
        #    object_id="obstacle_box",
        #    object_type="box",
        #    position=(0.2, 0.1, 0.0),
        #    dimensions=(0.05, 0.05, 0.6)
        #)

    def _load_poses(self) -> dict:
        """Load poses from poses.yaml file"""
        try:
            package_share_directory = get_package_share_directory('ned3pro_packaging_manager')
            poses_file_path = os.path.join(package_share_directory, 'config', 'poses.yaml')
            
            with open(poses_file_path, 'r') as file:
                poses_data = yaml.safe_load(file)
            
            self.get_logger().info(f"Loaded poses: {list(poses_data['poses'].keys())}")
            return poses_data['poses']
            
        except Exception as e:
            self.get_logger().error(f"Failed to load poses.yaml: {e}")
            # Return default poses if file loading fails
            return {
                'grip': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'safe': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            }

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

            # Object detected → stop conveyor and execute pick sequence
            self.conveyor.set_running(False)
            self.get_logger().info("Object detected - conveyor stopped, executing pick sequence")
            
            if self.pick_place is not None:
                # Execute pick sequence: grip position → close gripper → return to 0.0
                self._execute_pick_sequence()
            else:
                self.get_logger().warn("MoveIt2 not available, skipping robot movement")
            
            self._last_object_detected = False  # Reset to wait for the next object

    def _execute_pick_sequence(self):
        """Execute the complete pick sequence: grip position -> close gripper -> home pose -> intermediate position -> open gripper -> return to grip"""
        try:
            # Example of adding collision objects to simulate obstacles
            # You can uncomment these lines to test
            
            # # Add a box as an obstacle
            # self.pick_place.add_collision_object(
            #     object_id="obstacle_box",
            #     object_type="box",
            #     position=(0.2, 0.1, 0.3),
            #     dimensions=(0.1, 0.1, 0.2)
            # )
            
            # # Add a cylinder as an obstacle
            # self.pick_place.add_collision_object(
            #     object_id="obstacle_cylinder",
            #     object_type="cylinder",
            #     position=(0.15, -0.1, 0.4),
            #     dimensions=(0.05, 0.15, 0)  # radius=0.05m, height=0.15m
            # )

            grip_pose = self.poses.get('grip')
            success = self.pick_place.use_single_pipeline_planning(grip_pose, "pilz_ptp")
            if not success:
                self.get_logger().error("Failed to reach grip position")
                return False

            
            gripper_success = self.pick_place.close_gripper()
            if not gripper_success:
                self.get_logger().error("Failed to close gripper")
                return False
            
            home_pose = self.poses.get('place')
            success2 = self.pick_place.use_single_pipeline_planning(home_pose, "pilz_ptp")
            if not success2:
                self.get_logger().error("Failed to reach home position")
                return False
            
            # 4. Go to the intermediate position with decrementing x
            intermediate_x = 0.09177011656393808 - (self._loop_count * 0.03)
            intermediate_pose = {
                'x': intermediate_x,
                'y': 0.2841085251085336,
                'z': 0.17462129016934397,
                'qx': -0.5402673034761233,
                'qy': 0.43502689461930927,
                'qz': 0.5052171450691023,
                'qw': 0.5134379009001422
            }
            
            self.get_logger().info(f"Intermediate position #{self._loop_count + 1}: x={intermediate_x:.6f}")
            success_intermediate = self.pick_place.use_single_pipeline_planning(intermediate_pose, "pilz_ptp")
            if not success_intermediate:
                self.get_logger().error("Failed to reach intermediate position")
                return False
            
            # 5. Open the gripper
            gripper_success2 = self.pick_place.open_gripper()
            if not gripper_success2:
                self.get_logger().error("Failed to open gripper")
                return False
            success3 = self.pick_place.use_single_pipeline_planning(grip_pose, "pilz_ptp")
            if not success3:
                self.get_logger().error("Failed to reach grip position")
                return False
            
            # Increment the counter for the next iteration
            self._loop_count += 1
            
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error in pick sequence: {e}")
            return False


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
