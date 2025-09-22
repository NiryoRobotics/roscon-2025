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
            return self.plan_and_execute(sleep_time=0.0)
                
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
            return self.plan_and_execute(sleep_time=0.0)
                
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

    def use_single_pipeline_planning(self, target=None, pipeline_name: str = "ompl_rrtc", pose_link: str = "hand_link") -> bool:
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

    def use_multi_pipeline_planning(self, target=None, pose_link: str = "hand_link") -> bool:
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
        Ajouter un objet de collision à la scène de planification.
        
        Args:
            object_id (str): Identifiant unique de l'objet
            object_type (str): Type d'objet ("box", "cylinder", "sphere")
            position (tuple): Position (x, y, z) en mètres
            dimensions (tuple): Dimensions selon le type:
                - box: (longueur, largeur, hauteur)
                - cylinder: (rayon, hauteur, 0)
                - sphere: (rayon, 0, 0)
            orientation (tuple): Orientation quaternion (qx, qy, qz, qw)
            frame_id (str): Frame de référence (défaut: "base_link")
        
        Returns:
            bool: True si l'objet a été ajouté avec succès
        """
        try:
            with self._planning_scene_monitor.read_write() as scene:
                collision_object = CollisionObject()
                collision_object.header.frame_id = frame_id
                collision_object.id = object_id
                
                # Créer la primitive selon le type
                primitive = SolidPrimitive()
                
                if object_type.lower() == "box":
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = list(dimensions)
                elif object_type.lower() == "cylinder":
                    primitive.type = SolidPrimitive.CYLINDER
                    primitive.dimensions = [dimensions[0], dimensions[1]]  # rayon, hauteur
                elif object_type.lower() == "sphere":
                    primitive.type = SolidPrimitive.SPHERE
                    primitive.dimensions = [dimensions[0]]  # rayon seulement
                else:
                    self._logger.error(f"Type d'objet non supporté: {object_type}")
                    return False
                
                # Créer la pose
                pose = Pose()
                pose.position.x = position[0]
                pose.position.y = position[1]
                pose.position.z = position[2]
                pose.orientation.x = orientation[0]
                pose.orientation.y = orientation[1]
                pose.orientation.z = orientation[2]
                pose.orientation.w = orientation[3]
                
                # Ajouter à l'objet de collision
                collision_object.primitives.append(primitive)
                collision_object.primitive_poses.append(pose)
                collision_object.operation = CollisionObject.ADD
                
                # Appliquer à la scène
                scene.apply_collision_object(collision_object)
                scene.current_state.update()
                
                self._logger.info(f"Objet de collision '{object_id}' ajouté: {object_type} à {position}")
                return True
                
        except Exception as e:
            self._logger.error(f"Erreur lors de l'ajout de l'objet de collision '{object_id}': {e}")
            return False

    def remove_collision_object(self, object_id: str) -> bool:
        """
        Supprimer un objet de collision de la scène de planification.
        
        Args:
            object_id (str): Identifiant de l'objet à supprimer
        
        Returns:
            bool: True si l'objet a été supprimé avec succès
        """
        try:
            with self._planning_scene_monitor.read_write() as scene:
                collision_object = CollisionObject()
                collision_object.id = object_id
                collision_object.operation = CollisionObject.REMOVE
                
                scene.apply_collision_object(collision_object)
                scene.current_state.update()
                
                self._logger.info(f"Objet de collision '{object_id}' supprimé")
                return True
                
        except Exception as e:
            self._logger.error(f"Erreur lors de la suppression de l'objet de collision '{object_id}': {e}")
            return False

    def remove_all_collision_objects(self) -> bool:
        """
        Supprimer tous les objets de collision de la scène de planification.
        
        Returns:
            bool: True si tous les objets ont été supprimés avec succès
        """
        try:
            with self._planning_scene_monitor.read_write() as scene:
                scene.remove_all_collision_objects()
                scene.current_state.update()
                
                self._logger.info("Tous les objets de collision ont été supprimés")
                return True
                
        except Exception as e:
            self._logger.error(f"Erreur lors de la suppression de tous les objets de collision: {e}")
            return False

    def check_collision(self, robot_state=None, joint_group_name: str = "arm") -> bool:
        """
        Vérifier si l'état actuel du robot est en collision.
        
        Args:
            robot_state: État du robot à vérifier (None pour l'état actuel)
            joint_group_name (str): Nom du groupe de joints
        
        Returns:
            bool: True si le robot est en collision
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
                
                self._logger.info(f"État de collision: {'EN COLLISION' if collision_status else 'SANS COLLISION'}")
                return collision_status
                
        except Exception as e:
            self._logger.error(f"Erreur lors de la vérification de collision: {e}")
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

        # --- Subscription ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(DigitalIOState, self.digital_state_topic, self._on_digital_state, qos)

        self.get_logger().info("packaging_node_moveit2 started: monitoring sensor, controlling conveyor and robot")

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
        """Execute the complete pick sequence: grip position → close gripper → return to 0.0"""
        try:
            # Exemple d'ajout d'objets de collision pour simuler des obstacles
            # Vous pouvez décommenter ces lignes pour tester
            
            # # Ajouter une boîte comme obstacle
            # self.pick_place.add_collision_object(
            #     object_id="obstacle_box",
            #     object_type="box",
            #     position=(0.2, 0.1, 0.3),
            #     dimensions=(0.1, 0.1, 0.2)
            # )
            
            # # Ajouter un cylindre comme obstacle
            # self.pick_place.add_collision_object(
            #     object_id="obstacle_cylinder",
            #     object_type="cylinder",
            #     position=(0.15, -0.1, 0.4),
            #     dimensions=(0.05, 0.15, 0)  # rayon=0.05m, hauteur=0.15m
            # )

            grip_pose = self.poses.get('grip')
            success = self.pick_place.use_single_pipeline_planning(grip_pose, "ompl_rrtc")

            
            gripper_success = self.pick_place.close_gripper()

            home_pose = self.poses.get('place')
            success2 = self.pick_place.use_single_pipeline_planning(home_pose, "ompl_rrtc")
            gripper_success2 = self.pick_place.open_gripper()
            success3 = self.pick_place.use_single_pipeline_planning(grip_pose, "ompl_rrtc")
            if not success2 and success and gripper_success and gripper_success2 and success3:
                self.get_logger().error("Failed trajectory")
                return False
            
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
