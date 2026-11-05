#!/usr/bin/env python3
"""brain_node.py - intelligence of the arm: kinematics and pick & place state machine.

Subscribes to:
  - /joint_states (sensor_msgs/JointState)
  - /object_position or /ball_position (geometry_msgs/PoseStamped)
  - /drop_box_position (geometry_msgs/PoseStamped)

Publishes:
  - /end_effector_pose (geometry_msgs/PoseStamped)
  - /joint_command (sensor_msgs/JointState)

Services:
  - /go_to_target (so101_interfaces/srv/GoToTarget)
"""
from enum import Enum, auto
from pathlib import Path
import threading
import time

import ikpy.chain
import numpy as np
import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from so101_interfaces.srv import GoToTarget

ARM_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
JOINT_NAMES = ARM_JOINT_NAMES + ["gripper"]

# Gripper commands in %
GRIPPER_OPEN = 100.0
GRIPPER_CLOSED = 0.0

# URDF path inside container or workspace
URDF_PATHS = [
    Path("/ros2_ws/src/so101_description/urdf/so101.urdf"),
    Path("/ros2_ws/install/so101_description/share/so101_description/urdf/so101.urdf"),
]


class State(Enum):
    INIT = auto()
    IDLE = auto()
    APPROACH = auto()
    LOWER = auto()
    GRASP = auto()
    LIFT = auto()
    TRANSPORT = auto()
    RELEASE = auto()
    RETRACT = auto()
    FINISHED = auto()


class BrainNode(rclpy.node.Node):
    """Brain of the arm: kinematics, FK/IK, and autonomous pick & place."""

    def __init__(self):
        super().__init__("so101_brain")
        self._lock = threading.Lock()

        # Load kinematic chain with ikpy
        self._chain = self._load_chain()
        self.get_logger().info(f"ikpy chain loaded with {len(self._chain.links)} links.")

        # Latest joint states (mapping name -> angle in rad)
        self._current_joints: dict[str, float] = {name: 0.0 for name in JOINT_NAMES}
        self._has_joint_states = False

        # Perception targets
        self._target_object_pos: np.ndarray | None = None
        self._target_box_pos: np.ndarray | None = None

        # State machine
        self._state = State.INIT
        self._state_timer = 0.0
        self._hold_steps = 0

        # Current commanded target {joint: val}
        self._cmd_q: np.ndarray | None = None
        self._cmd_gripper: float = GRIPPER_OPEN

        # Subscribers
        self._sub_joint_states = self.create_subscription(
            JointState, "/joint_states", self._cb_joint_states, 10
        )
        self._sub_obj_pos = self.create_subscription(
            PoseStamped, "/object_position", self._cb_object_pos, 10
        )
        self._sub_ball_pos = self.create_subscription(
            PoseStamped, "/ball_position", self._cb_object_pos, 10
        )
        self._sub_box_pos = self.create_subscription(
            PoseStamped, "/drop_box_position", self._cb_box_pos, 10
        )

        # Publishers
        self._pub_ee_pose = self.create_publisher(PoseStamped, "/end_effector_pose", 10)
        self._pub_joint_cmd = self.create_publisher(JointState, "/joint_command", 10)

        # Service
        self._srv_go_to = self.create_service(GoToTarget, "go_to_target", self._cb_go_to_target)

        # Control loop timer (20 Hz)
        self._timer_control = self.create_timer(0.05, self._control_loop)

        self.get_logger().info("Brain node ready.")

    def _load_chain(self) -> ikpy.chain.Chain:
        """Finds URDF and loads ikpy kinematic chain."""
        urdf_file = None
        for p in URDF_PATHS:
            if p.is_file():
                urdf_file = str(p)
                break
        if urdf_file is None:
            raise FileNotFoundError("Could not find so101.urdf in expected locations.")

        return ikpy.chain.Chain.from_urdf_file(
            urdf_file,
            active_links_mask=[False, True, True, True, True, True, False]
        )

    def _cb_joint_states(self, msg: JointState):
        """Update current joint states and publish forward kinematics EE pose."""
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._current_joints:
                    self._current_joints[name] = float(pos)
            self._has_joint_states = True

        self._publish_end_effector_pose()

    def _cb_object_pos(self, msg: PoseStamped):
        """Update detected object position."""
        with self._lock:
            self._target_object_pos = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ])

    def _cb_box_pos(self, msg: PoseStamped):
        """Update detected drop box position."""
        with self._lock:
            self._target_box_pos = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ])

    def _cb_go_to_target(self, request: GoToTarget.Request, response: GoToTarget.Response):
        """Service callback to move end-effector to a 3D target via IK."""
        target = [
            request.target_pose.pose.position.x,
            request.target_pose.pose.position.y,
            request.target_pose.pose.position.z,
        ]
        q_sol = self._solve_ik(target)
        if q_sol is not None:
            with self._lock:
                self._cmd_q = q_sol
            self._send_joint_command(q_sol)
            response.success = True
            response.message = f"IK solved, moving to target {target}"
        else:
            response.success = False
            response.message = f"Failed to solve IK for target {target}"
        return response

    def _forward_kinematics(self, q_arm: list[float] | np.ndarray) -> np.ndarray:
        """Computes 4x4 FK transformation matrix for the given arm joint angles (5 angles)."""
        q_full = [0.0] + list(q_arm) + [0.0]
        return self._chain.forward_kinematics(q_full)

    def _publish_end_effector_pose(self):
        """Publishes the current EE pose calculated from /joint_states via FK."""
        with self._lock:
            if not self._has_joint_states:
                return
            q_arm = [self._current_joints[name] for name in ARM_JOINT_NAMES]

        fk = self._forward_kinematics(q_arm)
        pos = fk[:3, 3]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.w = 1.0
        self._pub_ee_pose.publish(msg)

    def _solve_ik(self, target_position: list[float] | np.ndarray, initial_q: np.ndarray | None = None) -> np.ndarray | None:
        """Solves inverse kinematics with downward vertical orientation constraint."""
        with self._lock:
            if initial_q is not None:
                q_init = [0.0] + list(initial_q) + [0.0]
            else:
                q_init = [0.0] + [self._current_joints[name] for name in ARM_JOINT_NAMES] + [0.0]

        try:
            # Try vertical orientation first (gripper pointing straight down into table/cube)
            q_full = self._chain.inverse_kinematics(
                target_position,
                initial_position=q_init,
                target_orientation=[0, 0, -1],
                orientation_mode="Z",
            )
            return np.array(q_full[1:6])
        except Exception:
            try:
                # Fallback to unconstrained position-only IK
                q_full = self._chain.inverse_kinematics(target_position, initial_position=q_init)
                return np.array(q_full[1:6])
            except Exception as e:
                self.get_logger().error(f"IK solver error: {e}")
                return None

    def _send_joint_command(self, q_arm: np.ndarray, gripper_pct: float | None = None):
        """Publishes a JointState message on /joint_command."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = [float(x) for x in q_arm]

        if gripper_pct is not None:
            msg.name.append("gripper")
            msg.position.append(float(gripper_pct))

        self._pub_joint_cmd.publish(msg)

    def _control_loop(self):
        """State machine for autonomous pick & place."""
        with self._lock:
            if not self._has_joint_states:
                return
            obj_pos = self._target_object_pos.copy() if self._target_object_pos is not None else None
            box_pos = self._target_box_pos.copy() if self._target_box_pos is not None else np.array([0.05, -0.25, 0.0125])

        if self._state == State.INIT:
            home_q = np.array([0.0, -0.2, 0.4, 0.4, 0.0])
            self._cmd_q = home_q
            self._cmd_gripper = GRIPPER_OPEN
            self._send_joint_command(self._cmd_q, self._cmd_gripper)
            self._hold_steps += 1
            if self._hold_steps >= 20:  # 1 second
                self._hold_steps = 0
                self._state = State.IDLE
                self.get_logger().info("Arm at Home. Ready for pick & place.")

        elif self._state == State.IDLE:
            if obj_pos is not None:
                # Check that object is not already inside the drop box
                in_box = (box_pos[0] - 0.06 <= obj_pos[0] <= box_pos[0] + 0.06) and \
                         (box_pos[1] - 0.06 <= obj_pos[1] <= box_pos[1] + 0.06)
                if in_box:
                    # Object already delivered, wait in idle
                    return

                target_approach = obj_pos + np.array([0.0, 0.0, 0.08])
                q_sol = self._solve_ik(target_approach)
                if q_sol is not None:
                    self._cmd_q = q_sol
                    self._cmd_gripper = GRIPPER_OPEN
                    self._state = State.APPROACH
                    self._hold_steps = 0
                    self.get_logger().info(f"Target object spotted at {obj_pos}. Approaching...")

        elif self._state == State.APPROACH:
            self._send_joint_command(self._cmd_q, self._cmd_gripper)
            self._hold_steps += 1
            if self._hold_steps >= 35:
                self._hold_steps = 0
                target_grasp = obj_pos + np.array([0.0, 0.0, 0.018])
                q_grasp = self._solve_ik(target_grasp, initial_q=self._cmd_q)
                if q_grasp is not None:
                    self._cmd_q = q_grasp
                    self._state = State.LOWER
                    self.get_logger().info("Lowering gripper...")

        elif self._state == State.LOWER:
            self._send_joint_command(self._cmd_q, self._cmd_gripper)
            self._hold_steps += 1
            if self._hold_steps >= 35:
                self._hold_steps = 0
                self._cmd_gripper = GRIPPER_CLOSED
                self._state = State.GRASP
                self.get_logger().info("Grasping cube...")

        elif self._state == State.GRASP:
            self._send_joint_command(self._cmd_q, self._cmd_gripper)
            self._hold_steps += 1
            if self._hold_steps >= 30:
                self._hold_steps = 0
                target_lift = obj_pos + np.array([0.0, 0.0, 0.13])
                q_lift = self._solve_ik(target_lift, initial_q=self._cmd_q)
                if q_lift is not None:
                    self._cmd_q = q_lift
                    self._state = State.LIFT
                    self.get_logger().info("Lifting cube...")

        elif self._state == State.LIFT:
            self._send_joint_command(self._cmd_q, self._cmd_gripper)
            self._hold_steps += 1
            if self._hold_steps >= 35:
                self._hold_steps = 0
                target_box = box_pos + np.array([0.0, 0.0, 0.06])
                q_box = self._solve_ik(target_box, initial_q=self._cmd_q)
                if q_box is not None:
                    self._cmd_q = q_box
                    self._state = State.TRANSPORT
                    self.get_logger().info(f"Transporting to drop box at {target_box}...")

        elif self._state == State.TRANSPORT:
            self._send_joint_command(self._cmd_q, self._cmd_gripper)
            self._hold_steps += 1
            if self._hold_steps >= 45:
                self._hold_steps = 0
                self._cmd_gripper = GRIPPER_OPEN
                self._state = State.RELEASE
                self.get_logger().info("Releasing cube into box...")

        elif self._state == State.RELEASE:
            self._send_joint_command(self._cmd_q, self._cmd_gripper)
            self._hold_steps += 1
            if self._hold_steps >= 30:
                self._hold_steps = 0
                home_q = np.array([0.0, -0.2, 0.4, 0.4, 0.0])
                self._cmd_q = home_q
                self._state = State.RETRACT
                self.get_logger().info("Retracting arm to home...")

        elif self._state == State.RETRACT:
            self._send_joint_command(self._cmd_q, self._cmd_gripper)
            self._hold_steps += 1
            if self._hold_steps >= 35:
                self._hold_steps = 0
                self._state = State.FINISHED
                self.get_logger().info("Pick & Place cycle completed successfully!")

        elif self._state == State.FINISHED:
            self._send_joint_command(self._cmd_q, self._cmd_gripper)
            self._hold_steps += 1
            if self._hold_steps >= 60:
                self._hold_steps = 0
                self._state = State.IDLE


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
