#!/usr/bin/env python3
"""ROS-only feedback-driven pick & place. No simulator/backend imports."""
from enum import Enum, auto
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from so101_interfaces.srv import GoToTarget
from scipy.spatial.transform import Rotation
from .kinematics import (ARM_JOINT_NAMES, FIXED_JAW_CLEARANCE, GRIPPER_LOWER,
    GRIPPER_UPPER, HOME, Kinematics, gripper_percent, yaw_rotation, plan_pick)

JOINT_NAMES = ARM_JOINT_NAMES + ["gripper"]
GRIPPER_OPEN = 55.0
URDF_PATHS = [Path("/ros2_ws/src/so101_description/urdf/so101.urdf"),
    Path(__file__).resolve().parents[2] / "so101_description/urdf/so101.urdf"]


class State(Enum):
    INIT = auto()
    IDLE = auto()
    PLANNING = auto()
    APPROACH = auto()
    LOWER = auto()
    GRASP = auto()
    LIFT = auto()
    TRANSPORT = auto()
    PLACE = auto()
    RELEASE = auto()
    CLEAR = auto()
    RETRACT = auto()
    RECOVER = auto()
    VERIFY = auto()
    MANUAL = auto()
    FINISHED = auto()
    FAILED = auto()


class BrainNode(Node):
    def __init__(self):
        super().__init__("so101_brain")
        self._lock = threading.RLock()
        self._urdf_path = next(p for p in URDF_PATHS if p.is_file())
        self._kinematics = Kinematics(self._urdf_path)
        self._chain = self._kinematics.chain
        self._current_joints = dict.fromkeys(JOINT_NAMES, 0.)
        self._has_joint_states = False
        self._joint_stamp = -np.inf
        self._gripper_effort = None
        self._target_object_pos = self._target_box_pos = None
        self._object_stamp = self._box_stamp = -np.inf
        self._object_yaw = 0.
        self._object_rotation = np.eye(3)
        self._state = State.INIT
        self._entered = self._stable_since = None
        self._cmd_q = self._sent_q = None
        self._motion_positions = []
        self._planned_solutions = []
        self._state_timeout = 12.
        self._planner = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pick_preflight")
        self._planning_future = None
        self._planning_cancel = threading.Event()
        self._cmd_gripper = GRIPPER_OPEN
        self._failure_reason = self._ik_diagnostics = None
        self._attempts = 0
        self._grasp_origin = self._last_gripper = self._stall_since = self._grip_angle = None
        self._last_candidate = None
        self._candidate_stamp = -np.inf
        self._candidate_count = self._verify_count = 0
        self._verify_stamp = -np.inf
        self.create_subscription(JointState, "/joint_states", self._cb_joint_states, 10)
        self.create_subscription(PoseStamped, "/object_position", self._cb_object_pos, 10)
        self.create_subscription(PoseStamped, "/ball_position", self._cb_object_pos, 10)
        self.create_subscription(PoseStamped, "/drop_box_position", self._cb_box_pos, 10)
        self._pub_ee_pose = self.create_publisher(PoseStamped, "/end_effector_pose", 10)
        self._pub_joint_cmd = self.create_publisher(JointState, "/joint_command", 10)
        self.create_service(GoToTarget, "go_to_target", self._cb_go_to_target)
        self.create_timer(.05, self._control_loop)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _valid_pose(self, msg):
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        stamp = self._stamp(msg)
        if msg.header.frame_id not in ("world", "base_link") or not np.all(np.isfinite(pos)):
            return None
        if stamp <= 0 or not -.05 <= self._now() - stamp <= .6:
            return None
        return pos, stamp

    def _cb_joint_states(self, msg):
        values = dict(zip(msg.name, msg.position))
        stamp = self._stamp(msg)
        if stamp <= self._joint_stamp or not -.05 <= self._now() - stamp <= .3:
            return
        if not all(n in values and np.isfinite(values[n]) for n in JOINT_NAMES):
            return
        with self._lock:
            self._current_joints.update({n: float(values[n]) for n in JOINT_NAMES})
            self._joint_stamp = self._stamp(msg)
            efforts = dict(zip(msg.name, msg.effort))
            self._gripper_effort = float(efforts["gripper"]) if "gripper" in efforts and np.isfinite(efforts["gripper"]) else None
            self._has_joint_states = True
        self._publish_end_effector_pose()

    def _cb_object_pos(self, msg):
        valid = self._valid_pose(msg)
        if valid is None:
            return
        pos, stamp = valid
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        if not np.all(np.isfinite(q)) or np.linalg.norm(q) < .99:
            return
        with self._lock:
            if stamp <= self._object_stamp:
                return
            self._target_object_pos, self._object_stamp = pos, stamp
            self._object_yaw = float(Rotation.from_quat(q).as_euler("xyz")[2])
            self._object_rotation = Rotation.from_quat(q).as_matrix()

    def _cb_box_pos(self, msg):
        valid = self._valid_pose(msg)
        if valid is not None:
            with self._lock:
                self._target_box_pos, self._box_stamp = valid

    def _forward_kinematics(self, q_arm):
        return self._kinematics.forward(q_arm)

    def _publish_end_effector_pose(self):
        pose = self._forward_kinematics([self._current_joints[n] for n in ARM_JOINT_NAMES])
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pose[:3, 3])
        quat = Rotation.from_matrix(pose[:3, :3]).as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, quat)
        self._pub_ee_pose.publish(msg)

    def _solve_ik(self, target_position, initial_q=None, yaw=None, max_orientation=np.radians(28)):
        initial = np.array(initial_q if initial_q is not None else [self._current_joints[n] for n in ARM_JOINT_NAMES])
        selected_yaw = getattr(self, "_grasp_yaw", 0.) if yaw is None else yaw
        sol = self._kinematics.solve(target_position, initial, selected_yaw, max_orientation)
        self._ik_diagnostics = sol.diagnostics() if sol else {"target": list(target_position), "rejected": True}
        return sol.joints if sol else None

    def _cb_go_to_target(self, request, response):
        with self._lock:
            valid = self._valid_pose(request.target_pose)
            if self._state != State.IDLE or valid is None:
                response.success, response.message = False, "Requires IDLE and a fresh world/base_link pose"
                return response
            q = self._solve_ik(valid[0])
            response.success = q is not None
            response.message = "Validated IK target" if q is not None else "IK residual or joint limit rejected"
            if q is not None:
                self._cmd_q = q
                self._transition(State.MANUAL)
            return response

    def _send_joint_command(self, q_arm, gripper_pct=None):
        if self._sent_q is None:
            self._sent_q = np.array([self._current_joints[n] for n in ARM_JOINT_NAMES])
        step = .004 if self._state in (State.LIFT, State.PLACE) else (.008 if self._state == State.TRANSPORT else .025)
        self._sent_q += np.clip(q_arm - self._sent_q, -step, step)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = list(map(float, self._sent_q))
        if gripper_pct is not None:
            msg.name.append("gripper")
            msg.position.append(float(gripper_pct))
            # Optional motor torque mode. Position remains a bounded fallback
            # on backends without calibrated effort control/observations.
            msg.effort = [0.] * len(ARM_JOINT_NAMES) + [-.5 if self._state in (State.LIFT, State.TRANSPORT, State.PLACE) else 0.]
        self._pub_joint_cmd.publish(msg)

    def _transition(self, state):
        self.get_logger().info(f"{self._state.name} -> {state.name}")
        self._state, self._entered, self._stable_since = state, self._now(), None
        self._state_timeout = 12.

    def _activate_motion(self, motion, state):
        self._motion_positions = []
        self._planned_solutions = list(motion.solutions[1:])
        first = motion.solutions[0]
        self._cmd_q, self._ik_diagnostics = first.joints.copy(), first.diagnostics()
        actual = np.array([self._current_joints[n] for n in ARM_JOINT_NAMES])
        travel = np.max(np.abs(np.diff(np.vstack([actual, *[s.joints for s in motion.solutions]]), axis=0)), axis=1).sum()
        speed = .08 if state == State.LIFT else (.16 if state == State.TRANSPORT else .5)
        self._transition(state)
        self._state_timeout = max(12., 1.4 * (travel / speed + len(motion.solutions) * .05) + 2.)

    def _converged(self, now):
        actual = np.array([self._current_joints[n] for n in ARM_JOINT_NAMES])
        good = self._cmd_q is not None and np.max(np.abs(actual - self._cmd_q)) < .015
        if good and self._planned_solutions:
            solution = self._planned_solutions.pop(0)
            self._cmd_q, self._ik_diagnostics = solution.joints.copy(), solution.diagnostics()
            self._stable_since = None
            return False
        if good and self._motion_positions:
            target = self._motion_positions.pop(0)
            tolerance = self._motion_orientation if not self._motion_positions else np.radians(28)
            solution = self._solve_ik(target, self._cmd_q, max_orientation=tolerance)
            if solution is None:
                self._fail("unreachable_cartesian_waypoint")
            else:
                self._cmd_q = solution
                self._stable_since = None
            return False
        self._stable_since = (self._stable_since if self._stable_since is not None else now) if good else None
        return good and now - self._stable_since >= .2

    def _move(self, position, state, max_orientation=np.radians(28)):
        self._planned_solutions = []
        start = self._forward_kinematics([self._current_joints[n] for n in ARM_JOINT_NAMES])[:3, 3]
        count = 1 if state == State.APPROACH else max(1, int(np.ceil(np.linalg.norm(np.asarray(position) - start) / .005)))
        points = list(np.linspace(start, position, count + 1)[1:])
        q = self._solve_ik(points[0], self._cmd_q, max_orientation=max_orientation if count == 1 else np.radians(28))
        if q is None:
            self._fail(f"unreachable_{state.name.lower()}")
            return False
        self._cmd_q = q
        self._motion_positions, self._motion_orientation = points[1:], max_orientation
        self._transition(state)
        return True

    def _fail(self, reason):
        self._motion_positions = []
        self._planned_solutions = []
        self._planning_cancel.set()
        self._failure_reason = reason
        self.get_logger().warn(reason)
        if self._attempts >= 2 or self._state in (State.RELEASE, State.VERIFY, State.RECOVER):
            self._cmd_q = np.array([self._current_joints[n] for n in ARM_JOINT_NAMES])
            self._cmd_gripper = gripper_percent(self._current_joints["gripper"])
            self._transition(State.FAILED)
            return
        self._attempts += 1
        self._cmd_gripper = GRIPPER_OPEN
        actual = np.array([self._current_joints[n] for n in ARM_JOINT_NAMES])
        tcp = self._forward_kinematics(actual)[:3, 3]
        q = self._solve_ik(tcp + [0, 0, .035], actual)
        if q is None:
            self._cmd_q = actual
            self._transition(State.FAILED)
        else:
            self._cmd_q = q
            self._transition(State.RECOVER)

    def _holding(self, now):
        if self._target_object_pos is None or now - self._object_stamp > .6:
            return False
        tcp = self._forward_kinematics([self._current_joints[n] for n in ARM_JOINT_NAMES])[:3, 3]
        return (self._target_object_pos[2] >= self._grasp_origin[2] + .05 and
            # A retained corner can lie anywhere within the cube's half
            # diagonal plus the 14.3mm tip reach, including visual uncertainty.
            np.linalg.norm(self._target_object_pos - tcp) < .058 and
            self._current_joints["gripper"] > GRIPPER_LOWER + .15)

    def _control_loop(self):
        with self._lock:
            self._tick()

    def _tick(self):
        now = self._now()
        if not self._has_joint_states:
            return
        if self._entered is None:
            self._entered, self._cmd_q = now, HOME.copy()
        if now - self._joint_stamp > .3:
            self._failure_reason = "stale_joint_feedback"
            self._cmd_q = np.array([self._current_joints[n] for n in ARM_JOINT_NAMES])
            self._sent_q = self._cmd_q.copy()
            self._cmd_gripper = gripper_percent(self._current_joints["gripper"])
            self._transition(State.FAILED)
            self._send_joint_command(self._cmd_q, self._cmd_gripper)
            return
        elapsed, state = now - self._entered, self._state
        if state not in (State.IDLE, State.FINISHED, State.FAILED) and elapsed > self._state_timeout:
            self._fail(f"timeout_{state.name.lower()}")
        elif state == State.INIT:
            if self._converged(now):
                self._transition(State.IDLE)
        elif state == State.IDLE:
            fresh = self._target_object_pos is not None and now - self._object_stamp < .5
            box_fresh = self._target_box_pos is not None and now - self._box_stamp < .5
            if fresh and box_fresh and abs(self._target_object_pos[2] - .025) < .012:
                obj = self._target_object_pos.copy()
                if np.linalg.norm(obj[:2] - self._target_box_pos[:2]) < .035:
                    return
                if self._object_stamp != self._candidate_stamp:
                    self._candidate_count = self._candidate_count + 1 if self._last_candidate is not None and np.linalg.norm(obj - self._last_candidate) < .006 else 1
                    self._last_candidate, self._candidate_stamp = obj.copy(), self._object_stamp
                if self._candidate_count >= 3:
                    self._planning_cancel = threading.Event()
                    self._planning_future = self._planner.submit(plan_pick, self._urdf_path, obj.copy(),
                        self._target_box_pos.copy(), self._object_yaw, self._cmd_q.copy(), self._planning_cancel)
                    self._transition(State.PLANNING)
                    self._state_timeout = 30.
        elif state == State.PLANNING:
            if self._planning_future.done():
                try:
                    plan = self._planning_future.result()
                except Exception as error:
                    self.get_logger().error(f"Pick preflight failed: {error}")
                    plan = None
                if plan is None:
                    self._fail("unreachable_pick_path")
                elif now - self._object_stamp > .5 or np.linalg.norm(self._target_object_pos - plan.origin) > .01:
                    self._candidate_count = 0
                    self._transition(State.IDLE)
                else:
                    self._pick_plan = plan
                    self._grasp_origin, self._grasp_yaw = plan.origin, plan.yaw
                    self._grasp_target, self._box_target = plan.grasp_target, plan.box
                    self._cmd_gripper = GRIPPER_OPEN
                    self._activate_motion(plan.approach, State.APPROACH)
        elif state == State.APPROACH:
            if self._converged(now):
                if now - self._object_stamp > .6 or np.linalg.norm(self._target_object_pos - self._grasp_origin) > .015:
                    self._fail("target_lost_or_moved_before_lower")
                else:
                    self._activate_motion(self._pick_plan.lower, State.LOWER)
        elif state == State.LOWER:
            if self._converged(now):
                self._last_gripper, self._stall_since = self._current_joints["gripper"], None
                self._transition(State.GRASP)
        elif state == State.GRASP:
            angle = self._current_joints["gripper"]
            commanded = GRIPPER_LOWER + self._cmd_gripper / 100 * (GRIPPER_UPPER - GRIPPER_LOWER)
            # A stiff servo does not visibly stall before damaging the grasp.
            # At 0.1rad/s, unloaded torque stays below this measured threshold.
            loaded = self._gripper_effort is not None and -self._gripper_effort > .4 and elapsed > .3
            bounded_gap = self._gripper_effort is None and angle <= .59
            self._stall_since = (self._stall_since if self._stall_since is not None else now) if loaded or bounded_gap else None
            self._last_gripper = angle
            if angle < GRIPPER_LOWER + .05:
                self._fail("empty_gripper")
            elif self._stall_since is not None and now - self._stall_since >= .1:
                self._grip_angle = angle - .0002
                self._cmd_gripper = gripper_percent(self._grip_angle)
                self._activate_motion(self._pick_plan.lift, State.LIFT)
            else:
                self._cmd_gripper = gripper_percent(angle - .0005 if loaded else commanded - .005)
        elif state == State.LIFT:
            converged = self._converged(now)
            if converged and self._holding(now):
                self._last_holding_confirmation = now
                self._activate_motion(self._pick_plan.transport, State.TRANSPORT)
            elif converged and now - self._stable_since > 1.7:
                self._fail("lift_not_visually_confirmed")
        elif state == State.TRANSPORT:
            if self._holding(now):
                self._last_holding_confirmation = now
                if self._converged(now):
                    tcp = self._forward_kinematics([self._current_joints[n] for n in ARM_JOINT_NAMES])[:3, 3]
                    half_height = .025 * np.abs(self._object_rotation[2]).sum()
                    # The held cube is tilted: using a fixed centre height can
                    # press a lower corner through the box floor.
                    # Robot-aware occlusion bounds the measured height error;
                    # an 8mm clearance avoids an unnecessary impact at release.
                    support_height = self._box_target[2] - .021 + half_height + .008
                    self._move([*self._box_target[:2], tcp[2] + support_height - self._target_object_pos[2]], State.PLACE)
            elif now - self._last_holding_confirmation > 1.5:
                self._fail("grasp_lost_during_transport")
        elif state == State.PLACE:
            if self._converged(now):
                self._cmd_gripper = GRIPPER_OPEN
                self._transition(State.RELEASE)
        elif state == State.RELEASE:
            opened = abs(self._current_joints["gripper"] - (GRIPPER_LOWER + GRIPPER_OPEN / 100 * (GRIPPER_UPPER - GRIPPER_LOWER))) < .02
            self._stable_since = (self._stable_since if self._stable_since is not None else now) if opened else None
            if opened and now - self._stable_since >= .2:
                tcp = self._forward_kinematics([self._current_joints[n] for n in ARM_JOINT_NAMES])[:3, 3]
                # The fixed finger is still in the hollow top web. Disengage
                # vertically before the lateral sweep to Home.
                self._move([*tcp[:2], max(tcp[2] + .02, .10)], State.CLEAR)
        elif state == State.CLEAR:
            if self._converged(now):
                self._cmd_q = HOME.copy()
                self._transition(State.RETRACT)
        elif state == State.RETRACT:
            if self._converged(now):
                self._verify_count = 0
                self._transition(State.VERIFY)
        elif state == State.VERIFY:
            if self._object_stamp > self._verify_stamp and now - self._object_stamp < .5:
                self._verify_stamp = self._object_stamp
                p = self._target_object_pos
                half_width = .025 * np.abs(self._object_rotation[:2]).sum(axis=1)
                inside = np.all(np.abs(p[:2] - self._box_target[:2]) + half_width <= .048) and .022 <= p[2] <= .04
                self._verify_count = self._verify_count + 1 if inside else 0
                if self._verify_count >= 4:
                    self._transition(State.FINISHED)
            if elapsed > 4. and self._state != State.FINISHED:
                self._fail("deposit_not_visually_confirmed")
        elif state == State.RECOVER:
            if self._converged(now):
                self._cmd_q = HOME.copy()
                self._candidate_count, self._last_candidate = 0, None
                self._transition(State.INIT)
        elif state == State.MANUAL:
            if self._converged(now):
                self._transition(State.IDLE)
        if self._cmd_q is not None:
            self._send_joint_command(self._cmd_q, self._cmd_gripper)

    def destroy_node(self):
        self._planning_cancel.set()
        self._planner.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


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
