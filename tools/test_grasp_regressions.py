"""Deterministic regressions; runs inside the project's ROS2 control image."""
import json
from pathlib import Path
import sys
import unittest
import hashlib
import threading
from types import SimpleNamespace
import cv2
import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, Image

ROOT = Path(__file__).resolve().parents[1]
for package in ("so101_brain", "so101_driver", "so101_perception"):
    sys.path.insert(0, str(ROOT / "ros2_ws/src" / package))
sys.path.insert(0, str(ROOT / "sim"))
from so101_brain.brain_node import BrainNode, State
from so101_brain.kinematics import (Kinematics, HOME, ARM_JOINT_NAMES, GRIPPER_LOWER, GRIPPER_UPPER,
    SHOULDER_XY, transport_arc, plan_pick)
from so101_driver.driver_node import DriverNode, _gripper_pct_to_rad, _gripper_rad_to_pct
from so101_perception.perception_node import PerceptionNode
from so101_perception.vision import detect_cube, detect_box
from so101_sim import SO101Sim


def pose(position, stamp=100., frame="world"):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int(round((stamp % 1) * 1e9))
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
    msg.pose.orientation.w = 1.
    return msg


class BrainRegressions(unittest.TestCase):
    def setUp(self):
        self.brain = BrainNode()
        self.time = 100.
        self.brain._now = lambda: self.time
        self.brain._send_joint_command = lambda *args: None

    def tearDown(self):
        self.brain.destroy_node()

    def test_stale_wrong_frame_and_nan_targets_rejected(self):
        for msg in (pose([.2, 0, .025], 98), pose([.2, 0, .025], frame="camera"), pose([np.nan, 0, .025])):
            self.brain._cb_object_pos(msg)
            self.assertIsNone(self.brain._target_object_pos)

    def test_alias_cannot_refresh_or_reorder_observation(self):
        self.brain._cb_object_pos(pose([.2, 0, .025]))
        self.time = 100.2
        self.brain._cb_object_pos(pose([.3, 0, .025], 99.9))
        self.brain._cb_object_pos(pose([.3, 0, .025], 100))
        np.testing.assert_allclose(self.brain._target_object_pos, [.2, 0, .025])
        self.assertEqual(self.brain._object_stamp, 100.)

    def test_convergence_requires_feedback_and_stability(self):
        self.brain._cmd_q = HOME.copy()
        self.assertFalse(self.brain._converged(100.))
        self.brain._current_joints.update(dict(zip(ARM_JOINT_NAMES, HOME)))
        self.assertFalse(self.brain._converged(100.))
        self.assertTrue(self.brain._converged(100.25))
        self.brain._current_joints["wrist_flex"] += .1
        self.assertFalse(self.brain._converged(101.))

    def test_no_motion_from_partial_joint_feedback(self):
        msg = JointState(name=["shoulder_pan"], position=[0.])
        self.brain._cb_joint_states(msg)
        self.assertFalse(self.brain._has_joint_states)

    def test_stale_joint_feedback_enters_failure(self):
        self.brain._has_joint_states = True
        self.brain._joint_stamp = 99.
        self.brain._tick()
        self.assertEqual(self.brain._state, State.FAILED)

    def test_empty_gripper_never_becomes_lift_or_finished(self):
        b = self.brain
        b._has_joint_states, b._joint_stamp, b._entered = True, 100., 99.
        b._state, b._attempts, b._cmd_q = State.GRASP, 2, HOME.copy()
        b._current_joints["gripper"] = b._last_gripper = GRIPPER_LOWER
        b._cmd_gripper = 0.
        b._tick()
        self.assertEqual(b._state, State.FAILED)
        self.assertEqual(b._failure_reason, "empty_gripper")

    def test_closed_gripper_without_visual_lift_is_not_holding(self):
        b = self.brain
        b._grasp_origin = np.array([.2, 0, .025])
        b._target_object_pos, b._object_stamp = np.array([.2, 0, .025]), 100.
        b._current_joints["gripper"] = .5
        self.assertFalse(b._holding(100.))
        b._target_object_pos[2] = .1
        self.assertFalse(b._holding(102.))

    def test_finished_requires_distinct_fresh_deposit_observations(self):
        b = self.brain
        b._has_joint_states, b._joint_stamp, b._entered = True, 100., 100.
        b._state, b._cmd_q = State.VERIFY, HOME.copy()
        b._box_target = np.array([.05, -.25, .025])
        b._target_object_pos, b._object_stamp = np.array([.05, -.25, .029]), 100.
        for _ in range(10):
            b._tick()
        self.assertEqual(b._state, State.VERIFY)
        for i in range(1, 4):
            self.time = b._joint_stamp = b._object_stamp = 100. + i * .1
            b._tick()
        self.assertEqual(b._state, State.FINISHED)

    def test_verify_rejects_cube_overhanging_box_rim(self):
        b = self.brain
        b._has_joint_states, b._joint_stamp, b._entered = True, 100., 100.
        b._state, b._cmd_q = State.VERIFY, HOME.copy()
        b._box_target = np.array([.05, -.25, .025])
        b._target_object_pos = np.array([.065, -.25, .029])
        from scipy.spatial.transform import Rotation
        b._object_rotation = Rotation.from_euler("z", np.pi / 4).as_matrix()
        for i in range(5):
            self.time = b._joint_stamp = b._object_stamp = 100. + i * .1
            b._tick()
        self.assertEqual(b._state, State.VERIFY)

    def test_release_waits_for_opening_then_clears_vertically(self):
        b = self.brain
        b._has_joint_states, b._joint_stamp, b._entered = True, 100., 99.
        b._state, b._cmd_q = State.RELEASE, HOME.copy()
        b._current_joints.update(dict(zip(ARM_JOINT_NAMES, HOME)))
        requested = []
        b._move = lambda target, state: requested.append((np.asarray(target), state))
        b._current_joints["gripper"] = .4
        b._tick()
        self.assertFalse(requested)
        b._current_joints["gripper"] = GRIPPER_LOWER + .55 * (GRIPPER_UPPER - GRIPPER_LOWER)
        for t in (100.1, 100.4):
            self.time = b._joint_stamp = t
            b._tick()
        self.assertEqual(requested[-1][1], State.CLEAR)
        current = b._forward_kinematics(HOME)[:3, 3]
        np.testing.assert_allclose(requested[-1][0][:2], current[:2])
        self.assertGreaterEqual(requested[-1][0][2], max(current[2] + .02, .10))

    def test_pending_preflight_keeps_home_command_without_blocking(self):
        from concurrent.futures import Future
        b = self.brain
        b._has_joint_states, b._joint_stamp, b._entered = True, 100., 100.
        b._state, b._cmd_q, b._planning_future = State.PLANNING, HOME.copy(), Future()
        commands = []
        b._send_joint_command = lambda q, grip: commands.append(q.copy())
        b._tick()
        self.assertFalse(b._planning_future.done())
        np.testing.assert_array_equal(commands[-1], HOME)

    def test_perception_preserves_source_stamp_and_expires(self):
        perception = PerceptionNode()
        try:
            now = [100.2]
            perception.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(now[0] * 1e9)))
            source = pose([.2, 0, .025], 100.).header.stamp
            observed = perception._pose(np.array([.2, 0, .025]), source)
            self.assertEqual(observed.header.stamp, source)
            now[0] = 100.7
            self.assertIsNone(perception._pose(np.array([.2, 0, .025]), source))
        finally:
            perception.destroy_node()


class PhysicsAndVisionRegressions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = SO101Sim(seed=0, sim_dt_per_step=0)
        cls.sim.connect()
        cls.kin = Kinematics(ROOT / "ros2_ws/src/so101_description/urdf/so101.urdf")

    @classmethod
    def tearDownClass(cls):
        cls.sim.disconnect()

    def test_slow_camera_uses_frozen_data_and_does_not_stop_physics(self):
        sim = SO101Sim(seed=0, sim_dt_per_step=0)
        sim.connect()
        rendering, release = threading.Event(), threading.Event()
        captured = []
        def render():
            rendering.set()
            release.wait(timeout=2.)
            return np.zeros((240, 320, 3), dtype=np.uint8)
        sim._renderer = SimpleNamespace(update_scene=lambda data, **kwargs: captured.append(data),
            render=render, close=lambda: None)
        worker = threading.Thread(target=sim.render_camera)
        try:
            worker.start()
            self.assertTrue(rendering.wait(timeout=2.))
            self.assertIsNot(captured[0], sim.data)
            old_time = captured[0].time
            sim.advance(.02)
            self.assertAlmostEqual(sim.data.time, old_time + .02)
            self.assertEqual(captured[0].time, old_time)
            self.assertFalse(release.is_set())
        finally:
            release.set()
            worker.join(timeout=3.)
            sim.disconnect()

    def test_driver_publishes_physics_clock_after_advancing(self):
        sim = SO101Sim(seed=0, sim_dt_per_step=0)
        sim.connect()
        messages = []
        driver = SimpleNamespace(_lock=threading.Lock(), _robot=sim,
            _target_positions={}, _target_gripper_effort=None, _use_sim=True,
            _pub_clock=SimpleNamespace(publish=messages.append))
        try:
            before = sim.simulation_time
            DriverNode._control_step(driver)
            self.assertAlmostEqual(sim.simulation_time, before + .02)
            self.assertEqual(len(messages), 1)
            stamp = messages[0].clock
            self.assertAlmostEqual(stamp.sec + stamp.nanosec * 1e-9, sim.simulation_time)
        finally:
            sim.disconnect()

    def test_slow_render_does_not_refresh_camera_capture_timestamp(self):
        now = [100.]
        images = []
        def render():
            now[0] += .5
            return np.zeros((240, 320, 3), dtype=np.uint8)
        driver = SimpleNamespace(_use_sim=True,
            _robot=SimpleNamespace(is_connected=True, render_camera=render,
                get_camera_intrinsics=lambda: np.eye(3), camera_width=320, camera_height=240),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: pose([0, 0, 0], now[0]).header.stamp)),
            _cv_bridge=SimpleNamespace(cv2_to_imgmsg=lambda *args, **kwargs: Image()),
            _pub_image=SimpleNamespace(publish=images.append),
            _pub_camera_info=SimpleNamespace(publish=lambda msg: None))
        DriverNode._publish_camera(driver)
        self.assertEqual(images[0].header.stamp.sec, 100)
        self.assertEqual(images[0].header.stamp.nanosec, 0)
        self.assertEqual(now[0], 100.5)

    def test_fk_frames_match_mujoco_position_and_orientation(self):
        for q in np.random.default_rng(73).uniform(-.7, .7, (20, 5)):
            for name, value in zip(ARM_JOINT_NAMES, q):
                self.sim.data.qpos[self.sim.model.joint(name).qposadr[0]] = value
            mujoco.mj_forward(self.sim.model, self.sim.data)
            fk = self.kin.forward(q)
            np.testing.assert_allclose(fk[:3, 3], self.sim.data.site("gripperframe").xpos, atol=4e-6)
            # URDF origins round pi/2 to 1.5708; MJCF uses CAD quaternions.
            np.testing.assert_allclose(fk[:3, :3], self.sim.data.site("gripperframe").xmat.reshape(3, 3), atol=2e-5)

    def test_unreachable_and_nonfinite_ik_rejected(self):
        self.assertIsNone(self.kin.solve([1., 0, 1.], HOME, 0.))
        self.assertIsNone(self.kin.solve([np.nan, 0, .1], HOME, 0.))

    def test_transport_arc_avoids_inner_unreachable_workspace(self):
        start, end = np.array([.14, .196, .125]), np.array([.05, -.25, .13])
        path = np.asarray(transport_arc(start, end))
        radius = np.linalg.norm(path[:, :2] - SHOULDER_XY, axis=1)
        self.assertGreaterEqual(radius.min(), min(np.linalg.norm(start[:2] - SHOULDER_XY), np.linalg.norm(end[:2] - SHOULDER_XY)) - 1e-10)
        np.testing.assert_allclose(path[-1], end)

    def test_full_path_preflight_selects_reachable_face_and_timeout(self):
        plan = plan_pick(ROOT / "ros2_ws/src/so101_description/urdf/so101.urdf", np.array([.140017, .196667, .025]),
            np.array([.05, -.25, .025]), .226, HOME)
        self.assertIsNotNone(plan)
        for motion in (plan.lower, plan.lift, plan.transport):
            for solution in motion.solutions:
                self.assertLessEqual(solution.position_error, .002)
                self.assertLessEqual(solution.orientation_error, np.radians(28))
                self.assertTrue(np.all(solution.joints >= self.kin.bounds[0]))
                self.assertTrue(np.all(solution.joints <= self.kin.bounds[1]))
        b = BrainNode()
        try:
            b._current_joints.update(dict(zip(ARM_JOINT_NAMES, plan.lift.solutions[-1].joints)))
            b._activate_motion(plan.transport, State.TRANSPORT)
            self.assertGreater(b._state_timeout, 12.)
        finally:
            b.destroy_node()

    def test_accepted_ik_meets_both_residual_limits(self):
        target = np.array([.24, -.09, .025])
        solution = self.kin.solve(target, HOME, np.pi / 2, np.radians(6))
        self.assertIsNotNone(solution)
        self.assertLessEqual(solution.position_error, .002)
        self.assertLessEqual(solution.orientation_error, np.radians(6))

    def test_gripper_zero_closed_hundred_open_and_roundtrip(self):
        self.assertAlmostEqual(_gripper_pct_to_rad(0), GRIPPER_LOWER)
        self.assertAlmostEqual(_gripper_pct_to_rad(100), GRIPPER_UPPER)
        for percent in (0., 25., 55., 100.):
            self.assertAlmostEqual(_gripper_rad_to_pct(_gripper_pct_to_rad(percent)), percent)

    def test_reads_do_not_advance_driver_configured_physics(self):
        before = self.sim.data.time
        for _ in range(10):
            self.sim.get_observation()
            self.sim.send_action({})
        self.assertEqual(self.sim.data.time, before)
        self.sim.advance(.02)
        self.assertAlmostEqual(self.sim.data.time - before, .02)

    def test_hybrid_command_uses_original_actuator_and_force_limits(self):
        sim = SO101Sim(seed=9, sim_dt_per_step=0)
        sim.connect()
        try:
            gains = sim.model.actuator_gainprm.copy()
            limits = sim.model.actuator_forcerange.copy()
            sim.data.qpos[sim.model.joint("gripper").qposadr[0]] = .6
            sim.data.qvel[:] = 0
            sim.set_gripper_effort(-.3)
            sim.step_once()
            self.assertAlmostEqual(sim.data.actuator_force[sim.model.actuator("gripper").id], -.3, places=8)
            np.testing.assert_array_equal(sim.model.actuator_gainprm, gains)
            np.testing.assert_array_equal(sim.model.actuator_forcerange, limits)
            sim.set_gripper_effort(None)
            self.assertIsNone(sim._gripper_effort_target)
        finally:
            sim.disconnect()

    def test_motor_reference_has_bounded_acceleration(self):
        sim = SO101Sim(seed=9, sim_dt_per_step=0)
        sim.connect()
        try:
            before = sim.data.ctrl[sim.model.actuator("shoulder_pan").id]
            sim.send_action({"shoulder_pan.pos": 1.})
            sim.step_once()
            self.assertLessEqual(abs(sim._reference_velocity["shoulder_pan"]), .8 * sim.model.opt.timestep + 1e-12)
            self.assertLessEqual(abs(sim.data.ctrl[sim.model.actuator("shoulder_pan").id] - before), .8 * sim.model.opt.timestep ** 2 + 1e-12)
        finally:
            sim.disconnect()

    def test_collision_parts_cover_original_stl_and_preserve_source(self):
        from scipy.spatial import ConvexHull
        directory = ROOT / "sim/so101_sim/assets/so101/assets"
        manifest = json.loads((directory / "moving_jaw_collision_manifest.json").read_text())
        raw = (directory / manifest["source"]).read_bytes()
        self.assertEqual(hashlib.sha256(raw).hexdigest(), manifest["source_sha256"])
        dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
        def stl(content):
            return np.frombuffer(content, dtype=dtype, count=int.from_bytes(content[80:84], "little"), offset=84)["vertices"]
        triangles = stl(raw)
        points = np.r_[triangles.reshape(-1, 3), triangles.mean(1)]
        outside = np.full(len(points), np.inf)
        for part in manifest["parts"]:
            data = (directory / (part["name"] + ".stl")).read_bytes()
            self.assertEqual(hashlib.sha256(data).hexdigest(), part["sha256"])
            hull = ConvexHull(np.unique(stl(data).reshape(-1, 3), axis=0))
            outside = np.minimum(outside, (points @ hull.equations[:, :3].T + hull.equations[:, 3]).max(1))
        self.assertLessEqual(outside.max(), .00075)

    @unittest.skipUnless((ROOT / "artifacts/baseline/scene_tek5.xml").is_file(), "Capture baseline to compare loaded physics")
    def test_mass_friction_actuation_and_joint_dynamics_preserved(self):
        from validate_pick_place import baseline_scene
        original = mujoco.MjModel.from_xml_path(str(baseline_scene()))
        for field in ("body_mass", "body_inertia", "dof_damping", "dof_frictionloss", "dof_armature", "actuator_gainprm", "actuator_biasprm", "actuator_forcerange", "actuator_ctrlrange"):
            np.testing.assert_allclose(getattr(self.sim.model, field), getattr(original, field), atol=1e-14)
        body = self.sim.model.body("moving_jaw_so101_v1").id
        new = self.sim.model.geom_friction[self.sim.model.geom_bodyid == body]
        old = original.geom_friction[(original.geom_bodyid == original.body("moving_jaw_so101_v1").id) & (original.geom_contype != 0)]
        for friction in new[self.sim.model.geom_contype[self.sim.model.geom_bodyid == body] != 0]:
            np.testing.assert_array_equal(friction, old[0])

    def test_brown_box_is_not_a_red_cube(self):
        image = np.zeros((240, 320, 3), np.uint8)
        cv2.rectangle(image, (100, 100), (150, 150), (89, 51, 31), -1)
        detected, _ = detect_cube(image, self.sim.get_camera_intrinsics())
        self.assertIsNone(detected)

    def test_calibrated_box_requires_visual_presence(self):
        self.assertIsNone(detect_box(np.zeros((240, 320, 3), np.uint8), self.sim.get_camera_intrinsics(), calibrated_center=[.05, -.25, .025]))

    def test_monocular_height_is_not_forced_to_table(self):
        for name, value in zip(ARM_JOINT_NAMES, HOME):
            self.sim.data.qpos[self.sim.model.joint(name).qposadr[0]] = value
        address = self.sim.model.joint("pickup_object_free").qposadr[0]
        self.sim.data.qpos[address:address + 3] = [.24, -.09, .12]
        mujoco.mj_forward(self.sim.model, self.sim.data)
        detected, _ = detect_cube(self.sim.render_camera(), self.sim.get_camera_intrinsics())
        self.assertIsNotNone(detected)
        self.assertLess(np.linalg.norm(detected.position - [.24, -.09, .12]), .015)

    def test_cube_in_box_remains_visible_with_wall_occlusion(self):
        for name, value in zip(ARM_JOINT_NAMES, HOME):
            self.sim.data.qpos[self.sim.model.joint(name).qposadr[0]] = value
        address = self.sim.model.joint("pickup_object_free").qposadr[0]
        self.sim.data.qpos[address:address + 7] = [.05, -.25, .029, 1, 0, 0, 0]
        mujoco.mj_forward(self.sim.model, self.sim.data)
        image = self.sim.render_camera()
        cv2.imwrite(str(ROOT / "artifacts/deposit_fixture.png"), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        detected, _ = detect_cube(image, self.sim.get_camera_intrinsics(), occlusion_box=[.05, -.25, .025])
        self.assertIsNotNone(detected)
        self.assertLess(np.linalg.norm(detected.position - [.05, -.25, .029]), .012)

    def test_fk_envelope_does_not_invent_a_visual_lift(self):
        for name, value in zip(ARM_JOINT_NAMES, HOME):
            self.sim.data.qpos[self.sim.model.joint(name).qposadr[0]] = value
        address = self.sim.model.joint("pickup_object_free").qposadr[0]
        self.sim.data.qpos[address:address + 7] = [.24, -.09, .025, 1, 0, 0, 0]
        mujoco.mj_forward(self.sim.model, self.sim.data)
        detected, _ = detect_cube(self.sim.render_camera(), self.sim.get_camera_intrinsics(), grasp_tcp=[.24, -.09, .125])
        self.assertIsNone(detected)

    def test_support_plane_does_not_confirm_hovering_cube_in_box(self):
        address = self.sim.model.joint("pickup_object_free").qposadr[0]
        self.sim.data.qpos[address:address + 7] = [.05, -.25, .12, 1, 0, 0, 0]
        mujoco.mj_forward(self.sim.model, self.sim.data)
        detected, _ = detect_cube(self.sim.render_camera(), self.sim.get_camera_intrinsics(), table_height=.029, occlusion_box=[.05, -.25, .025])
        if detected is not None:
            half = .025 * (abs(np.cos(detected.yaw)) + abs(np.sin(detected.yaw)))
            self.assertFalse(np.all(np.abs(detected.position[:2] - [.05, -.25]) + half <= .048))


if __name__ == "__main__":
    rclpy.init(args=["--ros-args", "-p", "use_sim:=true", "--log-level", "error"])
    try:
        program = unittest.main(exit=False, verbosity=2)
    finally:
        rclpy.shutdown()
    sys.exit(0 if program.result.wasSuccessful() else 1)
