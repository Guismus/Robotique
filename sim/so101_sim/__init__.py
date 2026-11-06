# SO101Sim — MuJoCo simulation backend of the SO-ARM101.
#
# Exposes STRICTLY the same interface as lerobot.robots.so_follower.SO101Follower:
#   connect() / disconnect() / get_observation() / send_action() / is_connected
#
# The ROS2 driver node must work the same way
# with SO101Sim or SO101Follower (ROS2 parameter `use_sim`).
#
# Units: radians for the 5 arm joints, 0-100 for the gripper
# (same as SO101FollowerConfig(use_radians=True) on the real arm).

import time
from pathlib import Path

import mujoco
import numpy as np

ASSETS_DIR = Path(__file__).resolve().parent / "assets/so101"
DEFAULT_SCENE = ASSETS_DIR / "scene_tek5.xml"

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER = "gripper"
ALL_JOINTS = ARM_JOINTS + [GRIPPER]

# Gripper opening: mapping 0-100 (LeRobot convention) <-> MJCF radians
GRIPPER_RANGE_RAD = None  # filled at init from the model


class SO101Sim:
    """Digital twin of the SO-ARM101. Same API as SO101Follower."""

    def __init__(
        self,
        scene_path: str | Path = DEFAULT_SCENE,
        camera_name: str = "external_cam",
        camera_width: int = 320,
        camera_height: int = 240,
        sim_dt_per_step: int = 5,
        seed: int | None = None,
    ):
        self.scene_path = str(scene_path)
        self.camera_name = camera_name
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.sim_dt_per_step = sim_dt_per_step
        self._rng = np.random.default_rng(seed)

        self.model: mujoco.MjModel | None = None
        self.data: mujoco.MjData | None = None
        self._renderer: mujoco.Renderer | None = None
        self._connected = False
        self._last_step_t = 0.0
        self._gripper_effort_target = None

    # ------------------------------------------------------------------ API

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, calibrate: bool = True) -> None:  # same signature as the real arm
        self.model = mujoco.MjModel.from_xml_path(self.scene_path)
        self.data = mujoco.MjData(self.model)

        self._joint_qpos_adr = {
            name: self.model.joint(name).qposadr[0] for name in ALL_JOINTS
        }
        self._actuator_id = {
            name: self.model.actuator(name).id for name in ALL_JOINTS
        }
        g_jnt = self.model.joint(GRIPPER)
        self._gripper_range = (float(g_jnt.range[0]), float(g_jnt.range[1]))
        self._arm_targets = {name: float(self.data.ctrl[self._actuator_id[name]]) for name in ARM_JOINTS}
        self._gripper_position_target = float(self.data.ctrl[self._actuator_id[GRIPPER]])
        self._reference_velocity = dict.fromkeys(ALL_JOINTS, 0.)

        mujoco.mj_forward(self.model, self.data)
        self.randomize_object()
        self._connected = True
        self._last_step_t = time.perf_counter()

    def disconnect(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        self._connected = False

    def get_observation(self) -> dict:
        """Same keys as the real arm ('<joint>.pos').
        The camera image is NOT included; callers must call
        render_camera() explicitly when they need it.
        """
        self._require_connected()
        self._step_sim()
        obs = {}
        for name in ARM_JOINTS:
            obs[f"{name}.pos"] = float(self.data.qpos[self._joint_qpos_adr[name]])
        obs[f"{GRIPPER}.pos"] = self._gripper_rad_to_pct(
            self.data.qpos[self._joint_qpos_adr[GRIPPER]]
        )
        for name in ALL_JOINTS:
            # Hinge actuator force is joint torque (Nm), a motor observation,
            # not an object/contact oracle. Real backends may omit this key.
            obs[f"{name}.effort"] = float(self.data.actuator_force[self._actuator_id[name]])
        return obs

    def send_action(self, action: dict) -> dict:
        """Position targets. Expected keys: '<joint>.pos' (radians, gripper 0-100)."""
        self._require_connected()
        applied = {}
        for key, val in action.items():
            name = key.removesuffix(".pos")
            if name not in self._actuator_id:
                continue
            if name == GRIPPER:
                target = self._gripper_pct_to_rad(val)
                self._gripper_position_target = target
            else:
                self._arm_targets[name] = float(val)
            applied[key] = val
        self._step_sim()
        return applied

    # ------------------------------------------------------- sim utilities

    def render_camera(self) -> np.ndarray:
        """RGB image (H, W, 3) uint8 of the fixed external camera."""
        if self._renderer is None:
            self._renderer = mujoco.Renderer(
                self.model, height=self.camera_height, width=self.camera_width
            )
        self._renderer.update_scene(self.data, camera=self.camera_name)
        return self._renderer.render()

    def randomize_object(self, radius_range=(0.18, 0.30), angle_range_deg=(-50, 60)) -> None:
        """Moves pickup_object to a reachable position, with a random yaw."""
        r = self._rng.uniform(*radius_range)
        a = np.radians(self._rng.uniform(*angle_range_deg))
        yaw = self._rng.uniform(0, np.pi / 2)  # the cube is invariant under 90° rotation
        adr = self.model.joint("pickup_object_free").qposadr[0]
        self.data.qpos[adr : adr + 3] = [r * np.cos(a), r * np.sin(a), 0.03]
        self.data.qpos[adr + 3 : adr + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        self.data.qvel[:] = 0
        # let the object settle on the ground (physics settling)
        for _ in range(200):
            mujoco.mj_step(self.model, self.data)

    def get_object_position(self) -> np.ndarray:
        """World position of the object — DEBUG/EVALUATION only.
        Forbidden in the pipeline: the position must come from perception."""
        return self.data.body("pickup_object").xpos.copy()

    def get_drop_box_position(self) -> np.ndarray:
        """World position of the drop_box."""
        return self.data.body("drop_box").xpos.copy()

    def get_camera_extrinsics(self) -> np.ndarray:
        """Camera->world pose (4x4). This is the 'provided TF' of the subject."""
        cam = self.data.camera(self.camera_name)
        T = np.eye(4)
        T[:3, :3] = cam.xmat.reshape(3, 3)
        T[:3, 3] = cam.xpos
        return T

    def get_camera_intrinsics(self) -> np.ndarray:
        """K matrix (3x3) derived from the MJCF fovy and the render resolution."""
        cam_id = self.model.camera(self.camera_name).id
        fovy = np.radians(self.model.cam_fovy[cam_id])
        fy = self.camera_height / (2 * np.tan(fovy / 2))
        fx = fy  # square pixels
        return np.array(
            [
                [fx, 0, (self.camera_width - 1) / 2],
                [0, fy, (self.camera_height - 1) / 2],
                [0, 0, 1],
            ]
        )

    # -------------------------------------------------------------- private

    def _step_sim(self) -> None:
        for _ in range(self.sim_dt_per_step):
            self.step_once()

    def set_gripper_effort(self, torque: float | None) -> None:
        """Optional hybrid motor command. Does not modify actuator physics.

        Existing position-servo gains convert the requested closing torque
        into a position reference each physics tick. The original ctrlrange
        and forcerange still apply. No object/contact state is consulted.
        """
        if torque is not None and (not np.isfinite(torque) or torque > 0):
            raise ValueError("Gripper effort must be a finite closing torque")
        if self._gripper_effort_target is not None and torque is None:
            self.data.ctrl[self._actuator_id[GRIPPER]] = self.data.qpos[self._joint_qpos_adr[GRIPPER]]
            self._reference_velocity[GRIPPER] = float(self.data.qvel[self.model.joint(GRIPPER).dofadr[0]])
        self._gripper_effort_target = torque

    def step_once(self) -> None:
        # Interpolate incoming position samples at the motor tick. The gains
        # and force limits stay unchanged; acceleration limits belong to the
        # command trajectory, avoiding impulses from 20Hz stair-step targets.
        dt = self.model.opt.timestep
        for name in ALL_JOINTS:
            if name == GRIPPER and self._gripper_effort_target is not None:
                continue
            actuator = self._actuator_id[name]
            goal = self._gripper_position_target if name == GRIPPER else self._arm_targets[name]
            maximum_velocity, acceleration = (.6, 1.2) if name == GRIPPER else (.5, .8)
            error = goal - self.data.ctrl[actuator]
            desired_velocity = np.sign(error) * min(maximum_velocity, np.sqrt(2 * acceleration * abs(error)))
            velocity = self._reference_velocity[name] + np.clip(desired_velocity - self._reference_velocity[name], -acceleration * dt, acceleration * dt)
            increment = velocity * dt
            if abs(increment) >= abs(error):
                self.data.ctrl[actuator] = goal
                self._reference_velocity[name] = 0.
            else:
                self.data.ctrl[actuator] += increment
                self._reference_velocity[name] = velocity
        if self._gripper_effort_target is not None:
            actuator = self._actuator_id[GRIPPER]
            address = self._joint_qpos_adr[GRIPPER]
            dof = self.model.joint(GRIPPER).dofadr[0]
            kp = self.model.actuator_gainprm[actuator, 0]
            kv = -self.model.actuator_biasprm[actuator, 2]
            reference = self.data.qpos[address] + (self._gripper_effort_target + kv * self.data.qvel[dof]) / kp
            self.data.ctrl[actuator] = np.clip(reference, *self.model.actuator_ctrlrange[actuator])
        mujoco.mj_step(self.model, self.data)

    def advance(self, seconds: float) -> None:
        """Advance from the driver's control clock, independently of sensor reads."""
        if not np.isfinite(seconds) or seconds < 0:
            raise ValueError("Expected a finite nonnegative simulation duration")
        for _ in range(round(seconds / self.model.opt.timestep)):
            self.step_once()

    def _gripper_pct_to_rad(self, pct: float) -> float:
        lo, hi = self._gripper_range
        return lo + np.clip(pct, 0, 100) / 100.0 * (hi - lo)

    def _gripper_rad_to_pct(self, rad: float) -> float:
        lo, hi = self._gripper_range
        return float(np.clip((rad - lo) / (hi - lo) * 100.0, 0, 100))

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("SO101Sim not connected: call connect() first.")
