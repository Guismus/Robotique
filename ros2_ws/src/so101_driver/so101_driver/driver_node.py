#!/usr/bin/env python3
"""driver_node.py - ROS2 driver of the SO-ARM101.

The only node that talks to the backend: SO101Sim (MuJoCo) or SO101Follower
(real arm), chosen by the `use_sim` parameter. The rest of the stack never
knows which one runs.
"""
from pathlib import Path
import threading

import numpy as np
import rclpy
import rclpy.node
from cv_bridge import CvBridge
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import CameraInfo, Image, JointState
from so101_interfaces.srv import SetJointPositions
from so101_sim import SO101Sim

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Joint limits in radians from URDF: [lower, upper]
JOINT_LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}

# Gripper joint limits in the URDF (rad).
# LeRobot convention: 0 % = closed (lower), 100 % = open (upper).
GRIPPER_RANGE_RAD = (-0.174533, 1.74533)

# Calibration file of the real arm, mounted by docker compose (CALIBRATION_FILE in docker/.env).
CALIBRATION_FILE = Path("/calibration/arm.json")
DEFAULT_PORT = "/dev/ttyACM0"

CONTROL_HZ = 50.0
JOINT_STATE_HZ = 30.0
CAMERA_HZ = 15.0


def _gripper_pct_to_rad(pct: float) -> float:
    """Convert gripper 0-100% to radians."""
    lo, hi = GRIPPER_RANGE_RAD
    return lo + float(np.clip(pct, 0.0, 100.0)) / 100.0 * (hi - lo)


def _gripper_rad_to_pct(rad: float) -> float:
    """Convert gripper radians to 0-100%."""
    lo, hi = GRIPPER_RANGE_RAD
    return float(np.clip((rad - lo) / (hi - lo) * 100.0, 0.0, 100.0))


class DriverNode(rclpy.node.Node):
    """Driver SO-ARM101: /joint_command -> backend -> /joint_states."""

    def __init__(self):
        super().__init__("so101_driver")
        self._lock = threading.Lock()
        self._use_sim: bool = self.declare_parameter("use_sim", True).value
        port: str = self.declare_parameter("port", DEFAULT_PORT).value
        mode = "simulation" if self._use_sim else "hardware"
        self.get_logger().info(f"Mode: {mode} (port={port})")

        self._robot = self._connect_arm(port)
        self.get_logger().info(f"Robot connected: {type(self._robot).__name__}")

        # Current commanded target {joint_name: value} (radians, gripper: 0-100)
        self._target_positions: dict[str, float] = {}
        self._target_gripper_effort = None
        self._effort_fallback_reported = False
        publish_clock = self.declare_parameter("publish_sim_clock", True).value
        self._pub_clock = self.create_publisher(ClockMessage, "/clock", 10) if (
            self._use_sim and publish_clock and self.get_parameter("use_sim_time").value) else None

        # Bridge for ROS Image messages
        self._cv_bridge = CvBridge()

        # Publishers
        self._pub_joint_states = self.create_publisher(JointState, "/joint_states", 10)
        self._pub_image = self.create_publisher(Image, "/external_cam/image_raw", 10)
        self._pub_camera_info = self.create_publisher(CameraInfo, "/external_cam/camera_info", 10)

        # Subscriber
        self._sub_joint_command = self.create_subscription(
            JointState, "/joint_command", self._cb_joint_command, 10
        )

        # Service for setting joint positions
        self._srv_set_joints = self.create_service(
            SetJointPositions, "set_joint_positions", self._cb_set_joint_positions
        )

        # Timers
        # This timer must run before /clock exists and survive system-clock
        # adjustments. All sensor/control consumers can use simulated time.
        self._control_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer_control = self.create_timer(1.0 / CONTROL_HZ, self._control_step,
            clock=self._control_clock)
        self._timer_joint_states = self.create_timer(1.0 / JOINT_STATE_HZ, self._publish_joint_states)
        if self._use_sim:
            # Rendering can take hundreds of milliseconds with software GL.
            # Motor/feedback callbacks remain in their own serial group.
            self._camera_group = MutuallyExclusiveCallbackGroup()
            self._timer_camera = self.create_timer(1.0 / CAMERA_HZ, self._publish_camera,
                callback_group=self._camera_group)

        self.get_logger().info("Driver node ready.")

    def _connect_arm(self, port: str = DEFAULT_PORT) -> SO101Sim | SO101Follower:
        """Creates and connects the backend chosen by use_sim (provided)."""
        if self._use_sim:
            # Only the 50Hz control timer advances physics. Publishing an image
            # or reading joint feedback must not change simulation speed.
            seed = self.declare_parameter("sim_seed", -1).value
            robot = SO101Sim(sim_dt_per_step=0, seed=seed if seed >= 0 else None)
            robot.connect()
            return robot

        if not CALIBRATION_FILE.is_file():
            raise RuntimeError(
                f"No calibration file at {CALIBRATION_FILE}: set CALIBRATION_FILE in docker/.env"
            )
        config = SO101FollowerConfig(
            port=port,
            id=CALIBRATION_FILE.stem,
            calibration_dir=CALIBRATION_FILE.parent,
            use_radians=True,
        )
        robot = SO101Follower(config)
        robot.connect(calibrate=False)
        if not robot.is_calibrated:
            robot.disconnect()
            raise RuntimeError(
                f"{CALIBRATION_FILE} does not match the motors: wrong file for this arm? "
                "Re-run lerobot-calibrate on the host."
            )
        return robot

    def destroy_node(self):
        """Disconnects the backend (provided). On the real arm this disables the torque."""
        if hasattr(self, "_robot") and self._robot.is_connected:
            self._robot.disconnect()
            self.get_logger().info("Robot disconnected.")
        super().destroy_node()

    def _cb_joint_command(self, msg: JointState):
        """Callback for incoming /joint_command messages.
        Supports partial commands (only named joints move).
        Arm joints in radians, gripper in 0-100%. Clamps to joint limits.
        """
        with self._lock:
            if "gripper" in msg.name:
                i = list(msg.name).index("gripper")
                effort = msg.effort[i] if i < len(msg.effort) else 0.
                self._target_gripper_effort = float(effort) if np.isfinite(effort) and effort < 0 else None
                if self._target_gripper_effort is not None and not self._use_sim and not self._effort_fallback_reported:
                    self.get_logger().warn("UNSUPPORTED_CAPABILITY: calibrated gripper effort control; using the bounded position fallback")
                    self._effort_fallback_reported = True
            for name, pos in zip(msg.name, msg.position):
                if name not in JOINT_NAMES or not np.isfinite(pos):
                    continue
                if name == "gripper":
                    clamped = float(np.clip(pos, 0.0, 100.0))
                else:
                    lo, hi = JOINT_LIMITS[name]
                    clamped = float(np.clip(pos, lo, hi))
                self._target_positions[name] = clamped

    def _cb_set_joint_positions(self, request: SetJointPositions.Request, response: SetJointPositions.Response):
        """Service callback: sets the 6 joints at once.
        Units: degrees for the 5 arm joints, 0-100 (%) for the gripper.
        """
        if len(request.positions) != 6 or not np.all(np.isfinite(request.positions)):
            response.success = False
            response.message = f"Expected 6 joint positions, got {len(request.positions)}"
            return response

        with self._lock:
            self._target_gripper_effort = None
            for idx, name in enumerate(JOINT_NAMES):
                val = request.positions[idx]
                if name == "gripper":
                    self._target_positions[name] = float(np.clip(val, 0.0, 100.0))
                else:
                    rad_val = np.radians(val)
                    lo, hi = JOINT_LIMITS[name]
                    self._target_positions[name] = float(np.clip(rad_val, lo, hi))

        response.success = True
        response.message = "Target joint positions updated."
        return response

    def _control_step(self):
        """Periodic control loop: sends active target positions to backend."""
        with self._lock:
            if not self._robot.is_connected:
                return
            action = {f"{k}.pos": v for k, v in self._target_positions.items()}
            gripper_effort = self._target_gripper_effort

        try:
            if action:
                self._robot.send_action(action)
            if self._use_sim:
                self._robot.set_gripper_effort(gripper_effort)
                self._robot.advance(1.0 / CONTROL_HZ)
                if self._pub_clock is not None:
                    message = ClockMessage()
                    nanoseconds = round(self._robot.simulation_time * 1e9)
                    message.clock.sec, message.clock.nanosec = divmod(nanoseconds, 1_000_000_000)
                    self._pub_clock.publish(message)
        except Exception as e:
            self.get_logger().error(f"Error in send_action: {e}")

    def _publish_joint_states(self):
        """Reads observations from backend and publishes /joint_states in radians."""
        if not self._robot.is_connected:
            return

        try:
            obs = self._robot.get_observation()
        except Exception as e:
            self.get_logger().error(f"Error reading observation: {e}")
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        positions = []
        if not all(f"{name}.pos" in obs and np.isfinite(obs[f"{name}.pos"]) for name in JOINT_NAMES):
            self.get_logger().error("Incomplete/nonfinite joint observation; feedback not published")
            return
        for name in JOINT_NAMES:
            raw_val = obs[f"{name}.pos"]
            if name == "gripper":
                # Backend observation gives 0-100 for gripper.
                # /joint_states for robot_state_publisher must be in radians!
                rad_val = _gripper_pct_to_rad(raw_val)
                positions.append(rad_val)
            else:
                positions.append(float(raw_val))

        msg.position = positions
        # Never label raw servo load/current as Nm without calibration.
        if all(f"{name}.effort" in obs for name in JOINT_NAMES):
            msg.effort = [float(obs[f"{name}.effort"]) for name in JOINT_NAMES]
        self._pub_joint_states.publish(msg)

    def _publish_camera(self):
        """Simulated camera publisher: publishes raw image and camera info."""
        if not self._use_sim or not self._robot.is_connected:
            return

        now = self.get_clock().now().to_msg()
        try:
            rgb_img = self._robot.render_camera()
        except Exception as e:
            self.get_logger().error(f"Error rendering camera: {e}")
            return

        # Image message
        img_msg = self._cv_bridge.cv2_to_imgmsg(rgb_img, encoding="rgb8")
        img_msg.header.stamp = now
        img_msg.header.frame_id = "external_cam"
        self._pub_image.publish(img_msg)

        # CameraInfo message
        K = self._robot.get_camera_intrinsics()
        info_msg = CameraInfo()
        info_msg.header.stamp = now
        info_msg.header.frame_id = "external_cam"
        info_msg.height = int(self._robot.camera_height)
        info_msg.width = int(self._robot.camera_width)
        info_msg.distortion_model = "plumb_bob"
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.k = [float(x) for x in K.flatten()]
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # P is 3x4 projection matrix [K | 0]
        P = np.zeros((3, 4))
        P[:3, :3] = K
        info_msg.p = [float(x) for x in P.flatten()]

        self._pub_camera_info.publish(info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
