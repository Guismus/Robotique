#!/usr/bin/env python3
"""driver_node.py - ROS2 driver of the SO-ARM101.

The only node that talks to the backend: SO101Sim (MuJoCo) or SO101Follower
(real arm), chosen by the `use_sim` parameter. The rest of the stack never
knows which one runs.

TODO:
  - Publish /joint_states in RADIANS (>= 20 Hz)
  - Subscribe to /joint_command (radians, gripper 0-100 %)
"""
from pathlib import Path

import math
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray
import rclpy
import rclpy.node
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from so101_sim import SO101Sim

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Gripper joint limits in the URDF (rad).
# LeRobot convention: 0 % = closed (lower), 100 % = open (upper).
GRIPPER_RANGE_RAD = (-0.174533, 1.74533)

# Calibration file of the real arm, mounted by docker compose (CALIBRATION_FILE in docker/.env).
CALIBRATION_FILE = Path("/calibration/arm.json")
DEFAULT_PORT = "/dev/ttyACM0"

CONTROL_HZ = 50.0
JOINT_STATE_HZ = 30.0


def _gripper_pct_to_rad(pct: float) -> float:
    return GRIPPER_RANGE_RAD[0] + max(0., min(100., pct)) / 100 * (GRIPPER_RANGE_RAD[1] - GRIPPER_RANGE_RAD[0])


class DriverNode(rclpy.node.Node):
    """Driver SO-ARM101: /joint_command -> backend -> /joint_states."""

    def __init__(self):
        super().__init__("so101_driver")
        self._use_sim: bool = self.declare_parameter("use_sim", True).value
        port: str = self.declare_parameter("port", DEFAULT_PORT).value
        mode = "simulation" if self._use_sim else "hardware"
        self.get_logger().info(f"Mode: {mode} (port={port})")

        self._robot = self._connect_arm(port)
        self.get_logger().info(f"Robot connected: {type(self._robot).__name__}")

        self._command = self._robot.get_observation()
        self._states = self.create_publisher(JointState, '/joint_states', 10)
        # QoS capteur (BEST_EFFORT) : une image perdue n'est pas renvoyee, on prefere la suivante (10 Hz).
        self._images = self.create_publisher(Image, '/external_cam/image_raw', qos_profile_sensor_data)
        self._info = self.create_publisher(CameraInfo, '/external_cam/camera_info', 1)
        self._pose = self.create_publisher(Float64MultiArray, '/external_cam/pose', 1)
        self._bridge = CvBridge()
        self.create_subscription(JointState, '/joint_command', self._cb_joint_command, 10)
        self.create_timer(1 / CONTROL_HZ, self._control_step)
        self.create_timer(1 / JOINT_STATE_HZ, self._publish_joint_states)
        if self._use_sim:
            self.create_timer(0.1, self._publish_camera)

        self.get_logger().info("Driver node ready.")

    def _connect_arm(self, port: str = DEFAULT_PORT) -> SO101Sim | SO101Follower:
        """Creates and connects the backend chosen by use_sim (provided)."""
        if self._use_sim:
            robot = SO101Sim(seed=self.declare_parameter('seed', 1).value)
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
        robot.bus.disable_torque()  # Comment out this line to control the robot.
        self.get_logger().info(
            "Torque disabled: arm can be moved freely by hand. Remove this part to control the arm."
        )
        return robot

    def destroy_node(self):
        """Disconnects the backend (provided). On the real arm this disables the torque."""
        if self._robot.is_connected:
            self._robot.disconnect()
            self.get_logger().info("Robot disconnected.")
        super().destroy_node()

    def _cb_joint_command(self, msg: JointState):
        # Une commande partielle ne change que les moteurs nommes.
        limits = [(-1.91986, 1.91986), (-1.74533, 1.74533), (-1.69, 1.69),
                  (-1.65806, 1.65806), (-2.74385, 2.84121), (0, 100)]
        if len(msg.name) != len(msg.position):
            return
        for name, value in zip(msg.name, msg.position):
            if name not in JOINT_NAMES or not math.isfinite(value):
                return
        for name, value in zip(msg.name, msg.position):
            lo, hi = limits[JOINT_NAMES.index(name)]
            self._command[name + '.pos'] = max(lo, min(hi, value))

    def _control_step(self):
        self._robot.send_action(self._command)

    def _publish_joint_states(self):
        obs = self._robot.get_observation()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(obs[n + '.pos']) for n in JOINT_NAMES]
        msg.position[-1] = _gripper_pct_to_rad(msg.position[-1])
        self._states.publish(msg)

    def _publish_camera(self):
        image = self._bridge.cv2_to_imgmsg(self._robot.render_camera(), 'rgb8')
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = 'external_cam'
        info = CameraInfo()
        info.header = image.header
        info.width, info.height = image.width, image.height
        info.k = self._robot.get_camera_intrinsics().flatten().tolist()
        # MuJoCo regarde vers -Z ; OpenCV regarde vers +Z, Y vers le bas.
        transform = self._robot.get_camera_extrinsics() @ np.diag([1., -1., -1., 1.])
        pose = Float64MultiArray(data=transform.flatten().tolist())
        self._info.publish(info)
        self._pose.publish(pose)
        self._images.publish(image)


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
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
