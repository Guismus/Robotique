"""Driver de simulation : seul noeud autorise a commander MuJoCo."""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
from simulation_test import Simulation


class Driver(Node):
    def __init__(self):
        super().__init__("classic_driver")
        seed = self.declare_parameter("seed", 0).value
        self.sim = Simulation(seed, vision=False)
        self.sim.arm_speed = 0.08
        self.bridge = CvBridge()
        self.states = self.create_publisher(JointState, "/joint_states", 10)
        self.images = self.create_publisher(Image, "/external_cam/image_raw", 1)
        self.info = self.create_publisher(CameraInfo, "/external_cam/camera_info", 1)
        self.pose = self.create_publisher(Float64MultiArray, "/external_cam/pose", 1)
        self.create_subscription(JointState, "/joint_command", self.command, 10)
        self.create_timer(0.01, self.tick)
        self.create_timer(1.0, self.camera)

    def command(self, msg):
        s = self.sim
        if (
            list(msg.name) != s.names
            or len(msg.position) != 6
            or not np.isfinite(msg.position).all()
        ):
            return
        if msg.effort and (len(msg.effort) != 6 or not np.isfinite(msg.effort).all()):
            return
        q = np.asarray(msg.position[:5])
        low, high = s.kin.bounds
        s.command[:5] = np.clip(q, low, high)
        s.command[5] = -0.174533 + np.clip(msg.position[5], 0, 100) / 100 * (
            1.74533 + 0.174533
        )
        effort = float(msg.effort[-1]) if msg.effort else 0.0
        s.force = float(np.clip(effort, -3.35, 0)) if effort < 0 else None

    def tick(self):
        s = self.sim
        for _ in range(10):
            s.step()
        msg = JointState()
        msg.name = s.names
        msg.header.stamp.sec = int(s.d.time)
        msg.header.stamp.nanosec = int((s.d.time - int(s.d.time)) * 1e9)
        msg.position = s.d.qpos[s.ids].tolist()
        msg.velocity = s.d.qvel[s.dofs].tolist()
        msg.effort = s.d.actuator_force[s.act].tolist()
        self.states.publish(msg)

    def camera(self):
        s = self.sim
        msg = self.bridge.cv2_to_imgmsg(s.robot.render_camera(), "rgb8")
        msg.header.frame_id = "external_cam"
        msg.header.stamp.sec = int(s.d.time)
        msg.header.stamp.nanosec = int((s.d.time - int(s.d.time)) * 1e9)
        k = CameraInfo()
        k.header = msg.header
        k.width = msg.width
        k.height = msg.height
        k.k = s.robot.get_camera_intrinsics().flatten().tolist()
        self.info.publish(k)
        self.pose.publish(
            Float64MultiArray(
                data=(s.robot.get_camera_extrinsics() @ np.diag([1, -1, -1, 1]))
                .flatten()
                .tolist()
            )
        )
        self.images.publish(msg)


def main():
    rclpy.init()
    node = Driver()
    try:
        rclpy.spin(node)
    finally:
        node.sim.robot.disconnect()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
