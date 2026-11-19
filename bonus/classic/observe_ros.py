"""Observer ROS et conserver son image finale, sans commander les moteurs."""

import json, time
from pathlib import Path
import cv2, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from cv_bridge import CvBridge


class Observer(Node):
    def __init__(self):
        super().__init__("classic_observer")
        self.state = ""
        self.simtime = 0.0
        self.frame = None
        self.bridge = CvBridge()
        self.create_subscription(String, "/brain_state", self.status, 10)
        self.create_subscription(Image, "/external_cam/image_raw", self.image, 1)
        self.create_subscription(JointState, "/joint_states", self.joints, 10)

    def status(self, msg):
        self.state = msg.data

    def image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "rgb8").copy()

    def joints(self, msg):
        self.simtime = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def main():
    rclpy.init()
    node = Observer()
    deadline = time.monotonic() + 600
    start = None
    before = None
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                node.state.startswith("ECHEC")
                or node.state == "PYRAMIDE_A_VERIFIER_PHYSIQUEMENT"
            ):
                if start is None:
                    start = node.simtime
                    before = None if node.frame is None else node.frame.copy()
                if node.simtime - start >= 5 and node.frame is not None:
                    break
        out = Path("/project/bonus/classic/results")
        out.mkdir(exist_ok=True)
        if node.frame is not None:
            cv2.imwrite(
                str(out / "ros_final.png"), cv2.cvtColor(node.frame, cv2.COLOR_RGB2BGR)
            )
        report = {
            "state": node.state,
            "sequence_completed": node.state == "PYRAMIDE_A_VERIFIER_PHYSIQUEMENT",
            "physical_ground_truth_checked": False,
            "observed_simulation_seconds": (
                None if start is None else node.simtime - start
            ),
            "mean_image_change": (
                None
                if before is None or node.frame is None
                else float(
                    np.mean(np.abs(node.frame.astype(float) - before.astype(float)))
                )
            ),
        }
        (out / "ros_observation.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)
        return 0 if report["sequence_completed"] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
