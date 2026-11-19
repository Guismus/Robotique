"""Noeud vision : apprentissage des pixels, geometrie pour les positions."""

import json, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray, String
from cv_bridge import CvBridge
from vision import CubeVision
from calibrated_planning import TARGETS


class Vision(Node):
    def __init__(self):
        super().__init__("classic_vision")
        self.vision = CubeVision()
        self.bridge = CvBridge()
        self.k = self.t = None
        self.pub = self.create_publisher(String, "/bonus/vision_state", 1)
        self.create_subscription(CameraInfo, "/external_cam/camera_info", self.info, 1)
        self.create_subscription(Float64MultiArray, "/external_cam/pose", self.pose, 1)
        self.create_subscription(Image, "/external_cam/image_raw", self.image, 1)

    def info(self, msg):
        self.k = np.array(msg.k).reshape(3, 3)

    def pose(self, msg):
        self.t = np.array(msg.data).reshape(4, 4)

    def image(self, msg):
        if self.k is None or self.t is None:
            return
        rgb = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        labels = self.vision.labels(rgb)
        cubes = self.vision.detect(rgb, self.k, self.t, labels=labels)
        for c in cubes:
            c["xyz"] = c["xyz"].tolist()
        checks = [
            self.vision.placement(rgb, self.k, self.t, p, color, labels=labels)
            for color, p in TARGETS
        ]
        self.pub.publish(
            String(data=json.dumps({"cubes": cubes, "placements": checks}))
        )


def main():
    rclpy.init()
    node = Vision()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
