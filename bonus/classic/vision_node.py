"""Noeud vision : apprentissage des pixels, geometrie pour les positions."""

import json, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray, String
from cv_bridge import CvBridge
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformBroadcaster
import display
from vision import CubeVision
from calibrated_planning import TARGETS


class Vision(Node):
    def __init__(self):
        super().__init__("classic_vision")
        self.vision = CubeVision()
        self.bridge = CvBridge()
        self.k = self.t = None
        self.pub = self.create_publisher(String, "/bonus/vision_state", 1)
        self.overlay_pub = self.create_publisher(Image, "/perception/overlay", 1)
        self.marker_pub = self.create_publisher(MarkerArray, "/perception/markers", 1)
        self.tf = TransformBroadcaster(self)
        # Un emplacement vu rempli le reste : le cube pose au-dessus le cache ensuite.
        self.seen_done = [False] * len(TARGETS)
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
        stamp = msg.header.stamp
        self.seen_done = [a or bool(b) for a, b in zip(self.seen_done, checks)]
        image = self.bridge.cv2_to_imgmsg(
            display.overlay(rgb, labels, cubes, TARGETS, self.seen_done, self.k, self.t), "bgr8")
        image.header = msg.header
        self.overlay_pub.publish(image)
        self.marker_pub.publish(display.markers(stamp, cubes, TARGETS, self.seen_done))
        self.tf.sendTransform(display.frames(stamp, cubes, self.t))


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
