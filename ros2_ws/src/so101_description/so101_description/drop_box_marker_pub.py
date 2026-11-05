"""
drop_box_marker_pub.py

Subscribes to /drop_box_position (PoseStamped) and publishes a
cube marker on /drop_box_marker for RViz.
Also subscribes to /object_position (PoseStamped) and publishes a
red 5cm cube marker on /object_marker for RViz.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class DropBoxMarkerPublisher(Node):
    """Publishes RViz markers for the drop_box and the red pickup cube."""

    def __init__(self) -> None:
        super().__init__("drop_box_marker_pub")
        self._sub_box = self.create_subscription(
            PoseStamped, "drop_box_position", self._cb_position, 10
        )
        self._pub_box = self.create_publisher(Marker, "drop_box_marker", 10)

        # 10cm × 10cm × 5cm cube (same size as the MuJoCo drop_box)
        self._marker = Marker()
        self._marker.header.frame_id = "world"
        self._marker.ns = "drop_box"
        self._marker.id = 0
        self._marker.type = Marker.CUBE
        self._marker.action = Marker.ADD
        self._marker.scale.x = 0.10
        self._marker.scale.y = 0.10
        self._marker.scale.z = 0.05
        self._marker.color.r = 0.45
        self._marker.color.g = 0.25
        self._marker.color.b = 0.15
        self._marker.color.a = 0.7  # semi-transparent
        self._marker.lifetime.sec = 0

        # 5cm × 5cm × 5cm cube for pickup object
        self._sub_obj = self.create_subscription(
            PoseStamped, "object_position", self._cb_obj_position, 10
        )
        self._pub_obj = self.create_publisher(Marker, "object_marker", 10)
        self._obj_marker = Marker()
        self._obj_marker.header.frame_id = "world"
        self._obj_marker.ns = "pickup_object"
        self._obj_marker.id = 1
        self._obj_marker.type = Marker.CUBE
        self._obj_marker.action = Marker.ADD
        self._obj_marker.scale.x = 0.05
        self._obj_marker.scale.y = 0.05
        self._obj_marker.scale.z = 0.05
        self._obj_marker.color.r = 0.85
        self._obj_marker.color.g = 0.05
        self._obj_marker.color.b = 0.22
        self._obj_marker.color.a = 0.95
        self._obj_marker.lifetime.sec = 0

    def _cb_position(self, msg: PoseStamped) -> None:
        self._marker.header.stamp = self.get_clock().now().to_msg()
        self._marker.pose = msg.pose
        self._pub_box.publish(self._marker)

    def _cb_obj_position(self, msg: PoseStamped) -> None:
        self._obj_marker.header.stamp = self.get_clock().now().to_msg()
        self._obj_marker.pose = msg.pose
        self._pub_obj.publish(self._obj_marker)


def main(args=None):
    rclpy.init(args=args)
    node = DropBoxMarkerPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
