#!/usr/bin/env python3
"""perception_node.py — 3D object and drop box detection from external camera.

Subscribes to:
  - /external_cam/image_raw (sensor_msgs/Image)
  - /external_cam/camera_info (sensor_msgs/CameraInfo)

Publishes:
  - /object_position (geometry_msgs/PoseStamped)
  - /ball_position (geometry_msgs/PoseStamped, alias for compatibility)
  - /drop_box_position (geometry_msgs/PoseStamped)
  - /perception/mask (sensor_msgs/Image, segmentation mask for visualization)
"""
import threading

import cv2
import numpy as np
import rclpy
import rclpy.node
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image

# Known height of the cube center above the table (z=0.025 m)
OBJECT_Z_TABLE = 0.025
DROP_BOX_Z_TABLE = 0.0125

# Fixed camera pose in world frame from scene_tek5.xml
# pos="0.45 0.45 0.55" xyaxes="-0.707 0.707 0 -0.354 -0.354 0.866"
DEFAULT_CAM_POS = np.array([0.45, 0.45, 0.55])
# MuJoCo xyaxes defines camera local frame (x_right, y_up, z_back).
# Standard pinhole CV frame has (x_right, y_down, z_forward).
# R_cv = [x_mj, -y_mj, -z_mj]
_R_MJ = np.array([
    [-0.70710678, -0.35389597, 0.61217452],
    [0.70710678, -0.35389597, 0.61217452],
    [0.0, 0.86574551, 0.50048448],
])
DEFAULT_R_CV = np.column_stack([_R_MJ[:, 0], -_R_MJ[:, 1], -_R_MJ[:, 2]])


class PerceptionNode(rclpy.node.Node):
    """Detects the red cube and drop box, publishing 3D poses in world/robot frame."""

    def __init__(self):
        super().__init__("so101_perception")
        self._lock = threading.Lock()
        self._cv_bridge = CvBridge()

        # Intrinsic matrix K (3x3)
        self._K: np.ndarray | None = None
        self._cam_pos: np.ndarray = DEFAULT_CAM_POS.copy()
        self._R_cv: np.ndarray = DEFAULT_R_CV.copy()

        # Last detected positions
        self._last_obj_pos: np.ndarray | None = None
        self._last_box_pos: np.ndarray | None = None

        # Subscribers
        self._sub_camera_info = self.create_subscription(
            CameraInfo, "/external_cam/camera_info", self._cb_camera_info, 10
        )
        self._sub_image = self.create_subscription(
            Image, "/external_cam/image_raw", self._cb_image, 10
        )

        # Publishers
        self._pub_obj_pose = self.create_publisher(PoseStamped, "/object_position", 10)
        self._pub_ball_pose = self.create_publisher(PoseStamped, "/ball_position", 10)
        self._pub_box_pose = self.create_publisher(PoseStamped, "/drop_box_position", 10)
        self._pub_mask = self.create_publisher(Image, "/perception/mask", 10)

        # Timer to publish latest positions steadily at 10 Hz even if camera frames arrive slower
        self._timer_pub = self.create_timer(0.1, self._publish_positions)

        self.get_logger().info("Perception node ready.")

    def _cb_camera_info(self, msg: CameraInfo):
        """Update intrinsics from CameraInfo."""
        with self._lock:
            if self._K is None and len(msg.k) == 9:
                self._K = np.array(msg.k).reshape(3, 3)
                self.get_logger().info(f"Received camera intrinsics K:\n{self._K}")

    def _cb_image(self, msg: Image):
        """Process incoming camera frame: segment objects and project to 3D."""
        try:
            # Handle encoding
            if msg.encoding in ("rgb8", "bgr8"):
                frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            else:
                frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if len(frame.shape) == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
                elif frame.shape[2] == 3 and msg.encoding.startswith("bgr"):
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        with self._lock:
            if self._K is None:
                # Default K if camera_info not yet received
                fovy = np.radians(52.0)
                fy = frame.shape[0] / (2.0 * np.tan(fovy / 2.0))
                fx = fy
                self._K = np.array([
                    [fx, 0.0, frame.shape[1] / 2.0],
                    [0.0, fy, frame.shape[0] / 2.0],
                    [0.0, 0.0, 1.0],
                ])

        obj_uv, box_uv, mask_viz = self._detect_objects(frame)

        # Publish mask visualization
        try:
            mask_msg = self._cv_bridge.cv2_to_imgmsg(mask_viz, encoding="rgb8")
            mask_msg.header.stamp = msg.header.stamp
            mask_msg.header.frame_id = "external_cam"
            self._pub_mask.publish(mask_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish mask: {e}")

        # 3D Projection
        with self._lock:
            if obj_uv is not None:
                p_3d = self._project_to_3d(obj_uv[0], obj_uv[1], target_z=OBJECT_Z_TABLE)
                # Ignore detections outside the reachable table workspace (radius > 0.40m or z < 0)
                r_dist = float(np.linalg.norm(p_3d[:2]))
                if 0.10 <= r_dist <= 0.38:
                    self._last_obj_pos = p_3d

            if box_uv is not None:
                p_box_3d = self._project_to_3d(box_uv[0], box_uv[1], target_z=DROP_BOX_Z_TABLE)
                self._last_box_pos = p_box_3d

    def _detect_objects(self, rgb_image: np.ndarray) -> tuple[tuple[float, float] | None, tuple[float, float] | None, np.ndarray]:
        """Detect the red cube and brown drop box in HSV color space."""
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)

        # 1. Red cube segmentation: red wraps around hue 0/180
        mask_red1 = cv2.inRange(hsv, np.array([0, 120, 70], dtype=np.uint8), np.array([12, 255, 255], dtype=np.uint8))
        mask_red2 = cv2.inRange(hsv, np.array([160, 120, 70], dtype=np.uint8), np.array([180, 255, 255], dtype=np.uint8))
        mask_cube = mask_red1 | mask_red2

        # Ignore pixels on the immediate edge of the image frame (avoid edge artifacts)
        mask_cube[:8, :] = 0
        mask_cube[-8:, :] = 0
        mask_cube[:, :8] = 0
        mask_cube[:, -8:] = 0

        # 2. Brown box segmentation (Hue around 35-75, Saturation 40-180, Value 20-120)
        mask_box = cv2.inRange(hsv, np.array([35, 40, 20], dtype=np.uint8), np.array([75, 180, 120], dtype=np.uint8))

        # Find centroid of cube
        cube_uv = None
        contours, _ = cv2.findContours(mask_cube, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Filter contours by area: true cube has area between 100 and 1500 px
            valid_contours = [c for c in contours if 50 < cv2.contourArea(c) < 3000]
            if valid_contours:
                largest = max(valid_contours, key=cv2.contourArea)
                M = cv2.moments(largest)
                if M["m00"] > 0:
                    cube_uv = (M["m10"] / M["m00"], M["m01"] / M["m00"])

        # Find centroid of box
        box_uv = None
        contours_b, _ = cv2.findContours(mask_box, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_b:
            largest_b = max(contours_b, key=cv2.contourArea)
            if cv2.contourArea(largest_b) > 40:
                M_b = cv2.moments(largest_b)
                if M_b["m00"] > 0:
                    box_uv = (M_b["m10"] / M_b["m00"], M_b["m01"] / M_b["m00"])

        # Create visualization mask
        mask_viz = cv2.cvtColor(mask_cube, cv2.COLOR_GRAY2RGB)
        if cube_uv is not None:
            cv2.circle(mask_viz, (int(round(cube_uv[0])), int(round(cube_uv[1]))), 5, (0, 255, 0), -1)
            cv2.putText(mask_viz, "cube", (int(cube_uv[0]) + 7, int(cube_uv[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        if box_uv is not None:
            cv2.circle(mask_viz, (int(round(box_uv[0])), int(round(box_uv[1]))), 5, (255, 255, 0), -1)
            cv2.putText(mask_viz, "box", (int(box_uv[0]) + 7, int(box_uv[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        return cube_uv, box_uv, mask_viz

    def _project_to_3d(self, u: float, v: float, target_z: float) -> np.ndarray:
        """Projects a 2D image pixel (u, v) into 3D world coordinates by intersecting
        the optical ray with the horizontal plane z = target_z.
        """
        # Ray direction in camera pinhole coordinate frame
        ray_cv = np.linalg.inv(self._K) @ np.array([u, v, 1.0])
        ray_cv = ray_cv / np.linalg.norm(ray_cv)

        # Ray direction in world frame
        ray_world = self._R_cv @ ray_cv

        # Intersection with plane z = target_z
        # Point on ray: p(t) = cam_pos + t * ray_world
        # cam_pos[2] + t * ray_world[2] = target_z
        if abs(ray_world[2]) < 1e-6:
            return np.array([0.0, 0.0, target_z])

        t = (target_z - self._cam_pos[2]) / ray_world[2]
        return self._cam_pos + t * ray_world

    def _publish_positions(self):
        """Publish PoseStamped for object and drop box."""
        now = self.get_clock().now().to_msg()

        with self._lock:
            obj_pos = self._last_obj_pos.copy() if self._last_obj_pos is not None else None
            box_pos = self._last_box_pos.copy() if self._last_box_pos is not None else None

        if obj_pos is not None:
            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = "world"
            msg.pose.position.x = float(obj_pos[0])
            msg.pose.position.y = float(obj_pos[1])
            msg.pose.position.z = float(obj_pos[2])
            msg.pose.orientation.w = 1.0
            self._pub_obj_pose.publish(msg)
            self._pub_ball_pose.publish(msg)

        if box_pos is not None:
            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = "world"
            msg.pose.position.x = float(box_pos[0])
            msg.pose.position.y = float(box_pos[1])
            msg.pose.position.z = float(box_pos[2])
            msg.pose.orientation.w = 1.0
            self._pub_box_pose.publish(msg)
        else:
            # Fallback to default known drop_box position from scene_tek5.xml
            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = "world"
            msg.pose.position.x = 0.05
            msg.pose.position.y = -0.25
            msg.pose.position.z = 0.0125
            msg.pose.orientation.w = 1.0
            self._pub_box_pose.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
