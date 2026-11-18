#!/usr/bin/env python3
"""Camera-only object/box poses, with source timestamps and bounded validity."""
import threading
import cv2
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, JointState
from visualization_msgs.msg import Marker, MarkerArray
from .vision import CAM_POS, R_CV, detect_cube, detect_box, project, cube_corners
from .occlusion import gripper_depth


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("so101_perception")
        self._lock = threading.RLock()
        self._cv_bridge = CvBridge()
        self._K = None
        self._cube = None
        self._last_obj_pos = self._last_box_pos = None
        self._obj_stamp = self._box_stamp = None
        self._tracking_cube = None
        self._tracking_stamp = -np.inf
        self._last_image_stamp = -np.inf
        self._table_prior = True
        self._was_open = False
        self._gripper_is_open = False
        self._box_support_count = 0
        self._last_supported_pose = None
        self._ee_position = None
        self._ee_rotation = np.eye(3)
        self._gripper_angle = 0.
        self._ee_stamp = -np.inf
        self._table_support_count = 0
        self._last_table_pose = None
        self._cam_pos = np.array(self.declare_parameter("camera_position", CAM_POS.tolist()).value)
        self._R_cv = np.array(self.declare_parameter("camera_rotation_cv", R_CV.ravel().tolist()).value).reshape(3, 3)
        # Static fixture calibration from scene_tek5.xml, gated by actual images.
        self._calibrated_box = np.array(self.declare_parameter("drop_box_center", [.05, -.25, .025]).value)
        self.create_subscription(CameraInfo, "/external_cam/camera_info", self._cb_camera_info, 10)
        self.create_subscription(Image, "/external_cam/image_raw", self._cb_image, 2)
        self.create_subscription(JointState, "/joint_states", self._cb_joints, 10)
        self.create_subscription(PoseStamped, "/end_effector_pose", self._cb_end_effector, 10)
        self._pub_obj_pose = self.create_publisher(PoseStamped, "/object_position", 10)
        self._pub_ball_pose = self.create_publisher(PoseStamped, "/ball_position", 10)
        self._pub_box_pose = self.create_publisher(PoseStamped, "/drop_box_position", 10)
        self._pub_mask = self.create_publisher(Image, "/perception/mask", 2)
        self._pub_overlay = self.create_publisher(Image, "/perception/overlay", 2)
        self._pub_markers = self.create_publisher(MarkerArray, "/perception/markers", 10)
        self.create_timer(.1, self._publish_positions)

    def _cb_camera_info(self, msg):
        matrix = np.array(msg.k).reshape(3, 3)
        if np.all(np.isfinite(matrix)) and matrix[0, 0] > 0 and matrix[1, 1] > 0:
            with self._lock:
                self._K = matrix

    def _cb_joints(self, msg):
        values = dict(zip(msg.name, msg.position))
        angle = values.get("gripper")
        if angle is not None and np.isfinite(angle):
            self._gripper_angle = float(angle)
            self._gripper_is_open = angle > .8
            self._was_open |= angle > .8
            if self._was_open and angle < .78:
                # The initial object rests on the table. Once grasping starts,
                # estimate height from known size, including after release.
                self._table_prior = False

    def _cb_end_effector(self, msg):
        if msg.header.frame_id == "world":
            position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            if np.all(np.isfinite(position)):
                self._ee_position = position
                quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
                if np.linalg.norm(quaternion) > .99:
                    self._ee_rotation = Rotation.from_quat(quaternion).as_matrix()
                self._ee_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _cb_image(self, msg):
        if self._K is None:
            return
        try:
            image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as error:
            self.get_logger().warn(f"Image conversion failed: {error}")
            return
        with self._lock:
            # Discard old images instead of turning a queued frame into a new target.
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            now = self.get_clock().now().nanoseconds * 1e-9
            if stamp <= self._last_image_stamp or stamp <= 0 or not -.05 <= now - stamp <= .6:
                return
            self._last_image_stamp = stamp
            prior = self._tracking_cube if now - self._tracking_stamp < .6 else None
            tcp = self._ee_position if not self._gripper_is_open and self._ee_position is not None and self._ee_position[2] > .075 and now - self._ee_stamp < .3 else None
            robot_depth = gripper_depth(image.shape, self._K, self._cam_pos, self._R_cv, self._ee_position, self._ee_rotation, self._gripper_angle) if self._ee_position is not None and now - self._ee_stamp < .3 else None
            pose, mask = detect_cube(image, self._K, prior, self._cam_pos, self._R_cv, .025 if self._table_prior else None, self._calibrated_box, tcp, robot_depth)
            near_box = prior is not None and np.linalg.norm(prior.position[:2] - self._calibrated_box[:2]) < .1
            if self._gripper_is_open and not self._table_prior and not near_box:
                on_table, _ = detect_cube(image, self._K, cam_pos=self._cam_pos, rotation=self._R_cv,
                    table_height=.025, occlusion_box=self._calibrated_box)
                stable_table = on_table is not None and self._last_table_pose is not None and np.linalg.norm(on_table.position - self._last_table_pose) < .004
                self._table_support_count = self._table_support_count + 1 if stable_table else (1 if on_table else 0)
                self._last_table_pose = on_table.position.copy() if on_table else None
                if self._table_support_count >= 3:
                    self._table_prior, pose = True, on_table
            if self._gripper_is_open and near_box and not self._table_prior:
                # A support-plane hypothesis must fit the visible red cap; an
                # open gripper alone never confirms delivery. Wall occlusion
                # is modelled explicitly, including for the reprojection test.
                supported, _ = detect_cube(image, self._K, cam_pos=self._cam_pos, rotation=self._R_cv,
                    table_height=self._calibrated_box[2] + .004, occlusion_box=self._calibrated_box)
                half_width = .025 * (abs(np.cos(supported.yaw)) + abs(np.sin(supported.yaw))) if supported else np.inf
                inside = supported is not None and np.all(np.abs(supported.position[:2] - self._calibrated_box[:2]) + half_width <= .048)
                stable = inside and self._last_supported_pose is not None and np.linalg.norm(supported.position - self._last_supported_pose) < .004
                self._box_support_count = self._box_support_count + 1 if stable else (1 if inside else 0)
                self._last_supported_pose = supported.position.copy() if inside else None
                if self._box_support_count >= 3:
                    pose = supported
            if pose:
                self._tracking_cube, self._tracking_stamp = pose, stamp
            self._last_obj_pos = pose.position.copy() if pose else None
            self._cube = pose
            self._obj_stamp = msg.header.stamp if pose else None
            self._last_box_pos = detect_box(image, self._K, self._cam_pos, self._R_cv, self._calibrated_box)
            self._box_stamp = msg.header.stamp if self._last_box_pos is not None else None
        mask_msg = self._cv_bridge.cv2_to_imgmsg(cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB), encoding="rgb8")
        mask_msg.header = msg.header
        self._pub_mask.publish(mask_msg)
        overlay = self._draw_overlay(image, pose, self._last_box_pos)
        overlay_msg = self._cv_bridge.cv2_to_imgmsg(overlay, encoding="rgb8")
        overlay_msg.header = msg.header  # même horodatage que l'image d'origine
        self._pub_overlay.publish(overlay_msg)

    def _draw_overlay(self, image, pose, box_pos):
        """Dessine sur l'image caméra ce que la perception a détecté."""
        out = image.copy()  # on ne touche pas l'image d'origine
        font = cv2.FONT_HERSHEY_SIMPLEX
        if pose is not None:  # le cube a été trouvé
            coins_3d = cube_corners(pose.position, pose.yaw, pose.roll, pose.pitch)
            pixels = project(coins_3d, self._K, self._cam_pos, self._R_cv)
            contour = cv2.convexHull(pixels.astype(np.int32))
            cv2.polylines(out, [contour], True, (0, 255, 0), 2)  # contour vert
            x, y, z = pose.position
            u, v = contour[:, 0, :].min(axis=0)
            cv2.putText(out, f"cube ({x:.2f}, {y:.2f}, {z:.2f})", (int(u), int(v) - 6),
                        font, .45, (0, 255, 0), 1, cv2.LINE_AA)
        if box_pos is not None:  # le bac a été trouvé
            h = .048  # demi-largeur intérieure du bac (4,8 cm)
            coins_3d = np.array([[box_pos[0] + dx, box_pos[1] + dy, box_pos[2]]
                                 for dx, dy in ((-h, -h), (h, -h), (h, h), (-h, h))])
            pixels = project(coins_3d, self._K, self._cam_pos, self._R_cv).astype(np.int32)
            cv2.polylines(out, [pixels], True, (255, 165, 0), 2)  # carré orange
            u, v = pixels.min(axis=0)
            cv2.putText(out, "bac", (int(u), int(v) - 6), font, .45, (255, 165, 0), 1, cv2.LINE_AA)
        return out

    def _pose(self, position, stamp, yaw=0., roll=0., pitch=0.):
        if position is None or stamp is None:
            return None
        age = self.get_clock().now().nanoseconds * 1e-9 - (stamp.sec + stamp.nanosec * 1e-9)
        if not -.05 <= age <= .6:
            return None
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = stamp, "world"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
        quaternion = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, quaternion)
        return msg

    def _publish_positions(self):
        with self._lock:
            obj = self._pose(self._last_obj_pos, self._obj_stamp, self._cube.yaw if self._cube else 0., self._cube.roll if self._cube else 0., self._cube.pitch if self._cube else 0.)
            box = self._pose(self._last_box_pos, self._box_stamp)
        if obj:
            self._pub_obj_pose.publish(obj)
            self._pub_ball_pose.publish(obj)
        if box:
            self._pub_box_pose.publish(box)
        markers = MarkerArray()
        if obj:  # cube détecté → cube rouge 5×5×5 cm
            markers.markers.append(self._marker(obj, 0, (.05, .05, .05), (1., 0., 0., 1.)))
        if box:  # bac détecté → plaque orange 9,6×9,6×1 cm, semi-transparente
            markers.markers.append(self._marker(box, 1, (.096, .096, .01), (1., .65, 0., .5)))
        if markers.markers:
            self._pub_markers.publish(markers)

    def _marker(self, pose_msg, ident, size, color):
        """Un pavé 3D dans RViz, à la position détectée par la caméra."""
        m = Marker()
        m.header = pose_msg.header  # repère "world" + même horodatage
        m.ns, m.id = "perception", ident  # 0 = cube, 1 = bac
        m.type, m.action = Marker.CUBE, Marker.ADD
        m.pose = pose_msg.pose  # position + rotation du cube
        m.scale.x, m.scale.y, m.scale.z = size  # taille en mètres
        m.color.r, m.color.g, m.color.b, m.color.a = color  # a = opacité (1 = plein)
        m.lifetime = Duration(seconds=0.5).to_msg()  # disparaît s'il n'est plus vu
        return m


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
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
