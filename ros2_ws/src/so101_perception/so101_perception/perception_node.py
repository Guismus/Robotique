"""Repere le cube rouge et le bac marron, puis calcule leur position."""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

# Affichage de chaque objet : (texte, couleur BGR sur l'image, taille du pave RViz en m, couleur RGBA RViz)
STYLES = {
    'object': ('cube', (0, 255, 0), (.05, .05, .05), (1., 0., 0., 1.)),
    'drop_box': ('bac', (0, 165, 255), (.096, .096, .01), (1., .65, 0., .5)),
}

# Calibration ArUco : centres (x, y) en m des marqueurs colles sur la table (memes valeurs que scene_tek5.xml).
MARKERS = {0: (0.48, -0.27), 1: (0.48, 0.27), 2: (-0.28, 0.27), 3: (-0.28, -0.27)}
MARKER_SIZE = 0.10  # cote du carre noir, en m
MARKER_Z = 0.001    # surface de la plaque, en m


def marker_corners(ident):
    """Coins du marqueur dans l'ordre d'OpenCV : haut-gauche, haut-droite, bas-droite, bas-gauche."""
    cx, cy = MARKERS[ident]
    h = MARKER_SIZE / 2
    return [[cx - h, cy + h, MARKER_Z], [cx + h, cy + h, MARKER_Z],
            [cx + h, cy - h, MARKER_Z], [cx - h, cy - h, MARKER_Z]]


def quaternion(r):
    """Matrice de rotation 3x3 -> quaternion (x, y, z, w)."""
    w = np.sqrt(max(0., 1. + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0., 1. + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = np.copysign(np.sqrt(max(0., 1. - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = np.copysign(np.sqrt(max(0., 1. - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return x, y, z, w


def project_pixel(u, v, height, k, transform):
    """On suit le rayon du pixel jusqu'a la hauteur choisie, en metres."""
    ray = transform[:3, :3] @ np.linalg.solve(k, [u, v, 1.])
    origin = transform[:3, 3]
    if abs(ray[2]) < 1e-8:
        return None
    distance = (height - origin[2]) / ray[2]
    return origin + distance * ray if distance > 0 else None


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('so101_perception')
        self.k = self.transform = None
        # 'aruco' : la camera se calibre avec les marqueurs ; 'sim' : pose donnee par la simulation.
        self.mode = self.declare_parameter('calibration', 'aruco').value
        # 'pick' (phase 1) : bac de 5 cm de haut ; 'stack' (pile de 3 cubes) : socle plat de 2 mm.
        stack = self.declare_parameter('mode', 'pick').value == 'stack'
        self.box_height = 0.002 if stack else 0.025
        # Reperes TF publies pour RViz, accroches a base_link : 'camera' (pose trouvee par la calibration,
        # fixe) puis 'cube' et 'bac' (ou 'socle' pour la pile), mis a jour a chaque image.
        self.frames = {'object': 'cube', 'drop_box': 'socle' if stack else 'bac'}
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.camera_tf_sent = False
        self.cubes_pub = self.create_publisher(PoseArray, '/objects_positions', 10)
        # Position de la camera trouvee par la calibration : le cerveau en a besoin pour savoir
        # ou doit apparaitre le sommet de la pile vu d'en haut (effet de perspective).
        self.camera_pub = self.create_publisher(PointStamped, '/perception/camera_position', 1)
        self.sim_transform = None  # pose donnee par la sim, gardee pour mesurer l'erreur de l'ArUco
        self.rvec = self.tvec = None
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)
        self.bridge = CvBridge()
        self.position_pubs = {name: self.create_publisher(PoseStamped, '/' + name + '_position', 10)
                           for name in ['object', 'drop_box']}
        self.mask_pub = self.create_publisher(Image, '/perception/mask', 1)
        self.overlay_pub = self.create_publisher(Image, '/perception/overlay', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/markers', 10)
        self.create_subscription(CameraInfo, '/external_cam/camera_info', self.camera_info, 1)
        self.create_subscription(Float64MultiArray, '/external_cam/pose', self.camera_pose, 1)
        # Meme QoS capteur que le driver (BEST_EFFORT) : sinon les deux ne se connectent pas.
        self.create_subscription(Image, '/external_cam/image_raw', self.image, qos_profile_sensor_data)

    def camera_info(self, msg):
        self.k = np.array(msg.k).reshape(3, 3)

    def camera_pose(self, msg):
        if len(msg.data) == 16:
            self.sim_transform = np.array(msg.data).reshape(4, 4)
            if self.mode == 'sim':
                self.transform = self.sim_transform

    def image(self, msg):
        if self.k is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.transform is None:
            if self.mode == 'aruco':
                self.calibrate(frame)
            return
        if not self.camera_tf_sent:
            self.static_tf.sendTransform(self.frame_tf(msg.header.stamp, 'camera', self.transform))
            self.camera_tf_sent = True
        frames = []
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        overlay = frame.copy()
        markers = MarkerArray()
        red = cv2.inRange(hsv, (0, 140, 60), (3, 255, 255))
        red |= cv2.inRange(hsv, (155, 90, 50), (179, 255, 255))
        brown = cv2.inRange(hsv, (5, 65, 30), (25, 255, 180))
        mask_msg = self.bridge.cv2_to_imgmsg(red, 'mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)
        self.publish_cubes(red, msg.header, overlay, markers)
        for ident, (name, mask, height) in enumerate([('object', red, 0.025), ('drop_box', brown, self.box_height)]):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            m = cv2.moments(contour)
            if m['m00'] < 20:
                continue
            point = project_pixel(m['m10']/m['m00'], m['m01']/m['m00'], height, self.k, self.transform)
            if point is None or not np.isfinite(point).all():
                continue
            result = PoseStamped()
            result.header.stamp = msg.header.stamp
            result.header.frame_id = 'base_link'
            result.pose.position.x, result.pose.position.y, result.pose.position.z = map(float, point)
            result.pose.orientation.w = 1.
            self.position_pubs[name].publish(result)
            self.draw(overlay, name, contour, point)
            markers.markers.append(self.marker(result, name, ident))
            pose = np.eye(4)
            pose[:3, 3] = point
            frames.append(self.frame_tf(msg.header.stamp, self.frames[name], pose))
        if self.rvec is not None:
            for i in MARKERS:
                pts, _ = cv2.projectPoints(np.array(marker_corners(i), np.float32), self.rvec, self.tvec, self.k, None)
                cv2.polylines(overlay, [pts.reshape(-1, 2).astype(np.int32)], True, (255, 0, 0), 1)
        overlay_msg =self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
        overlay_msg.header = msg.header
        self.overlay_pub.publish(overlay_msg)
        if markers.markers:
            self.marker_pub.publish(markers)
        if frames:
            self.tf.sendTransform(frames)

    def frame_tf(self, stamp, child, pose):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'base_link', child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, pose[:3, 3])
        q = t.transform.rotation
        q.x, q.y, q.z, q.w = map(float, quaternion(pose[:3, :3]))
        return t

    def publish_cubes(self, red, header, overlay, markers):
        """Pile : une position par tache rouge ; la plus grosse part aussi sur /object_position."""
        cubes = PoseArray()
        cubes.header.stamp, cubes.header.frame_id = header.stamp, 'base_link'
        contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            m = cv2.moments(contour)
            if m['m00'] < 80:
                continue
            point = project_pixel(m['m10']/m['m00'], m['m01']/m['m00'], 0.025, self.k, self.transform)
            if point is None or not np.isfinite(point).all():
                continue
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, point)
            pose.orientation.w = 1.
            cubes.poses.append(pose)
            self.draw(overlay, 'object', contour, point)
            stamped = PoseStamped(header=cubes.header, pose=pose)
            markers.markers.append(self.marker(stamped, 'object', 10 + len(cubes.poses)))
        self.cubes_pub.publish(cubes)
        camera = PointStamped(header=cubes.header)
        camera.point.x, camera.point.y, camera.point.z = map(float, self.transform[:3, 3])
        self.camera_pub.publish(camera)

    def calibrate(self, frame):
        """Trouve ou est la camera grace aux marqueurs ArUco dont on connait la position sur la table."""
        corners, ids, _ = self.aruco.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if ids is None:
            return
        world, pixels = [], []
        for c, i in zip(corners, ids.ravel()):
            if int(i) in MARKERS:
                world += marker_corners(int(i))
                pixels += c[0].tolist()
        if len(world) < 8:  # au moins 2 marqueurs pour une pose fiable
            return
        world, pixels = np.array(world, np.float32), np.array(pixels, np.float32)
        ok, rvec, tvec = cv2.solvePnP(world, pixels, self.k, None)
        if not ok:
            return
        # Erreur de reprojection : ecart moyen, en pixels, entre les coins vus et les coins recalcules.
        reproj = np.linalg.norm(cv2.projectPoints(world, rvec, tvec, self.k, None)[0].reshape(-1, 2) - pixels, axis=1).mean()
        if reproj > 2.:
            self.get_logger().warn(f'Calibration ArUco refusee : reprojection {reproj:.1f} px')
            return
        cam_from_world = np.eye(4)
        cam_from_world[:3, :3] = cv2.Rodrigues(rvec)[0]
        cam_from_world[:3, 3] = tvec.ravel()
        self.transform = np.linalg.inv(cam_from_world)
        self.rvec, self.tvec = rvec, tvec
        text = (f'Calibration ArUco : {len(world) // 4} marqueurs, camera en {np.round(self.transform[:3, 3], 3)}, '
                f'reprojection {reproj:.2f} px')
        if self.sim_transform is not None:
            text += f', ecart avec la sim {np.linalg.norm(self.transform[:3, 3] - self.sim_transform[:3, 3]) * 1000:.1f} mm'
        self.get_logger().info(text)

    def draw(self, overlay, name, contour, point):
        label, color, _, _ = STYLES[name]
        cv2.drawContours(overlay, [contour], -1, color, 2)
        x, y, _, _ = cv2.boundingRect(contour)
        cv2.putText(overlay, f'{label} ({point[0]:.2f}, {point[1]:.2f})', (x, max(y - 5, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)

    def marker(self, pose_msg, name, ident):
        _, _, size, color = STYLES[name]
        m = Marker()
        m.header = pose_msg.header
        m.ns, m.id = 'perception', ident
        m.type, m.action = Marker.CUBE, Marker.ADD
        m.pose = pose_msg.pose
        m.scale.x, m.scale.y, m.scale.z = size
        m.color.r, m.color.g, m.color.b, m.color.a = color
        m.lifetime = Duration(seconds=0.5).to_msg()
        return m


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
