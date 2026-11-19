"""Repere le cube rouge et le bac marron, puis calcule leur position."""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

# Affichage de chaque objet : (texte, couleur BGR sur l'image, taille du pave RViz en m, couleur RGBA RViz)
STYLES = {
    'object': ('cube', (0, 255, 0), (.05, .05, .05), (1., 0., 0., 1.)),          # cube rouge plein de 5 cm
    'drop_box': ('bac', (0, 165, 255), (.096, .096, .01), (1., .65, 0., .5)),    # plaque orange transparente
}


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
        self.bridge = CvBridge()
        self.position_pubs = {name: self.create_publisher(PoseStamped, '/' + name + '_position', 10)
                           for name in ['object', 'drop_box']}
        self.mask_pub = self.create_publisher(Image, '/perception/mask', 1)
        self.overlay_pub = self.create_publisher(Image, '/perception/overlay', 1)  # image camera + detections
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/markers', 10)  # cube et bac en 3D
        self.create_subscription(CameraInfo, '/external_cam/camera_info', self.camera_info, 1)
        self.create_subscription(Float64MultiArray, '/external_cam/pose', self.camera_pose, 1)
        self.create_subscription(Image, '/external_cam/image_raw', self.image, 1)

    def camera_info(self, msg):
        self.k = np.array(msg.k).reshape(3, 3)

    def camera_pose(self, msg):
        if len(msg.data) == 16:
            self.transform = np.array(msg.data).reshape(4, 4)

    def image(self, msg):
        if self.k is None or self.transform is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        overlay = frame.copy()  # on dessine sur une copie, l'image d'origine reste intacte
        markers = MarkerArray()
        red = cv2.inRange(hsv, (0, 140, 60), (3, 255, 255))
        red |= cv2.inRange(hsv, (155, 90, 50), (179, 255, 255))
        brown = cv2.inRange(hsv, (5, 65, 30), (25, 255, 180))
        mask_msg = self.bridge.cv2_to_imgmsg(red, 'mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)
        for ident, (name, mask, height) in enumerate([('object', red, 0.025), ('drop_box', brown, 0.025)]):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue  # Aucun faux point (0, 0, 0) si le cube disparait.
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
        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
        overlay_msg.header = msg.header  # meme horodatage que l'image de la camera
        self.overlay_pub.publish(overlay_msg)
        if markers.markers:
            self.marker_pub.publish(markers)

    def draw(self, overlay, name, contour, point):
        """Entoure l'objet detecte sur l'image et ecrit sa position, ex. 'cube (0.25, 0.10)'."""
        label, color, _, _ = STYLES[name]
        cv2.drawContours(overlay, [contour], -1, color, 2)
        x, y, _, _ = cv2.boundingRect(contour)  # coin haut-gauche de la zone detectee
        cv2.putText(overlay, f'{label} ({point[0]:.2f}, {point[1]:.2f})', (x, max(y - 5, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)

    def marker(self, pose_msg, name, ident):
        """Un pave 3D dans RViz, a la position calculee depuis la camera."""
        _, _, size, color = STYLES[name]
        m = Marker()
        m.header = pose_msg.header  # repere base_link + meme horodatage
        m.ns, m.id = 'perception', ident  # 0 = cube, 1 = bac
        m.type, m.action = Marker.CUBE, Marker.ADD
        m.pose = pose_msg.pose
        m.scale.x, m.scale.y, m.scale.z = size
        m.color.r, m.color.g, m.color.b, m.color.a = color  # a = opacite (1 = plein)
        m.lifetime = Duration(seconds=0.5).to_msg()  # disparait s'il n'est plus vu
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
