"""Affichage seulement : overlay camera, cubes dans RViz et reperes TF."""

import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration

# Classe du reseau -> (nom, couleur BGR pour l'image, couleur RGB pour RViz)
COLORS = {1: ("rouge", (0, 0, 255), (1.0, 0.1, 0.1)),
          2: ("vert", (0, 200, 0), (0.1, 0.8, 0.1)),
          3: ("bleu", (255, 80, 0), (0.1, 0.3, 1.0))}
SIZE = 0.05  # arete d'un cube, en m


def quaternion(r):
    """Matrice de rotation 3x3 -> quaternion (x, y, z, w)."""
    w = np.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return x, y, z, w


def yaw_matrix(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def to_pixel(point, k, t):
    """Point du monde -> pixel (u, v), ou None s'il est derriere la camera."""
    cam = np.linalg.inv(t) @ np.r_[point, 1.0]
    if cam[2] <= 0:
        return None
    p = k @ cam[:3]
    return p[:2] / p[2]


def top_square(center, yaw, k, t):
    """Les 4 coins du dessus du cube, en pixels."""
    r = yaw_matrix(yaw)
    h = SIZE / 2
    pts = [to_pixel(center + r @ [dx, dy, h], k, t) for dx, dy in ((-h, -h), (h, -h), (h, h), (-h, h))]
    return None if any(p is None for p in pts) else np.array(pts, np.int32)


def overlay(rgb, labels, cubes, targets, placements, k, t):
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    tint = img.copy()
    for color, (_, bgr, _) in COLORS.items():
        tint[labels == color] = bgr
    img = cv2.addWeighted(img, 0.6, tint, 0.4, 0)
    for c in cubes:
        name, bgr, _ = COLORS[c["color"]]
        square = top_square(np.asarray(c["xyz"]), c["yaw"], k, t)
        if square is not None:
            cv2.polylines(img, [square], True, bgr, 2)
            text = f'{c["xyz"][0]:.2f},{c["xyz"][1]:.2f}'
            (w, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.3, 1)
            x = int(square[:, 0].mean() - w / 2)
            y = int(square[:, 1].max() + 10)
            cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1, cv2.LINE_AA)
    for (color, p), ok in zip(targets, placements):
        square = top_square(np.asarray(p, float), 0.0, k, t)
        if square is not None:
            cv2.polylines(img, [square], True, (0, 255, 0) if ok else (160, 160, 160), 1)
    done = sum(bool(x) for x in placements)
    cv2.putText(img, f'pyramide : {done}/{len(targets)} cubes poses', (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def markers(stamp, cubes, targets, placements):
    out = MarkerArray()

    def add(ident, ns, xyz, yaw, rgb, alpha):
        m = Marker()
        m.header.stamp, m.header.frame_id = stamp, "base_link"
        m.ns, m.id, m.type, m.action = ns, ident, Marker.CUBE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
        q = quaternion(yaw_matrix(yaw))
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, q)
        m.scale.x = m.scale.y = m.scale.z = SIZE
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = alpha
        m.lifetime = Duration(seconds=0.5).to_msg()
        out.markers.append(m)

    for i, c in enumerate(cubes):
        add(i, "cubes_vus", c["xyz"], c["yaw"], COLORS[c["color"]][2], 1.0)
    for i, ((color, p), ok) in enumerate(zip(targets, placements)):
        add(i, "pyramide", p, 0.0, COLORS[color][2], 0.9 if ok else 0.15)
    return out


def frames(stamp, cubes, t):
    out = []

    def frame(child, rotation, xyz):
        f = TransformStamped()
        f.header.stamp, f.header.frame_id, f.child_frame_id = stamp, "base_link", child
        f.transform.translation.x, f.transform.translation.y, f.transform.translation.z = map(float, xyz)
        q = f.transform.rotation
        q.x, q.y, q.z, q.w = map(float, quaternion(rotation))
        out.append(f)

    frame("camera", t[:3, :3], t[:3, 3])
    count = {}
    for c in sorted(cubes, key=lambda c: (c["color"], c["xyz"][1], c["xyz"][0])):
        name = COLORS[c["color"]][0]
        count[name] = count.get(name, 0) + 1
        frame(f"cube_{name}_{count[name]}", yaw_matrix(c["yaw"]), c["xyz"])
    return out
