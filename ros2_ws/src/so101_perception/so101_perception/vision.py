"""Calibrated monocular cube pose from its known size and red silhouette.

No simulator state is read here. Height is estimated from apparent size rather
than forcing a lifted cube back onto the table plane. Cube yaw is modulo pi/2.
"""
from dataclasses import dataclass
import itertools
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

CAM_POS = np.array([.45, .45, .55])
_x = np.array([-.707, .707, 0.])
_x /= np.linalg.norm(_x)
_y = np.array([-.354, -.354, .866])
_y -= _x * (_x @ _y)
_y /= np.linalg.norm(_y)
R_CV = np.column_stack([_x, -_y, -np.cross(_x, _y)])
CUBE_SIZE = .05
_CORNERS = np.array(json.loads(Path(__file__).with_name("cube_hull.json").read_text())["vertices"])
_ANGLES = np.linspace(0, 2 * np.pi, 48, endpoint=False)
_DIRECTIONS = np.column_stack([np.cos(_ANGLES), np.sin(_ANGLES)])


@dataclass
class CubePose:
    position: np.ndarray
    yaw: float
    residual_px: float
    roll: float = 0.
    pitch: float = 0.


def red_mask(rgb, minimum_value=40):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (0, 140, minimum_value), (5, 255, 255)) | cv2.inRange(hsv, (160, 120, minimum_value), (180, 255, 255))
    return mask


def project(points, K, cam_pos=CAM_POS, rotation=R_CV):
    camera = (points - cam_pos) @ rotation
    pixels = camera @ K.T
    return pixels[:, :2] / pixels[:, 2:3]


def cube_corners(position, yaw, roll=0., pitch=0.):
    rotation = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    return _CORNERS @ rotation.T + position


def visible_above_box(points, cam_pos, center):
    """Visibility against the four calibrated, opaque walls of the open box."""
    if center is None:
        return points
    center = np.asarray(center)
    direction = points - cam_pos
    parallel = abs(direction) < 1e-12
    safe = np.where(parallel, 1., direction)
    hidden = np.zeros(len(points), dtype=bool)
    walls = [([sx * .05, 0, 0], [.002, .05, .025]) for sx in (-1, 1)]
    walls += [([0, sy * .05, 0], [.05, .002, .025]) for sy in (-1, 1)]
    for offset, half in walls:
        lower, upper = center + offset - half, center + offset + half
        first, last = (lower - cam_pos) / safe, (upper - cam_pos) / safe
        near, far = np.minimum(first, last), np.maximum(first, last)
        inside = (cam_pos >= lower) & (cam_pos <= upper)
        near = np.where(parallel, np.where(inside, -np.inf, np.inf), near)
        far = np.where(parallel, np.where(inside, np.inf, -np.inf), far)
        enter, leave = near.max(1), far.min(1)
        hidden |= (leave >= np.maximum(enter, 0)) & (enter < 1 - 1e-6) & (leave > 0)
    return points[~hidden]


def detect_cube(rgb, K, previous=None, cam_pos=CAM_POS, rotation=R_CV, table_height=None, occlusion_box=None, grasp_tcp=None, robot_depth=None):
    # Include shadowed faces in flight. While on the table the higher cutoff
    # prevents the floor reflection from joining the real object's silhouette.
    mask = red_mask(rgb, 40 if table_height is not None else 12)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in contours if 60 < cv2.contourArea(c) < 8000]
    if not valid:
        return None, mask
    contour = max(valid, key=cv2.contourArea)
    hull = cv2.convexHull(contour).reshape(-1, 2).astype(float)
    measured = (hull @ _DIRECTIONS.T).max(axis=0)
    centroid = hull.mean(axis=0)
    ray = rotation @ np.linalg.solve(K, [*centroid, 1])
    if ray[2] >= -1e-6:
        return None, mask
    guess = cam_pos + (.025 - cam_pos[2]) / ray[2] * ray
    def residual(parameters):
        roll, pitch = parameters[4:6] if len(parameters) == 6 else (0., 0.)
        shape = visible_above_box(cube_corners(parameters[:3], parameters[3], roll, pitch), cam_pos, occlusion_box)
        if len(shape) < 3:
            return np.full(len(measured), 1000.)
        pixels = project(shape, K, cam_pos, rotation)
        if robot_depth is not None:
            inside = (pixels[:, 0] >= 0) & (pixels[:, 0] < rgb.shape[1]) & (pixels[:, 1] >= 0) & (pixels[:, 1] < rgb.shape[0])
            indices = np.rint(pixels).astype(int)
            indices[:, 0] = np.clip(indices[:, 0], 0, rgb.shape[1] - 1)
            indices[:, 1] = np.clip(indices[:, 1], 0, rgb.shape[0] - 1)
            distances = ((shape - cam_pos) @ rotation)[:, 2]
            visible = ~inside | (distances <= robot_depth[indices[:, 1], indices[:, 0]] + .003)
            pixels = pixels[visible]
            if len(pixels) < 3:
                return np.full(len(measured), 1000.)
        model_hull = cv2.convexHull(pixels.astype(np.float32))
        height, width = rgb.shape[:2]
        _, clipped = cv2.intersectConvexConvex(model_hull, np.array([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]], dtype=np.float32))
        if clipped is None:
            return np.full(len(measured), 1000.)
        return (clipped.reshape(-1, 2) @ _DIRECTIONS.T).max(axis=0) - measured - .25
    starts = [np.r_[guess, yaw] for yaw in np.linspace(0, np.pi / 2, 4, endpoint=False)]
    if previous is not None:
        starts = [np.r_[previous.position, previous.yaw % (np.pi / 2)]]
    lower = [-.4, -.4, .022 if table_height is None else table_height - 1e-6, 0]
    upper = [.45, .45, .3 if table_height is None else table_height + 1e-6, np.pi / 2]
    if grasp_tcp is not None and table_height is None:
        # Resolve monocular depth/orientation ambiguity with the measured jaw
        # envelope. An image reprojection fit is still mandatory; no pose is
        # produced from FK alone, including when a cube is left on the table.
        lower[:3] = np.maximum(lower[:3], np.asarray(grasp_tcp) + [-.015, -.015, -.03]).tolist()
        upper[:3] = np.minimum(upper[:3], np.asarray(grasp_tcp) + [.015, .015, -.005]).tolist()
        if np.any(np.asarray(lower[:3]) >= upper[:3]):
            return None, mask
    if table_height is None:
        lower += [-.8, -.8]
        upper += [.8, .8]
        starts = [np.r_[start, previous.roll if previous else 0., previous.pitch if previous else 0.] for start in starts]
    fits = [least_squares(residual, np.clip(start, lower, upper), bounds=(lower, upper),
        diff_step=1e-3, max_nfev=60) for start in starts]
    fit = min(fits, key=lambda candidate: np.linalg.norm(candidate.fun))
    error = float(np.sqrt(np.mean(fit.fun ** 2)))
    if error > 1.3 or not np.all(np.isfinite(fit.x)):
        return None, mask
    # Reject abrupt pose jumps caused by partial occlusion; old observations
    # retain their source timestamp and will expire at the ROS boundary.
    if previous is not None and np.linalg.norm(fit.x[:3] - previous.position) > .05:
        return None, mask
    return CubePose(fit.x[:3], float(fit.x[3]), error, float(fit.x[4]) if len(fit.x) == 6 else 0., float(fit.x[5]) if len(fit.x) == 6 else 0.), mask


def detect_box(rgb, K, cam_pos=CAM_POS, rotation=R_CV, calibrated_center=None):
    """Image-gated fixed drop station, or corner-based position when uncalibrated.

    The scene's stationary box is a calibrated fixture. Its back rim is hidden
    by the arm, so fitting a full cuboid would bias its centre by centimetres.
    A configured centre is published only when brown walls are visible in its
    projected region. A moved fixture requires recalibration.
    """
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (6, 70, 25), (25, 240, 220))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in contours if cv2.contourArea(c) > 60]
    if not valid:
        return None
    if calibrated_center is not None:
        center = np.asarray(calibrated_center, dtype=float)
        if center.shape != (3,) or not np.all(np.isfinite(center)):
            return None
        corners = np.array(list(itertools.product((-.055, .055), (-.055, .055), (-.025, .025)))) + center
        roi = np.zeros(mask.shape, dtype=np.uint8)
        cv2.fillConvexPoly(roi, cv2.convexHull(project(corners, K, cam_pos, rotation).astype(np.int32)), 255)
        return center.copy() if np.count_nonzero(mask & roi) >= 60 else None
    hull = cv2.convexHull(max(valid, key=cv2.contourArea)).reshape(-1, 2)
    # The back rim is occluded by the arm. The nearest bottom corner of the
    # two outside walls remains visible. Its known table height gives depth;
    # the 52mm wall extent gives the box centre. No default world position.
    bottom = hull[hull[:, 1] == hull[:, 1].max()].mean(0)
    ray = rotation @ np.linalg.solve(K, [*bottom, 1])
    if ray[2] >= -1e-6:
        return None
    corner = cam_pos - cam_pos[2] / ray[2] * ray
    center = corner[:2] - np.sign(cam_pos[:2] - corner[:2]) * .052
    return np.r_[center, .025]
