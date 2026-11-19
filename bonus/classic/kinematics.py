"""FK/IK classiques : aucun reseau neuronal ne choisit les angles."""

import sys
import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, "/project/ros2_ws/src/so101_brain")
from so101_brain.kinematics import Kinematics

HOME = np.array([1.7, -1.2, 0.4, 0.4, 0.0])
SHOULDER = np.array([0.0388353, 0.0])


def rotation(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, s, 0], [s, -c, 0], [0, 0, -1.0]])


class ClassicalIK(Kinematics):
    def __init__(self):
        super().__init__("/project/sim/so101_sim/assets/so101/so101_new_calib.urdf")
        self.origins[-1][:3, 3] = [0.0171, -0.000218121, -0.0901274]

    def solve_pose(self, point, initial, yaw, tolerance=8, desired=None):
        desired = rotation(yaw) if desired is None else desired
        point = np.asarray(point)

        def residual(q):
            pose = self.forward(q)
            return np.r_[
                500 * (pose[:3, 3] - point), 2 * (pose[:3, :3] - desired).ravel()
            ]

        best = None
        for start in (
            initial,
            [initial[0], 0.4, -0.3, 1.4, initial[4]],
            [initial[0], -0.5, 0.8, 1.0, initial[4]],
        ):
            fit = least_squares(
                residual, np.clip(start, *self.bounds), bounds=self.bounds, max_nfev=100
            )
            pose = self.forward(fit.x)
            distance = np.linalg.norm(pose[:3, 3] - point)
            angle = np.degrees(
                np.arccos(np.clip((np.trace(desired.T @ pose[:3, :3]) - 1) / 2, -1, 1))
            )
            if distance < 0.002 and angle < tolerance:
                return fit.x
        raise ValueError(f"Pose inaccessible : {np.round(point,3)}, yaw={yaw:.2f}")

    def path(self, start, end, q, yaw, tolerance=25):
        points = np.linspace(
            start, end, max(2, int(np.linalg.norm(np.asarray(end) - start) / 0.005) + 1)
        )[1:]
        result = []
        for p in points:
            nxt = self.solve_pose(p, q, yaw, tolerance)
            if np.max(np.abs(nxt - q)) > 0.6:
                raise ValueError("Discontinuite de trajectoire")
            result.append(nxt)
            q = nxt
        return result

    def arc(self, start, end, q, yaw, end_yaw=None):
        a, b = np.asarray(start), np.asarray(end)
        ra = np.linalg.norm(a[:2] - SHOULDER)
        rb = np.linalg.norm(b[:2] - SHOULDER)
        aa = np.arctan2(*(a[:2] - SHOULDER)[::-1])
        ab = np.arctan2(*(b[:2] - SHOULDER)[::-1])
        delta = (ab - aa + np.pi) % (2 * np.pi) - np.pi
        result = []
        for f in np.linspace(0, 1, 60)[1:]:
            angle = aa + delta * f
            r = ra + (rb - ra) * f
            p = np.r_[
                SHOULDER + r * np.array([np.cos(angle), np.sin(angle)]),
                a[2] + (b[2] - a[2]) * f,
            ]
            angle_yaw = (
                yaw
                if end_yaw is None
                else yaw + ((end_yaw - yaw + np.pi) % (2 * np.pi) - np.pi) * f
            )
            q = self.solve_pose(p, q, angle_yaw, 55)
            result.append(q)
        return result
