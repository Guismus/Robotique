"""Orientation inclinee constante dans le plan radial, machoires horizontales."""

import numpy as np
from kinematics import rotation, SHOULDER


def orientation(point, tilt=45):
    yaw = np.arctan2(point[1], point[0] - SHOULDER[0]) + np.pi / 2
    angle = np.radians(tilt)
    c, s = np.cos(angle), np.sin(angle)
    return rotation(yaw) @ np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), yaw


def probe():
    from kinematics import ClassicalIK, HOME

    k = ClassicalIK()
    targets = [
        np.array([SHOULDER[0], y, z])
        for y, z in [
            (-0.17, 0.025),
            (-0.22, 0.025),
            (-0.27, 0.025),
            (-0.195, 0.075),
            (-0.245, 0.075),
            (-0.22, 0.125),
            (-0.22, 0.175),
        ]
    ]
    for tilt in (-60, -45, -30, 30, 45, 60):
        result = []
        for point in targets:
            r, yaw = orientation(point, tilt)
            try:
                k.solve_pose(point, HOME, yaw, 4, r)
                result.append(True)
            except ValueError:
                result.append(False)
        print(tilt, result, flush=True)


if __name__ == "__main__":
    probe()
