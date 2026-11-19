"""Pyramide sur un axe radial : inclinaison de pince constante, cubes droits."""

import numpy as np
from radial import orientation
from kinematics import rotation, SHOULDER

TILT = -30
TARGETS = [(1, np.array([SHOULDER[0], y, 0.025])) for y in (-0.27, -0.22, -0.17)]
TARGETS += [(2, np.array([SHOULDER[0], y, 0.075])) for y in (-0.245, -0.195)]
TARGETS += [(3, np.array([SHOULDER[0], -0.22, 0.125]))]


def solve(kin, point, q):
    r, yaw = orientation(point, TILT)
    return kin.solve_pose(point, q, yaw, 5, r)


def line(kin, start, end, q):
    result = []
    for p in np.linspace(
        start, end, max(2, int(np.linalg.norm(np.asarray(end) - start) / 0.004) + 1)
    )[1:]:
        q = solve(kin, p, q)
        result.append(q)
    return result


def plan_cube(kin, source, yaw, destination, q):
    r, _ = orientation(source, TILT)
    pickup = source + r[:, 0] * 0.003 + [0, 0, 0.025]
    r, _ = orientation(destination, TILT)
    drop = destination + r[:, 0] * 0.003 + [0, 0, 0.025]
    approach = solve(kin, pickup + [0, 0, 0.05], q)
    lower = line(kin, pickup + [0, 0, 0.05], pickup, approach)
    high = pickup.copy()
    high[2] = max(0.14, destination[2] + 0.05)
    lift = line(kin, pickup, high, lower[-1])
    above = drop.copy()
    above[2] = high[2]
    a, b = high[:2] - SHOULDER, above[:2] - SHOULDER
    ra, rb = np.linalg.norm(a), np.linalg.norm(b)
    aa, ab = np.arctan2(a[1], a[0]), np.arctan2(b[1], b[0])
    delta = (ab - aa + np.pi) % (2 * np.pi) - np.pi
    arc = []
    q = lift[-1]
    for f in np.linspace(0, 1, 80)[1:]:
        angle = aa + delta * f
        radius = ra + (rb - ra) * f
        p = np.r_[SHOULDER + radius * np.array([np.cos(angle), np.sin(angle)]), high[2]]
        q = solve(kin, p, q)
        arc.append(q)
    release = drop + [0, 0, 0.004]
    down = line(kin, above, release, q)
    clear = line(kin, release, release + [0, 0, 0.025], down[-1])
    return [
        ("APPROCHE", [approach]),
        ("DESCENTE", lower),
        ("SAISIE", []),
        ("LEVAGE", lift),
        ("TRANSPORT", arc),
        ("DEPOT", down),
        ("OUVERTURE", []),
        ("RETRAIT", clear),
    ], 0.0
