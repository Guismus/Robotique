"""Trajectoires classiques : saisir verticalement, incliner apres levage."""

import numpy as np
from radial import orientation
from kinematics import SHOULDER

TARGETS = [(1, np.array([SHOULDER[0], y, 0.025])) for y in (-0.24, -0.19, -0.14)]
TARGETS += [(2, np.array([SHOULDER[0], y, 0.075])) for y in (-0.215, -0.165)]
TARGETS += [(3, np.array([SHOULDER[0], -0.19, 0.125]))]


def plan_cube(kin, source, yaw, destination, q):
    steps = []

    def pose(point, tilt):
        nonlocal q
        r, angle = orientation(point, tilt)
        nxt = kin.solve_pose(point, q, angle, 5, r)
        if np.max(np.abs(nxt - q)) > 0.7:
            raise ValueError("Discontinuite de trajectoire")
        q = nxt
        return q.copy()

    def line(start, end, tilt):
        return [pose(p, tilt) for p in np.linspace(start, end, 20)[1:]]

    r, angle = orientation(source, 0)
    pick = source + r[:, 0] * 0.003 + [0, 0, 0.01]
    approach = pick + [0, 0, 0.045]
    try:
        q = kin.solve_pose(approach, q, angle, 5, r)
        steps += [("APPROCHE", [q.copy()]), ("DESCENTE", line(approach, pick, 0))]
    except ValueError:
        rr, aa = orientation(approach, -20)
        q = kin.solve_pose(approach, q, aa, 5, rr)
        near = pick + [0, 0, 0.01]
        steps += [
            ("APPROCHE", [q.copy()]),
            ("APPROCHE_BASSE", line(approach, near, -20)),
        ]
        steps.append(
            ("ALIGNEMENT_PRISE", [pose(near, t) for t in np.linspace(-20, 0, 16)[1:]])
        )
        steps.append(("DESCENTE", line(near, pick, 0)))
    steps.append(("SAISIE", []))
    high = pick + [0, 0, 0.025 if np.linalg.norm(pick[:2] - SHOULDER) > 0.23 else 0.06]
    steps.append(("LEVAGE", line(pick, high, 0)))
    steps.append(("INCLINAISON", [pose(high, t) for t in np.linspace(0, -20, 16)[1:]]))
    radius_now = np.linalg.norm(high[:2] - SHOULDER)
    if destination[2] > 0.04 and not 0.17 < radius_now < 0.22:
        inset = high.copy()
        inset[:2] = SHOULDER + (high[:2] - SHOULDER) * (0.20 / radius_now)
        steps.append(("RAPPROCHEMENT", line(high, inset, -20)))
        high = inset
    transit = high.copy()
    transit[2] = max(
        0.145 if destination[2] > 0.04 else 0.095, high[2], destination[2] + 0.03
    )
    steps.append(("MONTEE", line(high, transit, -20)))
    end = np.array([*destination[:2], transit[2]])
    if 0.04 < destination[2] < 0.1:
        end[1] += 0.02
    start = transit[:2] - SHOULDER
    radius = np.linalg.norm(start)
    angle = np.arctan2(start[1], start[0])
    other = end[:2] - SHOULDER
    rr = np.linalg.norm(other)
    aa = np.arctan2(other[1], other[0])
    delta = (aa - angle + np.pi) % (2 * np.pi) - np.pi
    arc = []
    for f in np.linspace(0, 1, 80)[1:]:
        theta = angle + delta * f
        rad = radius + (rr - radius) * f
        arc.append(
            pose(
                np.r_[
                    SHOULDER + rad * np.array([np.cos(theta), np.sin(theta)]),
                    transit[2],
                ],
                -20,
            )
        )
    steps.append(("TRANSPORT", arc))
    final_tilt = 0 if destination[2] < 0.1 else -20
    if final_tilt == 0:
        upright = end.copy()
        upright[2] = 0.085 if destination[2] < 0.04 else 0.095
        steps.append(("AJUSTEMENT_HAUTEUR", line(end, upright, -20)))
        end = upright
    steps.append(
        ("REDRESSEMENT", [pose(end, t) for t in np.linspace(-20, final_tilt, 16)[1:]])
    )
    release = destination.copy()
    release[2] += 0.014
    if final_tilt:
        r, _ = orientation(destination, final_tilt)
        release = destination - r @ np.array([0, 0, 0.013])
        release[2] += 0.011
    steps.append(("DEPOT", line(end, release, final_tilt)))
    steps.append(("OUVERTURE", []))
    if final_tilt:
        clear = release + [0, 0, 0.005]
        steps.append(("RETRAIT", line(release, clear, final_tilt)))
        if destination[2] > 0.04:
            forward = clear.copy()
            forward[1] += 0.025
            steps.append(("RECUL_APRES_DEPOT", line(clear, forward, final_tilt)))
            clear = forward
        steps.append(
            (
                "DEGAGEMENT_PINCE",
                [pose(clear, t) for t in np.linspace(final_tilt, -25, 12)[1:]],
            )
        )
        outside = clear.copy()
        outside[1] = -0.15
        steps.append(("DEGAGEMENT_BRAS", line(clear, outside, -25)))
    else:
        clear = release + [0, 0, 0.01]
        steps.append(("RETRAIT", line(release, clear, final_tilt)))
        if destination[2] > 0.04:
            forward = clear.copy()
            forward[1] += 0.025
            steps.append(("RECUL_APRES_DEPOT", line(clear, forward, final_tilt)))
            clear = forward
        steps.append(
            (
                "DEGAGEMENT_PINCE",
                [pose(clear, t) for t in np.linspace(final_tilt, -20, 16)[1:]],
            )
        )
        steps.append(("DEGAGEMENT_BRAS", line(clear, clear + [0, 0, 0.04], -20)))
    return steps, 0.0
