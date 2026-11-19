"""Essai physique de transport : vision IA, FK/IK classiques uniquement."""

import argparse, json
from pathlib import Path
import numpy as np
from simulation_test import Simulation
from radial import orientation
from kinematics import SHOULDER

p = argparse.ArgumentParser()
p.add_argument("--force", type=float, default=0.5)
p.add_argument("--tilt", type=float, default=-20)
p.add_argument("--seed", type=int, default=0)
a = p.parse_args()
s = Simulation(a.seed)
s.arm_speed = 0.08
s.grip_force = -a.force
cube = min(
    (c for c in s.detect() if c["color"] == 1),
    key=lambda c: np.linalg.norm(c["xyz"][:2] - [0.22, 0.01]),
)
identity = int(np.argmin(np.linalg.norm(s.truth()[:5] - cube["xyz"], axis=1)))
trace = []


def log(name):
    record = {
        "phase": name,
        "cube": s.truth()[identity].tolist(),
        "tcp": s.kin.forward(s.q)[:3, 3].tolist(),
        "gripper": float(s.d.qpos[s.ids[5]]),
    }
    trace.append(record)
    print(record, flush=True)


def move(point, tilt):
    r, yaw = orientation(point, tilt)
    s.move(s.kin.solve_pose(point, s.q, yaw, 5, r))


def line(start, end, tilt):
    for point in np.linspace(start, end, 20)[1:]:
        move(point, tilt)


try:
    r, yaw = orientation(cube["xyz"], 0)
    pick = cube["xyz"] + r[:, 0] * 0.003 + [0, 0, 0.01]
    move(pick + [0, 0, 0.045], 0)
    line(pick + [0, 0, 0.045], pick, 0)
    s.close()
    log("PRISE")
    high = pick + [0, 0, 0.06]
    line(pick, high, 0)
    s.wait(2)
    log("LEVAGE")
    for tilt in np.linspace(0, a.tilt, 16)[1:]:
        move(high, tilt)
    s.wait(1)
    log("INCLINAISON")
    end = np.array([SHOULDER[0], -0.22, high[2]])
    start = high[:2] - SHOULDER
    radius = np.linalg.norm(start)
    angle = np.arctan2(start[1], start[0])
    delta = (-np.pi / 2 - angle + np.pi) % (2 * np.pi) - np.pi
    for f in np.linspace(0, 1, 80)[1:]:
        theta = angle + delta * f
        rr = radius + (0.22 - radius) * f
        move(
            np.r_[SHOULDER + rr * np.array([np.cos(theta), np.sin(theta)]), high[2]],
            a.tilt,
        )
    s.wait(1)
    log("TRANSPORT")
    for tilt in np.linspace(a.tilt, 0, 16)[1:]:
        move(end, tilt)
    log("REDRESSEMENT")
    drop = end.copy()
    drop[2] = 0.039
    line(end, drop, 0)
    log("DEPOT")
    s.force = None
    s.command[5] = 0.88
    s.wait(4)
    line(drop, drop + [0, 0, 0.05], 0)
    s.wait(2)
    log("FINAL")
except (ValueError, RuntimeError) as e:
    log("ECHEC")
    print(str(e), flush=True)
    trace.append({"error": str(e)})
finally:
    Path("/project/bonus/classic/results").mkdir(exist_ok=True)
    Path(
        f"/project/bonus/classic/results/transport_{a.seed}_{a.force}_{a.tilt}.json"
    ).write_text(json.dumps(trace, indent=2))
    s.robot.disconnect()
