"""Reglage classique mesure de la prise. La verite terrain ne sert qu'au verdict."""

import json, argparse
from pathlib import Path
import numpy as np
from simulation_test import Simulation
from radial import orientation

p = argparse.ArgumentParser()
p.add_argument("--tilt", type=float, required=True)
a = p.parse_args()
results = []
for height in (0.0, 0.01, 0.02):
    for force in (0.5, 1.2, 2.0):
        s = Simulation(0)
        s.arm_speed = 0.08
        s.grip_force = -force
        cube = min(
            [c for c in s.detect() if c["color"] == 1],
            key=lambda c: np.linalg.norm(c["xyz"][:2] - [0.22, 0.01]),
        )
        identity = int(np.argmin(np.linalg.norm(s.truth()[:5] - cube["xyz"], axis=1)))
        initial = s.truth()[identity].copy()
        r, yaw = orientation(cube["xyz"], a.tilt)
        target = cube["xyz"] + r[:, 0] * 0.003 + [0, 0, height]
        record = {"tilt": a.tilt, "height_offset": height, "closing_torque": force}
        try:
            q = s.kin.solve_pose(target + [0, 0, 0.045], s.q, yaw, 5, r)
            s.move(q)
            for z in np.linspace(target[2] + 0.045, target[2], 12)[1:]:
                q = s.kin.solve_pose([*target[:2], z], q, yaw, 5, r)
                s.move(q)
            s.close()
            for z in np.linspace(target[2], target[2] + 0.06, 16)[1:]:
                q = s.kin.solve_pose([*target[:2], z], q, yaw, 5, r)
                s.move(q)
            s.wait(2)
            record["lift"] = float(s.truth()[identity, 2] - initial[2])
            record["success"] = record["lift"] > 0.04
            record["gripper_angle"] = float(s.d.qpos[s.ids[5]])
        except (RuntimeError, ValueError) as error:
            record["error"] = str(error)
            record["success"] = False
        print(record, flush=True)
        results.append(record)
        s.robot.disconnect()
Path(f"/project/bonus/classic/results/calibration_{a.tilt}.json").write_text(
    json.dumps(results, indent=2)
)
