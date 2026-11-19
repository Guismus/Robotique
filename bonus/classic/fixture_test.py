"""Diagnostic des deux derniers depots sur un socle prepare, pas un essai complet."""

import json
from pathlib import Path
import numpy as np, mujoco, cv2
from simulation_test import Simulation
from calibrated_planning import TARGETS, plan_cube
from kinematics import HOME

s = Simulation(3)
s.arm_speed = 0.08
s.grip_force = -1.2
fixture = [4, 2, 1, 7]
for i, (_, p) in zip(fixture, TARGETS):
    j = s.m.joint(("pickup_object" if i == 0 else f"cube_{i}") + "_free").qposadr[0]
    s.d.qpos[j : j + 3] = p
    s.d.qpos[j + 3 : j + 7] = [1, 0, 0, 0]
s.d.qvel[:] = 0
mujoco.mj_forward(s.m, s.d)
s.wait(2)
trace = []
try:
    for slot in (4, 5):
        color, dest = TARGETS[slot]
        s.move(HOME, 0.88)
        s.wait(1)
        for c in sorted(
            s.detect(), key=lambda c: np.linalg.norm(c["xyz"][:2] - [0.22, 0.01])
        ):
            if c["color"] != color:
                continue
            try:
                steps, _ = plan_cube(s.kin, c["xyz"], c["yaw"], dest, s.q)
                break
            except ValueError:
                continue
        else:
            raise RuntimeError("Aucun trajet disponible")
        for phase, path in steps:
            if phase == "SAISIE":
                s.close()
            elif phase == "OUVERTURE":
                s.move(s.q, 0.88)
                s.wait(6)
            else:
                s.follow(path)
            record = {"slot": slot, "phase": phase, "positions": s.truth().tolist()}
            trace.append(record)
            print(
                slot + 1,
                phase,
                "premier vert",
                np.round(s.truth()[7], 4).tolist(),
                flush=True,
            )
        s.move(HOME, 0.88)
        s.wait(5)
except (ValueError, RuntimeError) as error:
    print(error, flush=True)
finally:
    Path("/project/bonus/classic/results/fixture_last_two.json").write_text(
        json.dumps(
            {"fixture_prepared": True, "trace": trace, "final": s.truth().tolist()},
            indent=2,
        )
    )
    renderer = mujoco.Renderer(s.m, height=480, width=640)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.06, -0.16, 0.08]
    cam.distance = 0.5
    cam.azimuth = 145
    cam.elevation = -25
    renderer.update_scene(s.d, camera=cam)
    cv2.imwrite(
        "/project/bonus/classic/results/fixture_last_two.png",
        cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR),
    )
    renderer.close()
    s.robot.disconnect()
