import json
from pathlib import Path
import numpy as np, mujoco, cv2
from simulation_test import Simulation
from radial_planning import plan_cube, TARGETS

s = Simulation(0)
destination = TARGETS[0][1]
cube = min(
    [c for c in s.detect() if c["color"] == 1],
    key=lambda c: np.linalg.norm(c["xyz"][:2] - [0.22, 0.01]),
)
steps, yaw = plan_cube(s.kin, cube["xyz"], cube["yaw"], destination, s.q)
i = int(np.argmin(np.linalg.norm(s.truth()[:5] - cube["xyz"], axis=1)))
body = s.m.body("pickup_object" if i == 0 else f"cube_{i}").id
renderer = mujoco.Renderer(s.m, height=360, width=480)
cam = mujoco.MjvCamera()
cam.lookat[:] = [0.17, 0, 0.07]
cam.distance = 0.65
cam.azimuth = 150
cam.elevation = -30
out = Path("/project/bonus/classic/results")
video = cv2.VideoWriter(
    str(out / "diagnostic.avi"), cv2.VideoWriter_fourcc(*"MJPG"), 10, (480, 360)
)
trace = []
phase = ""
last = None


def observe(sim):
    global last
    contacts = []
    for c in sim.d.contact:
        a, b = int(c.geom1), int(c.geom2)
        if c.dist > 0:
            continue
        if sim.m.geom_bodyid[a] == body:
            other = b
        elif sim.m.geom_bodyid[b] == body:
            other = a
        else:
            continue
        contacts.append(sim.m.body(int(sim.m.geom_bodyid[other])).name)
    contacts = sorted(set(contacts))
    if contacts != last:
        print(
            phase,
            contacts,
            "z",
            round(float(sim.truth()[i, 2]), 3),
            "grip",
            round(float(sim.d.qpos[sim.ids[-1]]), 3),
            flush=True,
        )
        last = contacts
    renderer.update_scene(sim.d, camera=cam)
    frame = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
    cv2.putText(
        frame, phase, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
    )
    video.write(frame)
    trace.append(
        {
            "phase": phase,
            "t": float(sim.d.time),
            "cube": sim.truth()[i].tolist(),
            "q": sim.d.qpos[sim.ids].tolist(),
            "tcp": sim.kin.forward(sim.q)[:3, 3].tolist(),
            "contacts": contacts,
            "effort": float(sim.d.actuator_force[sim.act[-1]]),
        }
    )


s.observer = observe
s.last_observe = -1
for phase, path in steps:
    if phase == "SAISIE":
        s.close()
    elif phase == "OUVERTURE":
        s.force = None
        s.command[5] = 0.88
        s.wait(1)
    else:
        s.follow(path)
    renderer.update_scene(s.d, camera=cam)
    cv2.imwrite(
        str(out / (phase + ".png")), cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
    )
    if phase == "TRANSPORT":
        s.wait(1)
        break
video.release()
renderer.close()
(out / "diagnostic.json").write_text(json.dumps(trace, indent=2))
s.robot.disconnect()
