import numpy as np, mujoco
from simulation_test import Simulation
from radial_planning import plan_cube, TARGETS

s = Simulation(0)
cube = min(
    [c for c in s.detect() if c["color"] == 1],
    key=lambda c: np.linalg.norm(c["xyz"][:2] - [0.22, 0.01]),
)
idx = int(np.argmin(np.linalg.norm(s.truth()[:5] - cube["xyz"], axis=1)))
body = s.m.body("pickup_object" if idx == 0 else f"cube_{idx}").id
steps, _ = plan_cube(s.kin, cube["xyz"], cube["yaw"], TARGETS[0][1], s.q)
s.follow(steps[0][1])
s.follow(steps[1][1])
s.close()
center = s.d.body(body).xpos.copy()
r = s.d.body(body).xmat.reshape(3, 3)
print("center", center, "tcp", s.kin.forward(s.q)[:3, 3], "grip", s.d.qpos[s.ids[5]])
for i, c in enumerate(s.d.contact):
    a, b = s.m.geom_bodyid[c.geom1], s.m.geom_bodyid[c.geom2]
    if body not in (a, b):
        continue
    force = np.zeros(6)
    mujoco.mj_contactForce(s.m, s.d, i, force)
    if force[0] < 0.05:
        continue
    other = b if a == body else a
    print(
        s.m.body(int(other)).name,
        "local point mm",
        np.round((c.pos - center) @ r * 1000, 1),
        "normal",
        np.round(c.frame[:3] @ r, 2),
        "force",
        np.round(force[:3], 2),
        "penetration mm",
        round(-c.dist * 1000, 2),
    )
s.robot.disconnect()
