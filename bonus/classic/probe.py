from kinematics import ClassicalIK, HOME
import numpy as np

k = ClassicalIK()
for x, y in ((0.16, -0.17), (0.18, -0.15), (0.20, -0.15), (0.22, -0.15)):
    rows = []
    for z in (0.025, 0.075, 0.125, 0.165):
        ok = []
        for yaw in (0, np.pi / 2, np.pi, -np.pi / 2):
            try:
                k.solve_pose([x, y, z], HOME, yaw, 8)
                ok.append(round(yaw, 2))
            except ValueError:
                pass
        rows.append((z, ok))
    print(x, y, rows, flush=True)
