import numpy as np
from kinematics import ClassicalIK, HOME, SHOULDER
from radial import orientation

k = ClassicalIK()
for y in (-0.14, -0.16, -0.17, -0.19, -0.20, -0.21, -0.22, -0.24, -0.25, -0.27):
    row = []
    for z in (0.039, 0.095, 0.089, 0.12, 0.139, 0.17):
        point = np.array([SHOULDER[0], y, z])
        r, yaw = orientation(point, 0 if z < 0.13 else -20)
        try:
            k.solve_pose(point, HOME, yaw, 5, r)
            row.append(True)
        except ValueError:
            row.append(False)
    print(y, row, flush=True)
