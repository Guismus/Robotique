"""Measure approach/carry feasibility for the four equivalent cube faces."""
from pathlib import Path
import sys
import json
import numpy as np
root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "ros2_ws/src/so101_brain"))
from so101_brain.kinematics import Kinematics, HOME
kin = Kinematics(root / "ros2_ws/src/so101_description/urdf/so101.urdf")
result = []
for point in ([.24,-.09,.025], [.18,-.05,.125], [.05,-.18,.13], [.05,-.25,.105], [.05,-.25,.10], [.05,-.25,.095], [.05,-.25,.085]):
    rows = []
    for yaw in .07 + np.arange(4) * np.pi / 2:
        sol = kin.solve(point, HOME, yaw, np.radians(45))
        rows.append({"yaw": float(yaw), "orientation_degrees": float(np.degrees(sol.orientation_error)) if sol else None,
            "q": sol.joints.tolist() if sol else None})
    result.append({"target": point, "solutions": rows})
print(json.dumps(result, indent=2))
