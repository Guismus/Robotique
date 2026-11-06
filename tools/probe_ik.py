"""Diagnostic only: pose feasibility, without moving the simulated arm."""
import json
from pathlib import Path
import itertools
import ikpy.chain
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
chain = ikpy.chain.Chain.from_urdf_file(str(ROOT / "ros2_ws/src/so101_description/urdf/so101.urdf"),
    active_links_mask=[False, True, True, True, True, True, False])
bounds = np.array([link.bounds for link in chain.links[1:6]]).T


def solve(target, yaw):
    desired_x = np.array([np.cos(yaw), np.sin(yaw), 0])
    def residual(q):
        fk = chain.forward_kinematics([0, *q, 0])
        center = fk[:3, 3]
        return np.r_[(center - target) * 500, (fk[:3, 2] - [0, 0, -1]) * 3, (fk[:3, 0] - desired_x)]
    candidates = [least_squares(residual, q, bounds=bounds, max_nfev=150) for q in
        ([0, .2, -.5, 1.5, 0], [0, -.8, 1.2, 1.2, 0], [0, .8, -1.2, 1.2, -1.5])]
    best = min(candidates, key=lambda r: np.linalg.norm(r.fun))
    fk = chain.forward_kinematics([0, *best.x, 0])
    return {"q": best.x.tolist(), "position_error": float(np.linalg.norm(best.fun[:3]) / 500),
        "tilt_degrees": float(np.degrees(np.arccos(np.clip(-fk[2, 2], -1, 1)))),
        "yaw_error_degrees": float(np.degrees(np.arccos(np.clip(fk[:3, 0] @ desired_x, -1, 1))))}


if __name__ == "__main__":
    print(json.dumps([{"target": [x, y, z], "yaw": yaw, **solve([x, y, z], yaw)}
        for x, y, z, yaw in ((.24, -.09, .025, 0), (.24, -.09, .10, 0), (.24, -.09, .15, 0),
            (.05, -.25, .15, 0), (.3, 0, .025, .7), (.18, 0, .025, .7))], indent=2))
