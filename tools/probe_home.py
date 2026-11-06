"""Diagnostic-only camera visibility of observation poses; same random seeds."""
from pathlib import Path
import sys
import json
import mujoco
import numpy as np
root = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(root / "sim"), str(root / "ros2_ws/src/so101_perception")]
from so101_sim import SO101Sim, ARM_JOINTS
from so101_perception.vision import detect_cube
poses = [[0,-.5,.8,.4,0], [0,-1,.4,.4,0], [0,-1.2,.5,0,0], [-1.7,-.5,.8,.4,0]]
result = []
for q in poses:
    rows = []
    for seed in range(20):
        sim = SO101Sim(scene_path=root / "sim/so101_sim/assets/so101/scene_tek5.xml", seed=seed)
        sim.connect()
        for name, angle in zip(ARM_JOINTS + ["gripper"], q + [.8814]):
            sim.data.qpos[sim.model.joint(name).qposadr[0]] = angle
        mujoco.mj_forward(sim.model, sim.data)
        pose, _ = detect_cube(sim.render_camera(), sim.get_camera_intrinsics(), table_height=.025)
        rows.append(float(np.linalg.norm(pose.position - sim.get_object_position())) if pose else None)
        sim.disconnect()
    result.append({"q": q, "detected": sum(x is not None for x in rows), "errors": rows})
print(json.dumps(result, indent=2))
