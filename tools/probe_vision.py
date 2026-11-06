"""Diagnostic only: image-based estimates versus seeded simulator truth."""
import json
from pathlib import Path
import sys
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "sim"), str(ROOT / "ros2_ws/src/so101_perception")]
from so101_sim import SO101Sim
from so101_perception.vision import detect_cube, project, cube_corners, red_mask

results = []
for seed in range(20):
    sim = SO101Sim(scene_path=ROOT / "sim/so101_sim/assets/so101/scene_tek5.xml", seed=seed)
    sim.connect()
    image = sim.render_camera()
    pose, mask = detect_cube(image, sim.get_camera_intrinsics(), table_height=.025)
    truth = sim.get_object_position()
    results.append({"seed": seed, "truth": truth.tolist(), "estimate": pose.position.tolist() if pose else None,
        "error_m": float(np.linalg.norm(pose.position - truth)) if pose else None,
        "yaw": pose.yaw if pose else None, "residual": pose.residual_px if pose else None})
    if seed in (0, 1):
        from diagnose_geometry import vertices
        v = vertices(sim.model, sim.data, sim.model.geom("pickup_object_visual").id)
        local = (v - truth) @ sim.data.body("pickup_object").xmat.reshape(3, 3)
        results[-1]["visual_bounds"] = [local.min(0).tolist(), local.max(0).tolist()]
    if seed in (0, 1):
        cv2.imwrite(str(ROOT / f"artifacts/vision_mask_{seed}.png"), mask)
        rotation = sim.data.body("pickup_object").xmat.reshape(3, 3)
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
        p = project(cube_corners(truth, yaw), sim.get_camera_intrinsics())
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h = cv2.convexHull(max(contours, key=cv2.contourArea)).reshape(-1, 2)
        results[-1]["pixel_bounds"] = {"observed": [h.min(0).tolist(), h.max(0).tolist()], "projected": [p.min(0).tolist(), p.max(0).tolist()]}
        cv2.polylines(image, [cv2.convexHull(p.astype(np.int32))], True, (0, 255, 0), 1)
        cv2.imwrite(str(ROOT / f"artifacts/vision_overlay_{seed}.png"), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    sim.disconnect()
print(json.dumps(results, indent=2))
