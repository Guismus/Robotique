"""Evaluation-only segmentation to diagnose the camera's brown-box mask."""
from pathlib import Path
import sys
import json
import cv2
import numpy as np
from scipy.optimize import least_squares
root = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(root / "sim"), str(root / "ros2_ws/src/so101_perception")]
from so101_sim import SO101Sim
from so101_perception.vision import detect_box, project, _DIRECTIONS, CAM_POS, R_CV
import itertools
sim = SO101Sim(scene_path=root / "sim/so101_sim/assets/so101/scene_tek5.xml", seed=0)
sim.connect()
rgb = sim.render_camera()
print("box", detect_box(rgb, sim.get_camera_intrinsics()))
hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
sim._renderer.enable_segmentation_rendering()
sim._renderer.update_scene(sim.data, camera=sim.camera_name)
seg = sim._renderer.render()
ids = np.where(sim.model.geom_bodyid == sim.model.body("drop_box").id)[0]
pixels = hsv[np.isin(seg[:, :, 0], ids)]
print("pixels", len(pixels), "HSV quantiles", np.quantile(pixels, [0, .1, .5, .9, 1], axis=0))
for i in ids:
    ys, xs = np.where(seg[:, :, 0] == i)
    if len(xs):
        print("geom", i, sim.model.geom_pos[i], "pixels", len(xs), "bbox", [xs.min(), ys.min(), xs.max(), ys.max()])
mask = cv2.inRange(hsv, (6, 70, 25), (25, 240, 220))
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
hull = cv2.convexHull(max(contours, key=cv2.contourArea)).reshape(-1, 2)
support = (hull @ _DIRECTIONS.T).max(0)
corners = np.array([p for p in itertools.product((-.052, .052), (-.052, .052), (0., .05)) if p[0] > 0 or p[1] > 0])
def residual(xy):
    return (project(corners + [*xy, 0], sim.get_camera_intrinsics()) @ _DIRECTIONS.T).max(0) - support - .25
fit = least_squares(residual, [.05, -.25])
print("fit", fit.x, "rmse", np.sqrt(np.mean(fit.fun ** 2)), "bounds", hull.min(0), hull.max(0))
cv2.imwrite(str(root / "artifacts/box_mask.png"), mask)
sim.disconnect()
