import cv2, numpy as np
from simulation_test import Simulation
from shape_fit import fit_floor

s = Simulation(0)
rgb = s.robot.render_camera()
k = s.robot.get_camera_intrinsics()
t = s.robot.get_camera_extrinsics() @ np.diag([1, -1, -1, 1])
labels = s.vision.labels(rgb)
for color in (1, 2, 3):
    contours, _ = cv2.findContours(
        (labels == color).astype("uint8"), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    for c in contours:
        if cv2.contourArea(c) < 500:
            continue
        (u, v), _, _ = cv2.minAreaRect(c)
        ray = t[:3, :3] @ np.linalg.solve(k, [u, v, 1])
        guess = t[:3, 3] + (0.05 - t[2, 3]) / ray[2] * ray
        guess[2] = 0.025
        result = fit_floor(c, k, t, guess)
        if result is not None:
            fit, yaw, error = result
            ids = (
                range(5) if color == 1 else range(5, 8) if color == 2 else range(8, 10)
            )
            truth = min(s.truth()[list(ids)], key=lambda p: np.linalg.norm(p - guess))
            print(
                color,
                "avant mm",
                np.round(1000 * (guess - truth), 2),
                "apres mm",
                np.round(1000 * (fit - truth), 2),
                "pixel",
                error,
                flush=True,
            )
s.robot.disconnect()
