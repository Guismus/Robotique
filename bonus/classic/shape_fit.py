"""Ajustement geometrique du cube connu au contour produit par l'IA."""

import json
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares

VERTICES = np.array(
    json.loads(
        Path(__file__).with_name("cube_hull.json").read_text(encoding="utf-8-sig")
    )["vertices"]
)
ANGLES = np.linspace(0, 2 * np.pi, 48, endpoint=False)
DIRECTIONS = np.c_[np.cos(ANGLES), np.sin(ANGLES)]


def fit_floor(contour, k, t, guess):
    hull = cv2.convexHull(contour).reshape(-1, 2).astype(float)
    measured = (hull @ DIRECTIONS.T).max(0)

    def residual(v):
        x, y, yaw = v
        c, s = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        world = VERTICES @ rotation.T + [x, y, 0.025]
        camera = (world - t[:3, 3]) @ t[:3, :3]
        pixel = camera @ k.T
        pixel = pixel[:, :2] / pixel[:, 2:3]
        return (pixel @ DIRECTIONS.T).max(0) - measured - 0.25

    fits = [
        least_squares(
            residual,
            [*guess[:2], yaw],
            bounds=([-0.4, -0.4, -np.pi], [0.45, 0.45, np.pi]),
            diff_step=1e-3,
            max_nfev=60,
        )
        for yaw in (0, 0.4, 0.8, 1.2)
    ]
    fit = min(fits, key=lambda f: np.linalg.norm(f.fun))
    error = float(np.sqrt(np.mean(fit.fun**2)))
    if error > 1.3:
        return None
    return np.r_[fit.x[:2], 0.025], float(fit.x[2]), error
