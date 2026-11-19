"""Vision entrainee : les positions viennent seulement du RGB et de la calibration."""

import sys
import cv2, numpy as np, torch

sys.path.insert(0, "/project/bonus")
from network import CubeNet
from shape_fit import fit_floor


class CubeVision:
    def __init__(self):
        torch.set_num_threads(1)
        self.model = CubeNet()
        self.model.load_state_dict(
            torch.load(
                "/project/bonus/cube_net.pt", map_location="cpu", weights_only=True
            )
        )
        self.model.eval()

    def labels(self, rgb):
        with torch.no_grad():
            return (
                self.model(
                    torch.from_numpy(rgb.copy()).permute(2, 0, 1)[None].float() / 255
                )
                .argmax(1)[0]
                .numpy()
                .astype("uint8")
            )

    def placement(self, rgb, k, t, destination, color, labels=None):
        # Quatre coins du dessus : eviter le trou central et les contours fusionnes.
        labels = self.labels(rgb) if labels is None else labels
        scores = []
        for dx, dy in (
            (-0.018, -0.018),
            (-0.018, 0.018),
            (0.018, -0.018),
            (0.018, 0.018),
        ):
            world = np.r_[np.asarray(destination) + [dx, dy, 0.025], 1.0]
            camera = np.linalg.inv(t) @ world
            pixel = k @ camera[:3]
            if camera[2] <= 0:
                return False
            u, v = np.rint(pixel[:2] / pixel[2]).astype(int)
            patch = (
                labels[max(0, v - 2) : v + 3, max(0, u - 2) : u + 3]
                if 0 <= u < labels.shape[1] and 0 <= v < labels.shape[0]
                else np.array([])
            )
            scores.append(bool(patch.size and np.mean(patch == color) > 0.4))
        return sum(scores) >= 3

    def detect(self, rgb, k, t, top=0.05, source_only=True, labels=None):
        mask = self.labels(rgb) if labels is None else labels

        def project(u, v):
            ray = t[:3, :3] @ np.linalg.solve(k, [u, v, 1])
            o = t[:3, 3]
            return o + (top - o[2]) / ray[2] * ray

        cubes = []
        for color in (1, 2, 3):
            contours, _ = cv2.findContours(
                (mask == color).astype("uint8"),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            for contour in contours:
                if cv2.contourArea(contour) < 30:
                    continue
                rect = cv2.minAreaRect(contour)
                (u, v), (w, h), angle = rect
                expected = k[1, 1] * 0.05 / abs(t[2, 3] - top)
                if source_only and (
                    min(w, h) < 0.75 * expected
                    or cv2.contourArea(contour) < 0.65 * expected**2
                ):
                    continue
                p = project(u, v)
                p[2] = top - 0.025
                if source_only and p[1] < -0.11:
                    continue
                a = np.radians(angle)
                edge = project(u + np.cos(a), v + np.sin(a)) - project(u, v)
                yaw = float(np.arctan2(edge[1], edge[0]))
                if source_only:
                    fitted = fit_floor(contour, k, t, p)
                    if fitted is None:
                        continue
                    p, yaw, _ = fitted
                cubes.append(
                    {
                        "color": color,
                        "xyz": p,
                        "yaw": yaw,
                        "pixels": float(cv2.contourArea(contour)),
                    }
                )
        return cubes
