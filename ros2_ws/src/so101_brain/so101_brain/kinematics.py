"""Calculs du bras : lire la geometrie, puis chercher cinq angles."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']


class Kinematics:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.origins, self.axes, self.bounds = [], [], []
        for name in NAMES + ['gripper_frame_joint']:
            joint = root.find(f"joint[@name='{name}']")
            origin = joint.find('origin')
            matrix = np.eye(4)
            matrix[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
            matrix[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
            self.origins.append(matrix)
            if name in NAMES:
                self.axes.append(np.fromstring(joint.find('axis').get('xyz'), sep=' '))
                limit = joint.find('limit')
                self.bounds.append((float(limit.get('lower')), float(limit.get('upper'))))
        self.bounds = np.array(self.bounds).T

    def forward(self, angles):
        matrix = np.eye(4)
        for origin, axis, angle in zip(self.origins, self.axes, angles):
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            matrix = matrix @ origin @ rotation
        return matrix @ self.origins[-1]

    def solve(self, point, current, tilt=False):
        """tilt=False : pince verticale (+-11 deg). tilt=True : jusqu'a ~35 deg d'inclinaison,
        pour monter plus haut (pince verticale : 12 cm max ; 3e cube de la pile : ~13,5 cm)."""
        weight, max_tilt = (0.01, 0.6) if tilt else (0.05, 0.2)

        def error(q):
            pose = self.forward(q)
            # La pince pointe vers la table. Sa rotation autour de Z reste libre.
            return np.r_[pose[:3, 3] - point, weight * (pose[:3, 2] - [0, 0, -1])]
        best = None
        seeds = [current, np.zeros(5), [0, -0.8, 0.8, 0.8, 0]] + ([[0, -1.2, 0.5, 1.2, 0]] if tilt else [])
        for seed in seeds:
            result = least_squares(error, np.clip(seed, *self.bounds), bounds=self.bounds, max_nfev=150)
            if best is None or np.linalg.norm(error(result.x)) < np.linalg.norm(error(best)):
                best = result.x
        pose = self.forward(best)
        if np.linalg.norm(pose[:3, 3] - point) > 0.008 or np.linalg.norm(pose[:3, 2] - [0, 0, -1]) > max_tilt:
            raise ValueError('Cible inaccessible avec la pince verticale')
        return best
