"""A lancer dans control : python3 /ros2_ws/../... ; voir guide."""
import sys
from pathlib import Path
import unittest
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/so101_brain'))
from so101_brain.kinematics import Kinematics


class KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.kin = Kinematics(ROOT / 'sim/so101_sim/assets/so101/so101_new_calib.urdf')

    def test_pick_and_drop_targets(self):
        current = np.zeros(5)
        for target in [[.25, .05, .08], [.25, .05, .025], [.05, -.25, .08]]:
            current = self.kin.solve(np.array(target), current)
            self.assertLess(np.linalg.norm(self.kin.forward(current)[:3, 3] - target), .008)
            self.assertTrue(np.all(current >= self.kin.bounds[0]))
            self.assertTrue(np.all(current <= self.kin.bounds[1]))

    def test_far_target_is_refused(self):
        with self.assertRaises(ValueError):
            self.kin.solve(np.array([10., 0., 0.]), np.zeros(5))

if __name__ == '__main__':
    unittest.main()
