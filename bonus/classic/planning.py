"""Plan complet classique, calcule avant la prise."""

import numpy as np
from kinematics import rotation, SHOULDER


def plan_cube(kin, source, yaw, destination, q):
    for ty in (np.pi / 2, -np.pi / 2):
        source_angles = sorted(
            yaw + np.arange(4) * np.pi / 2,
            key=lambda a: abs((a - ty + np.pi) % (2 * np.pi) - np.pi),
        )
        for sy in source_angles:
            pickup = source + rotation(sy)[:, 0] * 0.003
            try:
                approach = kin.solve_pose(pickup + [0, 0, 0.045], q, sy, 28)
                lower = kin.path(pickup + [0, 0, 0.045], pickup, approach, sy, 8)
                first = pickup + [0, 0, 0.065]
                lift = kin.path(pickup, first, lower[-1], sy, 30)
                radial = first[:2] - SHOULDER
                lift_point = np.r_[
                    SHOULDER
                    + radial
                    * max(
                        0.7, (np.linalg.norm(radial) - 0.025) / np.linalg.norm(radial)
                    ),
                    max(0.14, destination[2] + 0.055),
                ]
                lift += kin.path(first, lift_point, lift[-1], sy, 55)
                above = destination + rotation(ty)[:, 0] * 0.003
                above[2] = max(0.14, destination[2] + 0.055)
                arc = kin.arc(lift_point, above, lift[-1], sy, ty)
                release = (
                    destination
                    + rotation(ty)[:, 0] * 0.003
                    + [0, 0, 0.012 if destination[2] > 0.04 else 0.004]
                )
                down = kin.path(
                    above, release, arc[-1], ty, 55 if destination[2] > 0.1 else 28
                )
                clear = kin.path(release, release + [0, 0, 0.045], down[-1], ty, 60)
                return [
                    ("APPROCHE", [approach]),
                    ("DESCENTE", lower),
                    ("SAISIE", []),
                    ("LEVAGE", lift),
                    ("TRANSPORT", arc),
                    ("DEPOT", down),
                    ("OUVERTURE", []),
                    ("RETRAIT", clear),
                ], ty
            except ValueError:
                continue
    raise ValueError("Aucun trajet complet disponible pour ce cube")
