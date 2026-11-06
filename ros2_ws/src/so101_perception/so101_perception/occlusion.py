"""Robot-aware image visibility from URDF frames and original visual meshes.

Uses joint/FK observations only. No simulator, contacts or object ground truth.
"""
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

_DATA = json.loads(Path(__file__).with_name("gripper_visual.json").read_text())
_MESHES = {name: (np.asarray(value["vertices"]), np.asarray(value["faces"])) for name, value in _DATA.items()}


def gripper_depth(shape, K, cam_position, camera_rotation, tcp_position, tcp_rotation, angle):
    """Approximate z-buffer of just the gripper; no rendering engine needed."""
    height, width = shape[:2]
    depth = np.full((height, width), np.inf, dtype=np.float32)
    body_rotation = tcp_rotation @ np.diag([-1., 1., -1.])
    body_position = tcp_position - body_rotation @ np.array([.0171, -.000218121, -.0901274])
    fixed_v, fixed_f = _MESHES["fixed"]
    fixed_body = fixed_v @ np.diag([1., -1., -1.]) + [0., -.000218214, .000949706]
    moving_v, moving_f = _MESHES["moving"]
    moving_rotation = Rotation.from_euler("x", np.pi / 2).as_matrix() @ Rotation.from_euler("z", angle).as_matrix()
    moving_body = (moving_v + [0., 0., .0189]) @ moving_rotation.T + [.0202, .0188, -.0234]
    polygons, distances = [], []
    for vertices, faces in ((fixed_body, fixed_f), (moving_body, moving_f)):
        world = vertices @ body_rotation.T + body_position
        camera = (world - cam_position) @ camera_rotation
        pixels = camera @ K.T
        pixels = pixels[:, :2] / pixels[:, 2:3]
        valid = (camera[faces, 2].min(axis=1) > .001)
        faces = faces[valid]
        triangle = np.rint(pixels[faces]).astype(np.int32)
        z = camera[faces, 2].min(axis=1)
        polygons.extend(triangle)
        distances.extend(z)
    # Conservative triangle depth; near triangles overwrite distant ones.
    # This map is an occlusion bound, not an object-depth measurement.
    for index in np.argsort(distances)[::-1]:
        cv2.fillConvexPoly(depth, polygons[index], float(distances[index]))
    return depth
