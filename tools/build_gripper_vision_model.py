"""Export known visual geometry for camera occlusion, without simulator state."""
from pathlib import Path
import hashlib
import json
import numpy as np
root = Path(__file__).resolve().parents[1]
assets = root / "sim/so101_sim/assets/so101/assets"
result = {}
dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3,3)), ("attribute", "<u2")])
for label, filename in (("fixed", "wrist_roll_follower_so101_v1.stl"), ("moving", "moving_jaw_so101_v1.stl")):
    data = (assets / filename).read_bytes()
    triangle = np.frombuffer(data, dtype=dtype, count=int.from_bytes(data[80:84], "little"), offset=84)["vertices"]
    vertices, indices = np.unique(triangle.reshape(-1,3), axis=0, return_inverse=True)
    result[label] = {"source": filename, "sha256": hashlib.sha256(data).hexdigest(), "vertices": vertices.astype(float).round(8).tolist(), "faces": indices.reshape(-1,3).tolist()}
(root / "ros2_ws/src/so101_perception/so101_perception/gripper_visual.json").write_text(json.dumps(result, separators=(",", ":")))
