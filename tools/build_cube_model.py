"""Export the known object's visual convex hull; never read dynamic sim state."""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

root = Path(__file__).resolve().parents[1]
source = root / "sim/so101_sim/assets/so101/assets/hollow_cube_rhombus_50mm.stl"
raw = source.read_bytes()
count = int.from_bytes(raw[80:84], "little")
triangles = np.frombuffer(raw, dtype=np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]), count=count, offset=84)
vertices = np.unique(triangles["vertices"].reshape(-1, 3), axis=0).astype(float) * .001 - [0, 0, .025]
hull = vertices[ConvexHull(vertices).vertices]
target = root / "ros2_ws/src/so101_perception/so101_perception/cube_hull.json"
target.write_text(json.dumps({"source": source.name, "sha256": hashlib.sha256(raw).hexdigest(), "units": "m", "vertices": hull.round(8).tolist()}, separators=(",", ":")))
print(f"Exported {len(hull)} known-shape vertices")
