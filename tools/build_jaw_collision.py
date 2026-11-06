"""Regenerate convex jaw parts from the unmodified STL (CoACD 1.0.14).

Only collision shape fidelity changes. No added margins, extrusion, force,
friction, mass or equality constraints. Generated parts are runtime assets.
"""
import hashlib
import json
import re
from pathlib import Path
import numpy as np
import coacd
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "sim/so101_sim/assets/so101/assets"
source = ASSETS / "moving_jaw_so101_v1.stl"
raw = source.read_bytes()
dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
triangles = np.frombuffer(raw, dtype=dtype, count=int.from_bytes(raw[80:84], "little"), offset=84)["vertices"]
vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
faces = inverse.reshape(-1, 3)
parts = coacd.run_coacd(coacd.Mesh(vertices.astype(float), faces), threshold=.0005,
    real_metric=True, preprocess_mode="auto", seed=0, resolution=3000, mcts_iterations=150,
    extrude=False, decimate=False)
manifest = {"source": source.name, "source_sha256": hashlib.sha256(raw).hexdigest(),
    "coacd": "1.0.14", "threshold_m": .0005, "seed": 0, "parts": []}
original_samples = np.concatenate([vertices, triangles.mean(1)])
cover_distance = np.full(len(original_samples), np.inf)
for index, (v, f) in enumerate(parts):
    name = f"moving_jaw_collision_{index:02d}"
    target = ASSETS / f"{name}.stl"
    output = np.zeros(len(f), dtype=dtype)
    output["vertices"] = np.asarray(v)[f]
    normals = np.cross(output["vertices"][:, 1] - output["vertices"][:, 0], output["vertices"][:, 2] - output["vertices"][:, 0])
    output["normal"] = normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-20)
    target.write_bytes(b"STL-derived convex jaw collision".ljust(80, b" ") + len(f).to_bytes(4, "little") + output.tobytes())
    hull = ConvexHull(v)
    signed = (original_samples @ hull.equations[:, :3].T + hull.equations[:, 3]).max(1)
    cover_distance = np.minimum(cover_distance, signed)
    manifest["parts"].append({"name": name, "sha256": hashlib.sha256(target.read_bytes()).hexdigest()})
manifest["max_original_surface_outside_hulls_m"] = float(cover_distance.max())
if cover_distance.max() > .00075:
    raise RuntimeError(f"Generated jaw loses original surface: {cover_distance.max()}")
xml_path = ROOT / "sim/so101_sim/assets/so101/so101_tek5.xml"
xml = xml_path.read_text()
line = '                  <geom type="mesh" class="collision" pos="-5.55112e-17 -5.55112e-17 0.0189" quat="1 -0 3.00524e-16 -2.00834e-17" mesh="moving_jaw_so101_v1" material="moving_jaw_so101_v1_material"/>'
replacement = '\n'.join(line.replace('mesh="moving_jaw_so101_v1"', f'mesh="{part["name"]}"') for part in manifest["parts"])
block = '                  <!-- STL-derived convex parts preserve the concave finger surface. -->\n' + replacement
if line in xml:
    xml = xml.replace(line, block)
else:
    pattern = r'                  <!-- STL-derived convex parts preserve the concave finger surface\. -->\n(?:                  <geom [^\n]*mesh="moving_jaw_collision_\d+"[^\n]*/>\n?)+'
    xml, count = re.subn(pattern, lambda match: block + '\n', xml)
    if count != 1:
        raise RuntimeError("Cannot uniquely identify the generated jaw collision block")
xml = re.sub(r'^    <mesh file="moving_jaw_collision_\d+\.stl"/>\n', '', xml, flags=re.MULTILINE)
asset_line = '    <mesh file="moving_jaw_so101_v1.stl"/>'
xml = xml.replace(asset_line, asset_line + '\n' + '\n'.join(f'    <mesh file="{part["name"]}.stl"/>' for part in manifest["parts"]))
xml_path.write_text(xml)
(ASSETS / "moving_jaw_collision_manifest.json").write_text(json.dumps(manifest, indent=2))
print(json.dumps(manifest, indent=2))
