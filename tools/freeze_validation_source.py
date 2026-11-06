"""Freeze allowlisted source/assets for an immutable multi-seed campaign."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
destination = args.output.resolve()
destination.relative_to((root / "artifacts").resolve())
if destination.exists():
    raise RuntimeError(f"Refusing to overwrite an existing snapshot: {destination}")
destination.mkdir(parents=True)
directories = ["sim/so101_sim"] + [f"ros2_ws/src/{name}" for name in ("so101_brain", "so101_driver", "so101_perception", "so101_description", "so101_interfaces")]
for relative in directories:
    shutil.copytree(root / relative, destination / relative, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
shutil.copytree(root / "tools", destination / "tools", ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
baseline = root / "artifacts/baseline"
if baseline.is_dir():
    (destination / "artifacts/baseline").mkdir(parents=True)
    for name in ("brain_node.py", "driver_node.py", "perception_node.py", "sim.py", "so101.urdf", "so101_tek5.xml", "scene_tek5.xml", "commit.txt"):
        shutil.copy2(baseline / name, destination / "artifacts/baseline" / name)
hashes = {path.relative_to(destination).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
    for path in sorted(destination.rglob("*")) if path.is_file()}
(destination / "source_manifest.json").write_text(json.dumps(hashes, indent=2))
print(json.dumps({"snapshot": str(destination), "files": len(hashes), "manifest_sha256": hashlib.sha256((destination / "source_manifest.json").read_bytes()).hexdigest()}))
