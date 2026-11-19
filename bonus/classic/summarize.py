"""Rassembler uniquement les essais complets du controleur actuel."""

import json
from pathlib import Path

root = Path("/project/bonus/classic/results")
rows = []
for seed in range(5):
    path = root / f"calibrated_pyramid_{seed}.json"
    if not path.exists():
        continue
    r = json.loads(path.read_text())
    if r.get("requested_cubes") != 6:
        continue
    rows.append(
        {
            "seed": seed,
            "success": r["success"],
            "status": r["status"],
            "placed": len(r["used_ids"]),
            "max_error_mm": 1000 * max(r["position_errors_m"], default=0),
            "drift_mm": (
                None
                if r.get("drift_after_5s_m") is None
                else 1000 * r["drift_after_5s_m"]
            ),
        }
    )
summary = {
    "tested_seeds": [r["seed"] for r in rows],
    "successes": sum(r["success"] for r in rows),
    "episodes": len(rows),
    "success_rate": sum(r["success"] for r in rows) / len(rows) if rows else None,
    "calibration_seeds": [0, 3],
    "validation_seeds": [1, 2, 4],
    "rows": rows,
}
(root / "validation_summary.json").write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
