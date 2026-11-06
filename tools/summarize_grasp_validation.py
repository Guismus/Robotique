"""Aggregate immutable per-seed evidence; never infer success from FINISHED."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def read_results(directory):
    return {int(path.parent.name.split("_")[-1]): (json.loads(path.read_text()), path.parent)
        for path in directory.glob("seed_*/result.json")}


def approach_error(trace):
    for previous, current in zip(trace, trace[1:]):
        if previous["state"] == "APPROACH" and current["state"] == "LOWER":
            target = (previous.get("ik") or {}).get("target")
            if target:
                return float(np.linalg.norm(np.asarray(previous["tcp_fk"]) - target))
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, nargs="+", required=True)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    original = read_results(args.before)
    collected = {}
    runs = []
    for directory in args.after:
        current = read_results(directory)
        if collected.keys() & current.keys():
            raise RuntimeError("Duplicate seed across campaigns")
        collected.update(current)
        runs.append(json.loads((directory / "summary.json").read_text()))
    if set(collected) != set(range(20)) or set(original) != set(range(20)):
        raise RuntimeError("Both comparisons must contain all 20 seeds, 0 through 19")
    if not all(run["sources_unchanged"] for run in runs):
        raise RuntimeError("A campaign changed its source inputs")
    if not all(run["source_hashes"] == runs[0]["source_hashes"] for run in runs):
        raise RuntimeError("Campaigns used different runtime sources")
    manifest_path = args.snapshot / "source_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    damaged = [name for name, expected in manifest.items()
        if hashlib.sha256((args.snapshot / name).read_bytes()).hexdigest() != expected]
    if damaged:
        raise RuntimeError(f"Snapshot source changed: {damaged}")
    live_mismatches = [name for name, expected in runs[0]["source_hashes"].items()
        if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != expected]
    results = []
    for seed, (raw, directory) in sorted(collected.items()):
        trace = json.loads((directory / "trace.json").read_text())
        record = dict(raw)
        record["raw_success"] = raw["success"]
        # Include robot/box and robot/base contacts as well as the original
        # evaluator's cube, robot/ground and self-contact checks. This can only
        # make acceptance stricter; the underlying per-seed evidence is kept.
        maximum = max(raw["phase_penetrations"].values(), default=0.)
        record["all_contacts_max_penetration_m"] = maximum
        record["collision_ok"] = maximum < .003
        record["success"] = bool(raw["detected"] and raw["physical_grasp"] and raw["inside_box"] and record["collision_ok"])
        record["approach_error_m"] = approach_error(trace)
        record["trace"] = str(directory / "trace.json")
        record["initial_pose_difference_from_baseline_m"] = float(np.linalg.norm(np.asarray(raw["initial"]) - original[seed][0]["initial"]))
        record["recoveries"] = sum(row["state"] == "RECOVER" and (i == 0 or trace[i - 1]["state"] != "RECOVER") for i, row in enumerate(trace))
        record["application_success"] = bool(record["success"] and raw["state"] == "FINISHED")
        results.append(record)
    if max(row["initial_pose_difference_from_baseline_m"] for row in results) > 1e-7:
        raise RuntimeError("Initial object poses changed relative to the baseline")
    successes = sum(row["success"] for row in results)
    baseline = [item[0] for item in original.values()]
    summary = {"successes": successes, "trials": 20, "success_rate": successes / 20,
        "status": "PASS" if successes >= 18 else "FAIL", "before_successes": sum(row["success"] for row in baseline),
        "counts": {name: sum(bool(row[name]) for row in results) for name in ("detected", "bilateral_contacts_seen", "lifted_5cm", "held_transport", "physical_grasp", "inside_box", "collision_ok", "application_success")},
        "before_counts": {name: sum(bool(row.get(name, False)) for row in baseline) for name in ("detected", "lifted_5cm", "held_transport", "physical_grasp", "inside_box")},
        "criteria": runs[0]["criteria"], "mujoco": runs[0]["mujoco"], "numpy": runs[0]["numpy"], "opencv": runs[0]["opencv"],
        "source_hashes": runs[0]["source_hashes"], "live_runtime_matches_snapshot": not live_mismatches,
        "live_mismatches": live_mismatches, "snapshot": str(args.snapshot),
        "snapshot_manifest_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        "snapshot_integrity": "PASS", "results": results}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2))
    lines = ["# Mesures par seed", "", f"Résultat strict : **{successes}/20 ({successes * 5} %) — {summary['status']}**.", "",
        "Succès = détection + contacts bilatéraux + levage réel ≥5cm + maintien durant tout le transport + dépôt entier + tous les contacts <3mm.", "",
        "| Seed | Détection (err. mm) | Approche (err. mm) | Contact bilat. haut (s) | Levage max (cm) | Min transport Δz (cm) | Dans boîte | Pénétration max (mm) | État | Succès |",
        "|---:|---:|---:|---:|---:|---:|:---:|---:|:---|:---:|"]
    for row in results:
        def number(value, scale=1):
            return f"{value * scale:.2f}" if value is not None else "MISSING"
        minimum = row["minimum_transport_height_m"]
        delta = minimum - row["initial"][2] if minimum is not None else None
        lines.append(f"| {row['seed']} | {number(row['initial_detection_error_m'], 1000)} | {number(row['approach_error_m'], 1000)} | {row['bilateral_lift_contact_seconds']:.2f} | {number(row['maximum_lift_m'], 100)} | {number(delta, 100)} | {'oui' if row['inside_box'] else 'non'} | {number(row['all_contacts_max_penetration_m'], 1000)} | {row['state']} | {'PASS' if row['success'] else 'FAIL'} |")
    lines += ["", "Les récupérations restent incluses dans les minima et les pics du cycle ; un échec antérieur n'est pas masqué.",
        "Le score agrégé prend aussi en compte les contacts robot/boîte, conservés dans phase_penetrations. Les result.json bruts ne sont pas réécrits.",
        "", f"Intégrité du snapshot : PASS. SHA-256 du manifeste : `{summary['snapshot_manifest_sha256']}`.",
        f"Sources applicatives courantes identiques au snapshot : {'PASS' if not live_mismatches else 'FAIL'}."]
    (args.output / "per_seed.md").write_text("\n".join(lines) + "\n")
    print(json.dumps({key: summary[key] for key in ("status", "successes", "trials", "success_rate", "counts", "snapshot_integrity", "live_runtime_matches_snapshot")}))
    return 0 if summary["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
