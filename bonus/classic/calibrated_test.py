"""Evaluation physique separee du controle par vision et FK/IK."""

import argparse, json
from pathlib import Path
import numpy as np
from simulation_test import Simulation
from calibrated_planning import plan_cube, TARGETS
from kinematics import HOME
from radial import orientation


def recover(sim):
    sim.force = None
    sim.command[5] = 0.88
    sim.wait(6)
    point = sim.kin.forward(sim.q)[:3, 3]
    for height in np.linspace(point[2], max(0.10, point[2]), 15)[1:]:
        target = np.r_[point[:2], height]
        r, yaw = orientation(target, -20)
        sim.move(sim.kin.solve_pose(target, sim.q, yaw, 8, r))
    sim.move(HOME, 0.88)
    sim.wait(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--cubes", type=int, choices=range(1, 7), default=6)
    parser.add_argument("--video", action="store_true")
    a = parser.parse_args()
    sim = Simulation(a.seed)
    sim.arm_speed = 0.08
    sim.grip_force = -1.2
    trace = []
    used = []
    attempts = []
    status = "non_demarre"
    out = Path("/project/bonus/classic/results")
    out.mkdir(exist_ok=True)
    recorder = None
    if a.video:
        from recording import Recorder

        recorder = Recorder(sim, out / f"pyramid_{a.seed}.mp4")
        sim.observer = recorder.frame
        sim.last_observe = -1
    try:
        for slot, (color, destination) in enumerate(TARGETS[: a.cubes]):
            failed = []
            placed = False
            for attempt in range(3):
                sim.move(HOME, 0.88)
                sim.wait(1)
                candidates = [
                    c
                    for c in sim.detect()
                    if c["color"] == color
                    and all(
                        np.linalg.norm(c["xyz"][:2] - p[:2]) > 0.045 for p in failed
                    )
                ]
                selected = None
                for cube in sorted(
                    candidates,
                    key=lambda c: np.linalg.norm(c["xyz"][:2] - [0.22, 0.01]),
                ):
                    try:
                        steps, _ = plan_cube(
                            sim.kin, cube["xyz"], cube["yaw"], destination, sim.q
                        )
                        selected = cube
                        break
                    except ValueError as error:
                        print("CANDIDAT REFUSE", str(error), flush=True)
                if selected is None:
                    raise RuntimeError(
                        f"Aucun cube visible et accessible de couleur {color}"
                    )
                # Les identites physiques ne servent qu'au rapport, pas a la commande.
                ids = (
                    range(5)
                    if color == 1
                    else range(5, 8) if color == 2 else range(8, 10)
                )
                identity = min(
                    ids, key=lambda i: np.linalg.norm(sim.truth()[i] - selected["xyz"])
                )
                print(
                    "CUBE",
                    slot + 1,
                    "ESSAI",
                    attempt + 1,
                    "VISION",
                    selected,
                    flush=True,
                )
                try:
                    for name, path in steps:
                        if recorder:
                            recorder.phase = f"Cube {slot+1}/6 - {name}"
                        print(slot + 1, name, flush=True)
                        if name == "SAISIE":
                            sim.close()
                        elif name == "OUVERTURE":
                            sim.move(sim.q, 0.88)
                            sim.wait(6)
                        else:
                            for q in path:
                                sim.move(q)
                                if (
                                    sim.force is not None
                                    and sim.d.qpos[sim.ids[5]] < 0.15
                                ):
                                    raise RuntimeError("Perte de prise detectee")
                        trace.append(
                            {
                                "slot": slot,
                                "attempt": attempt,
                                "phase": name,
                                "cube_for_evaluation": sim.truth()[identity].tolist(),
                                "all_cubes_for_evaluation": sim.truth().tolist(),
                            }
                        )
                except RuntimeError as error:
                    attempts.append(
                        {"slot": slot, "attempt": attempt, "error": str(error)}
                    )
                    print("REPRISE", str(error), flush=True)
                    failed.append(selected["xyz"].copy())
                    recover(sim)
                    continue
                sim.move(HOME, 0.88)
                sim.wait(4)
                print("DEPOT MESURE", sim.truth()[identity].tolist(), flush=True)
                visible = sim.vision.placement(
                    sim.robot.render_camera(),
                    sim.robot.get_camera_intrinsics(),
                    sim.robot.get_camera_extrinsics() @ np.diag([1, -1, -1, 1]),
                    destination,
                    color,
                )
                if not visible:
                    raise RuntimeError(
                        "Depot non confirme par la vision : sequence arretee"
                    )
                used.append(identity)
                placed = True
                break
            if not placed:
                raise RuntimeError("Trois prises manquees : sequence arretee")
        stability_start = sim.truth().copy()
        sim.wait(5)
        status = "sequence_terminee"
    except (ValueError, RuntimeError) as error:
        status = str(error)
        print("ARRET", status, flush=True)
    finally:
        if recorder:
            recorder.finish(sim, out / f"pyramid_{a.seed}_final.png")
        from recording import final_image

        final_image(sim, out / f"calibrated_pyramid_{a.seed}_final.png")
        positions = sim.truth()
        drift = (
            max(
                (
                    float(np.linalg.norm(positions[i] - stability_start[i]))
                    for i in used
                ),
                default=0.0,
            )
            if status == "sequence_terminee"
            else None
        )
        upright = all(
            float(
                sim.d.body("pickup_object" if i == 0 else f"cube_{i}").xmat.reshape(
                    3, 3
                )[2, 2]
            )
            > 0.98
            for i in used
        )
        errors = [
            float(np.linalg.norm(positions[i] - TARGETS[n][1]))
            for n, i in enumerate(used)
        ]
        report = {
            "seed": a.seed,
            "requested_cubes": a.cubes,
            "drift_after_5s_m": drift,
            "upright": upright,
            "status": status,
            "position_errors_m": errors,
            "success": status == "sequence_terminee"
            and len(used) == a.cubes
            and len(set(used)) == a.cubes
            and all(e < 0.012 for e in errors)
            and drift is not None
            and drift < 0.002
            and upright,
            "used_ids": used,
            "positions": positions.tolist(),
            "attempts": attempts,
            "trace": trace,
        }
        (out / f"calibrated_pyramid_{a.seed}.json").write_text(
            json.dumps(report, indent=2)
        )
        sim.robot.disconnect()
    return 0 if report["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
