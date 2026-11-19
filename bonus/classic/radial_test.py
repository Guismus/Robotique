"""Evaluation de la pyramide : vision IA, planification FK/IK et physique intacte."""

import argparse, json
from pathlib import Path
import numpy as np
from simulation_test import Simulation
from radial_planning import plan_cube, TARGETS, solve, line
from kinematics import HOME


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--cubes", type=int, default=6)
    a = parser.parse_args()
    sim = Simulation(a.seed)
    trace = []
    used = []
    out = Path("/project/bonus/classic/results")
    out.mkdir(exist_ok=True)
    try:
        for slot, (color, destination) in enumerate(TARGETS[: a.cubes]):
            sim.move(HOME, 0.88)
            sim.wait(0.5)
            candidates = [c for c in sim.detect() if c["color"] == color]
            selected = None
            for cube in sorted(
                candidates, key=lambda c: np.linalg.norm(c["xyz"][:2] - [0.22, 0.01])
            ):
                try:
                    steps, yaw = plan_cube(
                        sim.kin, cube["xyz"], cube["yaw"], destination, sim.q
                    )
                    selected = cube
                    break
                except ValueError:
                    continue
            if selected is None:
                raise RuntimeError(
                    f"Aucun cube visible et accessible de couleur {color}"
                )
            # Association reservee au rapport, jamais utilisee pour les commandes.
            ids = (
                range(5) if color == 1 else range(5, 8) if color == 2 else range(8, 10)
            )
            identity = min(
                ids, key=lambda i: np.linalg.norm(sim.truth()[i] - selected["xyz"])
            )
            used.append(identity)
            print("CUBE", slot + 1, "VISION", selected, flush=True)
            for index, (name, path) in enumerate(steps):
                print(slot + 1, name, flush=True)
                if name == "SAISIE":
                    sim.close()
                elif name == "OUVERTURE":
                    sim.force = None
                    sim.command[5] = 0.88
                    sim.wait(1.0)
                else:
                    sim.follow(path)
                if name == "TRANSPORT":
                    for correction in range(4):
                        pose = sim.kin.forward(sim.q)
                        seen = [
                            c
                            for c in sim.detect(float(pose[2, 3]), False)
                            if c["color"] == color
                        ]
                        if not seen:
                            raise RuntimeError("Cube tenu non visible")
                        cube = min(
                            seen,
                            key=lambda c: np.linalg.norm(
                                c["xyz"][:2] - destination[:2]
                            ),
                        )
                        delta = destination[:2] - cube["xyz"][:2]
                        print("CORRECTION", delta.tolist(), flush=True)
                        if np.linalg.norm(delta) < 0.003:
                            break
                        if np.linalg.norm(delta) > 0.08:
                            raise RuntimeError("Cube tenu non identifie")
                        adjusted = pose[:3, 3].copy()
                        adjusted[:2] += 0.8 * delta
                        sim.move(solve(sim.kin, adjusted, sim.q))
                    above = sim.kin.forward(sim.q)[:3, 3]
                    release = above.copy()
                    release[2] = destination[2] + 0.029
                    down = line(sim.kin, above, release, sim.q)
                    clear = line(sim.kin, release, release + [0, 0, 0.025], down[-1])
                    steps[index + 1 :] = [
                        ("DEPOT", down),
                        ("OUVERTURE", []),
                        ("RETRAIT", clear),
                    ]
                trace.append(
                    {
                        "slot": slot,
                        "phase": name,
                        "cube_for_evaluation": sim.truth()[identity].tolist(),
                    }
                )
            sim.move(HOME, 0.88)
            sim.wait(1.0)
            print("DEPOT MESURE", sim.truth()[identity].tolist(), flush=True)
        sim.wait(5.0)
        status = "sequence_terminee"
    except (ValueError, RuntimeError) as error:
        status = str(error)
        print("ARRET", status, flush=True)
    finally:
        positions = sim.truth()
        report = {
            "seed": a.seed,
            "status": status,
            "used_ids": used,
            "positions": positions.tolist(),
            "trace": trace,
        }
        (out / f"pyramid_{a.seed}.json").write_text(json.dumps(report, indent=2))
        sim.robot.disconnect()


if __name__ == "__main__":
    main()
