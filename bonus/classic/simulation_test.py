"""Banc physique : moteur classique et vision IA, sans lecture d'objet pour commander."""

import sys, json, argparse, time
from pathlib import Path
import numpy as np, mujoco

sys.path.insert(0, "/project/sim")
from so101_sim import SO101Sim, ARM_JOINTS
from kinematics import ClassicalIK, HOME, rotation
from vision import CubeVision


class Simulation:
    def __init__(self, seed, vision=True):
        self.robot = SO101Sim(
            scene_path="/project/sim/so101_sim/assets/so101/scene_classic.xml",
            camera_width=640,
            camera_height=480,
            seed=seed,
        )
        self.robot.connect()
        self.m, self.d = self.robot.model, self.robot.data
        self.names = ARM_JOINTS + ["gripper"]
        self.ids = [self.m.joint(n).qposadr[0] for n in self.names]
        self.dofs = [self.m.joint(n).dofadr[0] for n in self.names]
        self.act = [self.m.actuator(n).id for n in self.names]
        self.command = np.r_[HOME, 0.88]
        self.d.qpos[self.ids] = self.command
        self.d.ctrl[self.act] = self.command
        self.velocity = np.zeros(6)
        self.force = None
        self.kin = ClassicalIK()
        self.vision = CubeVision() if vision else None
        rng = np.random.default_rng(seed)
        slots = [(x, y) for x in (0.14, 0.22, 0.30) for y in (-0.07, 0.01, 0.09, 0.17)]
        rng.shuffle(slots)
        for name, (x, y) in zip(
            ["pickup_object"] + [f"cube_{i}" for i in range(1, 10)], slots
        ):
            j = self.m.joint(name + "_free").qposadr[0]
            yaw = (
                np.arctan2(y, x - 0.0388353) + np.pi / 2
            )  # Faces presentees dans le plan de prise
            self.d.qpos[j : j + 3] = [
                x + rng.uniform(-0.002, 0.002),
                y + rng.uniform(-0.002, 0.002),
                0.026,
            ]
            self.d.qpos[j + 3 : j + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        self.d.qvel[:] = 0
        mujoco.mj_forward(self.m, self.d)
        for _ in range(500):
            self.step()

    @property
    def q(self):
        return self.d.qpos[self.ids[:5]].copy()

    def step(self):
        dt = self.m.opt.timestep
        for i, a in enumerate(self.act):
            if i == 5 and self.force is not None:
                kp = self.m.actuator_gainprm[a, 0]
                kv = -self.m.actuator_biasprm[a, 2]
                ref = (
                    self.d.qpos[self.ids[i]]
                    + (self.force + kv * self.d.qvel[self.dofs[i]]) / kp
                )
                self.d.ctrl[a] = np.clip(ref, *self.m.actuator_ctrlrange[a])
                continue
            error = self.command[i] - self.d.ctrl[a]
            speed = getattr(self, "arm_speed", 0.25) if i < 5 else 0.1
            acc = 0.8 if i < 5 else 1.2
            v = np.sign(error) * min(speed, np.sqrt(2 * acc * abs(error)))
            self.velocity[i] += np.clip(v - self.velocity[i], -acc * dt, acc * dt)
            step = self.velocity[i] * dt
            if abs(step) > abs(error):
                self.d.ctrl[a] = self.command[i]
                self.velocity[i] = 0
            else:
                self.d.ctrl[a] += step
        mujoco.mj_step(self.m, self.d)
        if getattr(self, "observer", None) and self.d.time - self.last_observe >= 0.1:
            self.last_observe = self.d.time
            self.observer(self)

    def wait(self, seconds):
        for _ in range(int(seconds / self.m.opt.timestep)):
            self.step()

    def move(self, q, grip=None):
        self.command[:5] = q
        if grip is not None:
            self.force = None
            self.command[5] = grip
        timeout = 3 + 1.7 * np.max(np.abs(q - self.q)) / getattr(
            self, "arm_speed", 0.25
        )
        for _ in range(int(timeout / self.m.opt.timestep)):
            self.step()
            if (
                np.max(np.abs(self.q - q)) < 0.008
                and np.max(np.abs(self.d.qvel[self.dofs[:5]])) < 0.05
            ):
                return
        raise RuntimeError("Trajectoire bloquee ou cible moteur non atteinte")

    def follow(self, points):
        for q in points:
            self.move(q)

    def close(self):
        self.force = None
        self.command[5] = -0.174533
        loaded = 0
        for _ in range(int(15 / self.m.opt.timestep)):
            self.step()
            effort = self.d.actuator_force[self.act[5]]
            loaded = loaded + 1 if effort < -0.4 else 0
            if loaded * self.m.opt.timestep > 0.10:
                self.force = getattr(self, "grip_force", -0.6)
                settled = 0
                for hold in range(int(3 / self.m.opt.timestep)):
                    self.step()
                    speed = abs(self.d.qvel[self.dofs[5]])
                    settled = (
                        settled + 1
                        if speed < 0.025 and hold * self.m.opt.timestep > 0.5
                        else 0
                    )
                    if settled * self.m.opt.timestep > 0.25:
                        if self.d.qpos[self.ids[5]] < -0.13:
                            raise RuntimeError("Pince fermee a vide")
                        return
                raise RuntimeError("Serrage non stabilise")
        raise RuntimeError("Aucun effort de prise mesure")

    def detect(self, top=0.05, source_only=True):
        mujoco.mj_forward(self.m, self.d)
        return self.vision.detect(
            self.robot.render_camera(),
            self.robot.get_camera_intrinsics(),
            self.robot.get_camera_extrinsics() @ np.diag([1, -1, -1, 1]),
            top,
            source_only,
        )

    def truth(self):
        return np.array(
            [
                self.d.body(n).xpos.copy()
                for n in ["pickup_object"] + [f"cube_{i}" for i in range(1, 10)]
            ]
        )


def plan(kin, source, yaw, destination, q):
    for sy in yaw + np.arange(4) * np.pi / 2:
        pickup = source + rotation(sy)[:, 0] * 0.003
        try:
            approach = kin.solve_pose(pickup + [0, 0, 0.045], q, sy, 28)
            lower = kin.path(pickup + [0, 0, 0.045], pickup, approach, sy, 8)
            lift_point = pickup + [0, 0, 0.085]
            lift = kin.path(pickup, lift_point, lower[-1], sy, 28)
            # Garder le yaw durant ce premier diagnostic de prise/depot.
            above = destination + rotation(sy)[:, 0] * 0.003 + [0, 0, 0.06]
            arc = kin.arc(lift_point, above, lift[-1], sy)
            drop = destination + rotation(sy)[:, 0] * 0.003
            down = kin.path(above, drop, arc[-1], sy, 10)
            clear = kin.path(drop, drop + [0, 0, 0.045], down[-1], sy, 28)
            return [
                ("APPROCHE", [approach]),
                ("DESCENTE", lower),
                ("SAISIE", []),
                ("LEVAGE", lift),
                ("TRANSPORT", arc),
                ("DEPOT", down),
                ("OUVERTURE", []),
                ("RETRAIT", clear),
            ], sy
        except ValueError:
            continue
    raise ValueError("Aucune trajectoire complete disponible")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seed", type=int, default=0)
    a = p.parse_args()
    sim = Simulation(a.seed)
    trace = []
    initial = sim.truth()
    destination = np.array([0.11, -0.17, 0.025])
    selected = None
    for cube in sorted(sim.detect(), key=lambda c: -c["pixels"]):
        if cube["color"] != 1:
            continue
        try:
            steps, yaw = plan(sim.kin, cube["xyz"], cube["yaw"], destination, sim.q)
            selected = cube
            break
        except ValueError:
            continue
    if selected is None:
        raise RuntimeError("Aucun cube rouge avec trajet complet")
    identity = int(np.argmin(np.linalg.norm(initial[:5] - selected["xyz"], axis=1)))
    print("VISION", selected, "EVALUATION_CUBE", identity, flush=True)
    for step_index, (name, path) in enumerate(steps):
        print(name, flush=True)
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
                detections = [
                    c
                    for c in sim.detect(float(pose[2, 3] + 0.025), False)
                    if c["color"] == 1
                ]
                seen = min(
                    detections,
                    key=lambda c: np.linalg.norm(c["xyz"][:2] - destination[:2]),
                )
                delta = destination[:2] - seen["xyz"][:2]
                print("CORRECTION VISION", delta.tolist(), flush=True)
                if np.linalg.norm(delta) < 0.003:
                    break
                if np.linalg.norm(delta) > 0.08:
                    raise RuntimeError("Cube tenu non identifie par la camera")
                adjusted = pose[:3, 3].copy()
                adjusted[:2] += 0.8 * delta
                sim.move(sim.kin.solve_pose(adjusted, sim.q, yaw, 28))
            above = sim.kin.forward(sim.q)[:3, 3]
            drop = above.copy()
            drop[2] = destination[2]
            down = sim.kin.path(above, drop, sim.q, yaw, 10)
            clear = sim.kin.path(drop, drop + [0, 0, 0.045], down[-1], yaw, 28)
            steps[step_index + 1 :] = [
                ("DEPOT", down),
                ("OUVERTURE", []),
                ("RETRAIT", clear),
            ]
        trace.append(
            {
                "phase": name,
                "cube": sim.truth()[identity].tolist(),
                "joints": sim.q.tolist(),
            }
        )
    sim.wait(2.0)
    final = sim.truth()[identity]
    print(
        "FINAL",
        final.tolist(),
        "erreur",
        float(np.linalg.norm(final - destination)),
        flush=True,
    )
    out = Path("/project/bonus/classic/results")
    out.mkdir(exist_ok=True)
    (out / f"seed_{a.seed}.json").write_text(
        json.dumps(
            {
                "seed": a.seed,
                "destination": destination.tolist(),
                "final": final.tolist(),
                "trace": trace,
            },
            indent=2,
        )
    )
    sim.robot.disconnect()


if __name__ == "__main__":
    main()
