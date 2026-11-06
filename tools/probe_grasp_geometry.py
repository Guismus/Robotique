"""Isolated geometry/force experiment, NOT a pick & place acceptance test.

Known-pose fixture setup and internal state are deliberately used only here
to separate grasp geometry from camera errors. Production and the acceptance
runner use the real ROS /joint_command pipeline and image perception.
"""
from pathlib import Path
import json
import sys
import itertools
import mujoco
import numpy as np

root = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(root / "sim"), str(root / "ros2_ws/src/so101_brain")]
from so101_sim import SO101Sim, ARM_JOINTS
from so101_brain.kinematics import Kinematics, HOME, yaw_rotation

kin = Kinematics(root / "ros2_ws/src/so101_description/urdf/so101.urdf")
results = []
for offset, tangent, torque in itertools.product((0.,), (0.,), (.5, .6)):
    sim = SO101Sim(seed=0, sim_dt_per_step=0)
    sim.connect()
    model, data = sim.model, sim.data
    initial = sim.get_object_position().copy()
    rotation = data.body("pickup_object").xmat.reshape(3, 3)
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0]) + np.pi
    desired = yaw_rotation(yaw)
    target = initial + desired[:, 0] * .003 + desired[:, 1] * tangent + [0, 0, offset]
    grasp = kin.solve(target, HOME, yaw, np.radians(6), rotation=desired)
    lift = kin.solve(target + [0, 0, .075], grasp.joints, yaw, rotation=desired) if grasp else None
    path = []
    if lift:
        previous = grasp.joints
        for p in np.linspace(target, target + [0, 0, .075], 16)[1:]:
            solution = kin.solve(p, previous, yaw, rotation=desired)
            if solution is None:
                lift = None
                break
            path.append(solution.joints)
            previous = solution.joints
        if lift:
            for p in np.linspace(target + [0, 0, .075], [.05, -.25, .13], 51)[1:]:
                solution = kin.solve(p, previous, yaw, rotation=desired)
                if solution is None:
                    lift = None
                    break
                path.append(solution.joints)
                previous = solution.joints
    if not grasp or not lift:
        results.append({"offset": offset, "torque": torque, "unreachable": True})
        continue
    for name, value in zip(ARM_JOINTS, grasp.joints):
        data.qpos[model.joint(name).qposadr[0]] = value
        data.ctrl[model.actuator(name).id] = value
        sim._arm_targets[name] = value
    gaddr, gid = model.joint("gripper").qposadr[0], model.actuator("gripper").id
    data.qpos[gaddr] = data.ctrl[gid] = .8814
    sim._gripper_position_target = .8814
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    initial_penetration = max((-float(c.dist) for c in data.contact if model.body("pickup_object").id in (model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2])), default=0.)
    objid, fixed, moving = [model.body(name).id for name in ("pickup_object", "gripper", "moving_jaw_so101_v1")]
    maximum_penetration = 0.
    lifted = 0.
    contact_rows = []
    hold = False
    reference = grasp.joints.copy()
    touching = set()
    waypoint = 0
    detail = []
    for step in range(15000):
        if not hold:
            sim._gripper_position_target -= .0002
            if step > 100 and -data.actuator_force[gid] > torque and len(touching) == 2:
                hold = True
                data.ctrl[gid] = data.qpos[gaddr] - torque / 998.22
                lift_start = step
        else:
            sim.set_gripper_effort(-torque)
            if step - lift_start > 250:
                if waypoint < len(path) - 1 and np.max(np.abs(reference - path[waypoint])) < .001:
                    waypoint += 1
                reference += np.clip(path[waypoint] - reference, -.00016, .00016)
                for name, value in zip(ARM_JOINTS, reference):
                    sim._arm_targets[name] = value
        sim.step_once()
        touching = set()
        net = np.zeros(3)
        contact_detail = []
        for cid, c in enumerate(data.contact):
            b1, b2 = model.geom_bodyid[c.geom1], model.geom_bodyid[c.geom2]
            if objid in (b1, b2) and (b1 in (fixed, moving) or b2 in (fixed, moving)):
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, cid, force)
                world = c.frame.reshape(3, 3).T @ force[:3] * (1 if b2 == objid else -1)
                net += world
                contact_detail.append({"body": int(b1 if b2 == objid else b2), "world_force": world.tolist(), "friction": c.friction.tolist(), "force": force.tolist(), "point": c.pos.tolist()})
                maximum_penetration = max(maximum_penetration, -float(c.dist))
                touching.add(int(b1 if b2 == objid else b2))
                if step % 250 == 0:
                    contact_rows.append({"t": step * .002, "position": c.pos.tolist(), "normal": c.frame[:3].tolist()})
        lifted = max(lifted, float(data.body("pickup_object").xpos[2] - initial[2]))
        if step % 10 == 0:
            detail.append({"t": step * .002, "hold": hold, "cube": data.body("pickup_object").xpos.tolist(), "tcp": data.site("gripperframe").xpos.tolist(), "effort": float(data.actuator_force[gid]), "gripper": float(data.qpos[gaddr]), "net_jaw_force": net.tolist(), "contacts": contact_detail})
    result = {"offset": offset, "tangent": tangent, "torque": torque, "initial_penetration_m": initial_penetration, "lift_m": lifted, "final_height": float(data.body("pickup_object").xpos[2]),
        "max_penetration_m": maximum_penetration, "angle": float(data.qpos[gaddr]), "final_bilateral": len(touching) == 2, "contacts": contact_rows}
    results.append(result)
    (root / "artifacts/grasp_force_detail.json").write_text(json.dumps(detail, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k != "contacts"}), flush=True)
    sim.disconnect()
(root / "artifacts/grasp_geometry_probe.json").write_text(json.dumps(results, indent=2))
