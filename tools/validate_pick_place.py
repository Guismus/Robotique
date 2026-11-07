"""Seeded ROS-topic integration test. Ground truth is used only by this evaluator.

Run inside the control image after colcon build; see docs/GRASP_VALIDATION.md.
The driver, perception and brain are the actual ROS nodes. Their timers are
cancelled and advanced on a simulation clock to remove wall-time scheduling.
"""
import argparse
from concurrent.futures import wait
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

import cv2
import mujoco
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[1]
for package in ("so101_brain", "so101_driver", "so101_perception"):
    sys.path.insert(0, str(ROOT / "ros2_ws/src" / package))
sys.path.insert(0, str(ROOT / "sim"))
from so101_driver.driver_node import DriverNode
from so101_sim import SO101Sim, ARM_JOINTS


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def baseline_scene():
    baseline = ROOT / "artifacts/baseline"
    robot_xml = (baseline / "so101_tek5.xml").read_text().replace('meshdir="assets"', f'meshdir="{ROOT / "sim/so101_sim/assets/so101/assets"}"')
    (baseline / "robot_resolved.xml").write_text(robot_xml)
    scene = baseline / "scene_resolved.xml"
    scene.write_text((baseline / "scene_tek5.xml").read_text().replace('file="so101_tek5.xml"', 'file="robot_resolved.xml"'))
    return scene


def source_hashes(baseline=False):
    paths = ["ros2_ws/src/so101_brain/so101_brain/brain_node.py", "ros2_ws/src/so101_brain/so101_brain/kinematics.py",
        "ros2_ws/src/so101_perception/so101_perception/perception_node.py", "ros2_ws/src/so101_perception/so101_perception/vision.py",
        "ros2_ws/src/so101_perception/so101_perception/occlusion.py", "ros2_ws/src/so101_perception/so101_perception/cube_hull.json",
        "ros2_ws/src/so101_perception/so101_perception/gripper_visual.json", "ros2_ws/src/so101_driver/so101_driver/driver_node.py",
        "ros2_ws/src/so101_description/urdf/so101.urdf", "sim/so101_sim/__init__.py", "sim/so101_sim/assets/so101/scene_tek5.xml",
        "sim/so101_sim/assets/so101/so101_tek5.xml", "sim/so101_sim/assets/so101/assets/moving_jaw_collision_manifest.json"]
    if baseline:
        paths = [f"artifacts/baseline/{name}" for name in ("brain_node.py", "perception_node.py", "driver_node.py", "sim.py", "so101.urdf", "scene_tek5.xml", "so101_tek5.xml")]
    return {path: hashlib.sha256((ROOT / path).read_bytes()).hexdigest() for path in paths}


def run(seed, args):
    brain_path = ROOT / ("artifacts/baseline/brain_node.py" if args.baseline else "ros2_ws/src/so101_brain/so101_brain/brain_node.py")
    perception_path = ROOT / ("artifacts/baseline/perception_node.py" if args.baseline else "ros2_ws/src/so101_perception/so101_perception/perception_node.py")
    brain_module = load(brain_path, "baseline_brain" if args.baseline else "so101_brain.brain_node")
    perception_module = load(perception_path, "baseline_perception" if args.baseline else "so101_perception.perception_node")
    SimClass = load(ROOT / "artifacts/baseline/sim.py", "baseline_sim").SO101Sim if args.baseline else SO101Sim
    DriverClass = load(ROOT / "artifacts/baseline/driver_node.py", "baseline_driver").DriverNode if args.baseline else DriverNode
    brain_module.URDF_PATHS = [ROOT / ("artifacts/baseline/so101.urdf" if args.baseline else "ros2_ws/src/so101_description/urdf/so101.urdf")]
    scene = ROOT / "sim/so101_sim/assets/so101/scene_tek5.xml"
    if args.baseline:
        scene = baseline_scene()
    sim = SimClass(scene_path=scene, seed=seed, sim_dt_per_step=0)
    sim.connect()
    if not args.baseline:
        # The evaluator advances every physics substep to inspect contacts.
        sim.advance = lambda seconds: None
    def connect_sim(node, port):
        assert node._use_sim is True, "This evaluator forbids physical hardware"
        return sim
    original_connect = DriverClass._connect_arm
    DriverClass._connect_arm = connect_sim
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true", "-p", "publish_sim_clock:=false", "--log-level", "warn"])
    driver = DriverClass()
    DriverClass._connect_arm = original_connect
    perception = perception_module.PerceptionNode()
    brain = brain_module.BrainNode()
    if args.baseline:
        original_ik = brain._solve_ik
        def recorded_ik(target, *arguments, **keywords):
            solution = original_ik(target, *arguments, **keywords)
            pose = brain._forward_kinematics(solution) if solution is not None else None
            brain._ik_diagnostics = {"target": list(target), "position_error_m": float(np.linalg.norm(pose[:3, 3] - target)) if pose is not None else None,
                "orientation_error_rad": float(np.arccos(np.clip(-pose[2, 2], -1, 1))) if pose is not None else None}
            return solution
        brain._solve_ik = recorded_ik
    observer = Node("grasp_evaluator")
    clock_pub = observer.create_publisher(Clock, "/clock", 10)
    command_messages = []
    observer.create_subscription(JointState, "/joint_command", lambda msg: command_messages.append(msg), 10)
    nodes = (driver, perception, brain, observer)
    executor = SingleThreadedExecutor()
    for node in nodes:
        for timer in node.timers:
            timer.cancel()
        executor.add_node(node)
    def pump(count=30):
        for _ in range(count):
            executor.spin_once(timeout_sec=.0001)
    pump(200)
    initial = sim.get_object_position().copy()
    obj_id = sim.model.body("pickup_object").id
    fixed_id = sim.model.body("gripper").id
    moving_id = sim.model.body("moving_jaw_so101_v1").id
    out = args.output / f"seed_{seed:03d}"
    out.mkdir(parents=True, exist_ok=True)
    trace = []
    detected = False
    lifted = False
    contacts_seen = set()
    bilateral_lift_steps = 0
    transport_heights = []
    max_penetration = 0.0
    max_floor_penetration = 0.0
    max_self_penetration = 0.0
    collision_peaks = {}
    phase_penetrations = {}
    robot_ids = {sim.model.body(name).id for name in ("shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1")}
    fixed_visual = next(i for i in range(sim.model.ngeom) if sim.model.geom_bodyid[i] == fixed_id and sim.model.geom_dataid[i] == sim.model.mesh("wrist_roll_follower_so101_v1").id)
    moving_visual = next(i for i in range(sim.model.ngeom) if sim.model.geom_bodyid[i] == moving_id and sim.model.geom_dataid[i] == sim.model.mesh("moving_jaw_so101_v1").id)
    def jaw_aperture():
        body = sim.data.body("gripper")
        rotation = body.xmat.reshape(3, 3)
        origin = body.xpos + rotation @ np.array([-.0077, -.000218121, -.0901274])
        left = mujoco.mj_rayMesh(sim.model, sim.data, fixed_visual, origin, -rotation[:, 0])
        right = mujoco.mj_rayMesh(sim.model, sim.data, moving_visual, origin, rotation[:, 0])
        return float(left + right) if left >= 0 and right >= 0 else None
    video = None
    if args.video:
        video = cv2.VideoWriter(str(out / "camera.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (320, 240))
    try:
        for tick in range(int(args.seconds / .05)):
            t = (tick + 1) * .05
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = int(t), int(round((t % 1) * 1e9))
            if clock.clock.nanosec == 1000000000:
                clock.clock.sec += 1
                clock.clock.nanosec = 0
            clock_pub.publish(clock)
            pump()
            driver._publish_joint_states()
            pump()
            if tick % 2 == 0:
                driver._publish_camera()
                pump()
                perception._publish_positions()
                pump()
                if video:
                    video.write(cv2.cvtColor(sim.render_camera(), cv2.COLOR_RGB2BGR))
                if tick in (0, 40, 100, 160, 240, 400, 600):
                    cv2.imwrite(str(out / f"camera_{tick:04d}.png"), cv2.cvtColor(sim.render_camera(), cv2.COLOR_RGB2BGR))
            brain._control_loop()
            if brain._state.name == "PLANNING":
                # Planner CPU time is not simulated plant time. Production
                # keeps publishing Home while its independent worker runs;
                # this deterministic evaluator holds the plant clock here.
                wait([brain._planning_future], timeout=30.)
                if not brain._planning_future.done():
                    raise TimeoutError("Preflight did not finish within 30 seconds of wall time")
                brain._control_loop()
            pump()
            driver._control_step()
            pair_contacts = []
            for _ in range(round(.05 / sim.model.opt.timestep)):
                if args.baseline:
                    mujoco.mj_step(sim.model, sim.data)
                else:
                    sim.step_once()
                touching = set()
                if brain._state.name == "TRANSPORT":
                    transport_heights.append(float(sim.data.body("pickup_object").xpos[2]))
                for contact_id, contact in enumerate(sim.data.contact):
                    b1, b2 = sim.model.geom_bodyid[contact.geom1], sim.model.geom_bodyid[contact.geom2]
                    if obj_id in (b1, b2):
                        other = b2 if b1 == obj_id else b1
                        if other in (fixed_id, moving_id):
                            contacts_seen.add(int(other))
                            touching.add(int(other))
                            force = np.zeros(6)
                            mujoco.mj_contactForce(sim.model, sim.data, contact_id, force)
                            pair_contacts.append({"body": sim.model.body(int(other)).name, "distance": float(contact.dist), "normal_force_n": float(force[0]), "position": contact.pos.tolist(), "normal": contact.frame[:3].tolist()})
                        max_penetration = max(max_penetration, -float(contact.dist))
                    if 0 in (b1, b2) and (b1 in robot_ids or b2 in robot_ids):
                        max_floor_penetration = max(max_floor_penetration, -float(contact.dist))
                    if b1 in robot_ids and b2 in robot_ids:
                        max_self_penetration = max(max_self_penetration, -float(contact.dist))
                    penetration = -float(contact.dist)
                    relevant = obj_id in (b1, b2) or b1 in robot_ids or b2 in robot_ids
                    if relevant:
                        phase = brain._state.name
                        phase_penetrations[phase] = max(phase_penetrations.get(phase, 0.), penetration)
                        key = " / ".join(sorted([sim.model.body(int(b1)).name, sim.model.body(int(b2)).name]))
                        if key not in collision_peaks or penetration > collision_peaks[key]["penetration_m"]:
                            collision_peaks[key] = {"t": t, "phase": phase, "penetration_m": penetration}
                if len(touching) == 2 and sim.data.body("pickup_object").xpos[2] >= initial[2] + .05:
                    bilateral_lift_steps += 1
            q = [float(sim.data.qpos[sim.model.joint(n).qposadr[0]]) for n in ARM_JOINTS]
            mujoco.mj_forward(sim.model, sim.data)
            fk = brain._forward_kinematics(q)
            obj = sim.get_object_position()
            estimate = getattr(brain, "_target_object_pos", None)
            detected |= estimate is not None
            lifted |= obj[2] >= initial[2] + .05
            state = brain._state.name
            row = {"t": t, "state": state, "cube": obj.tolist(), "estimate": estimate.tolist() if estimate is not None else None,
                "tcp_fk": fk[:3, 3].tolist(), "tcp_mj": sim.data.site("gripperframe").xpos.tolist(),
                "cube_quaternion_wxyz": sim.data.body("pickup_object").xquat.tolist(), "tcp_rotation": fk[:3, :3].tolist(),
                "q": q, "command": brain._cmd_q.tolist() if brain._cmd_q is not None else None,
                "sent_command": brain._sent_q.tolist() if getattr(brain, "_sent_q", None) is not None else None,
                "gripper_rad": float(sim.data.qpos[sim.model.joint("gripper").qposadr[0]]),
                "jaw_aperture_m_at_grasp_plane": jaw_aperture(),
                "object_observation_stamp": float(brain._object_stamp) if np.isfinite(getattr(brain, "_object_stamp", -np.inf)) else None,
                "gripper_command_pct": brain._cmd_gripper, "gripper_effort_nm": getattr(brain, "_gripper_effort", None),
                "contacts": pair_contacts[-10:], "ik": getattr(brain, "_ik_diagnostics", None),
                "failure": getattr(brain, "_failure_reason", None)}
            row["motion_timeout_seconds"] = getattr(brain, "_state_timeout", None)
            if not trace or state != trace[-1]["state"]:
                print(json.dumps({"seed": seed, "t": t, "state": state, "cube": obj.tolist(), "estimate": row["estimate"], "failure": row["failure"]}), flush=True)
            trace.append(row)
            if tick % 20 == 0:
                (out / "progress.json").write_text(json.dumps(row))
            if state in ("FINISHED", "FAILED") and tick > 20:
                # Allow a released object to settle before measuring delivery.
                if state == "FINISHED":
                    for _ in range(round(1.0 / sim.model.opt.timestep)):
                        mujoco.mj_step(sim.model, sim.data)
                break
        obj = sim.get_object_position()
        cv2.imwrite(str(out / "final_camera.png"), cv2.cvtColor(sim.render_camera(), cv2.COLOR_RGB2BGR))
        rot = sim.data.body("pickup_object").xmat.reshape(3, 3)
        half = np.abs(rot) @ np.array([.025, .025, .025])
        box = sim.get_drop_box_position()
        inside = bool(np.all(np.abs(obj[:2] - box[:2]) + half[:2] <= .049) and .023 <= obj[2] <= .045)
        held = bool(transport_heights and min(transport_heights) >= initial[2] + .05)
        physical = lifted and held and bilateral_lift_steps * sim.model.opt.timestep >= .2
        collision_ok = max(phase_penetrations.values(), default=0.) < .003
        result = {"seed": seed, "initial": initial.tolist(), "final": obj.tolist(), "detected": bool(detected),
            "bilateral_contacts_seen": len(contacts_seen) == 2, "lifted_5cm": bool(lifted), "held_transport": held,
            "physical_grasp": bool(physical), "inside_box": inside, "collision_ok": collision_ok,
            "bilateral_lift_contact_seconds": bilateral_lift_steps * sim.model.opt.timestep,
            "max_cube_penetration_m": max_penetration, "max_finger_floor_penetration_m": max_floor_penetration,
            "max_robot_self_penetration_m": max_self_penetration, "phase_penetrations": phase_penetrations, "collision_peaks": collision_peaks,
            "minimum_transport_height_m": min(transport_heights) if transport_heights else None,
            "maximum_lift_m": max(row["cube"][2] for row in trace) - initial[2],
            "initial_detection_error_m": next((float(np.linalg.norm(np.array(row["estimate"]) - row["cube"])) for row in trace if row["estimate"] is not None), None),
            "max_fk_mujoco_error_m": max(float(np.linalg.norm(np.array(row["tcp_fk"]) - row["tcp_mj"])) for row in trace),
            "state": brain._state.name, "failure": getattr(brain, "_failure_reason", None),
            "success": bool(detected and physical and inside and collision_ok), "joint_command_messages": len(command_messages)}
        (out / "trace.json").write_text(json.dumps(trace, indent=2))
        (out / "result.json").write_text(json.dumps(result, indent=2))
        return result
    finally:
        if video:
            video.release()
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(20)))
    parser.add_argument("--seconds", type=float, default=180)
    parser.add_argument("--baseline", action="store_true")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/validation")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    hashes = source_hashes(args.baseline)
    results = []
    for seed in args.seeds:
        result = run(seed, args)
        results.append(result)
        print(json.dumps(result), flush=True)
    summary = {"results": results, "successes": sum(r["success"] for r in results), "trials": len(results),
        "success_rate": sum(r["success"] for r in results) / len(results), "mujoco": mujoco.__version__, "baseline": args.baseline,
        "source_hashes": hashes, "sources_unchanged": source_hashes(args.baseline) == hashes,
        "numpy": np.__version__, "opencv": cv2.__version__, "dt": .002,
        "criteria": {"lift_m": .05, "max_penetration_m": .003, "bilateral_lift_contact_seconds": .2}}
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2))
    return (0 if summary["success_rate"] >= .9 else 1) if summary["sources_unchanged"] else 2


if __name__ == "__main__":
    sys.exit(main())
