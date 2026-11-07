"""Simulation smoke with the three ROS nodes in separate processes.

The deterministic evaluator is still required for physical acceptance. This
test exercises normal timers, the unmodified backend factory and DDS delivery.
"""
import argparse
import json
import math
import multiprocessing as mp
from pathlib import Path
from queue import Empty
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
for package in ("so101_brain", "so101_driver", "so101_perception"):
    sys.path.insert(0, str(ROOT / "ros2_ws/src" / package))


def run_node(kind, seed, stop, queue):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
    from so101_brain.brain_node import BrainNode
    from so101_driver.driver_node import DriverNode
    from so101_perception.perception_node import PerceptionNode
    import so101_sim
    timings = {}
    owner = DriverNode if kind == "driver" else BrainNode
    names = ("_control_step", "_publish_joint_states", "_publish_camera") if kind == "driver" else ("_control_loop", "_cb_joint_states", "_cb_object_pos", "_transition", "_send_joint_command")
    if kind in ("driver", "brain"):
        for name in names:
            original = getattr(owner, name)
            def timed(self, *arguments, function=original, label=name):
                start = time.monotonic()
                record = timings.setdefault(label, {"count": 0, "max_seconds": 0., "total_seconds": 0., "max_gap_seconds": 0., "previous": start})
                record["max_gap_seconds"] = max(record["max_gap_seconds"], start - record["previous"])
                record["previous"] = start
                if label == "_cb_joint_states":
                    age = self._now() - self._stamp(arguments[0])
                    record["max_source_age_seconds"] = max(record.get("max_source_age_seconds", 0.), age)
                try:
                    return function(self, *arguments)
                finally:
                    duration = time.monotonic() - start
                    record["count"] += 1
                    record["max_seconds"] = max(record["max_seconds"], duration)
                    record["total_seconds"] += duration
            setattr(owner, name, timed)
    rclpy.init(args=["--ros-args", "-p", "use_sim:=true", "-p", "use_sim_time:=true", "-p", f"sim_seed:={seed}"])
    node = {"driver": DriverNode, "perception": PerceptionNode, "brain": BrainNode}[kind]()
    if kind == "driver":
        assert node._use_sim is True
    previous = None
    executor = MultiThreadedExecutor(num_threads=2) if kind == "driver" else SingleThreadedExecutor()
    executor.add_node(node)
    queue.put({"node": kind, "ready": True, "backend_module": so101_sim.__file__})
    try:
        while not stop.is_set():
            executor.spin_once(timeout_sec=.02)
            if kind == "brain" and node._state.name != previous:
                previous = node._state.name
                joint_age = node._now() - node._joint_stamp
                queue.put({"state": previous, "failure": node._failure_reason,
                    "joint_age_seconds": joint_age if math.isfinite(joint_age) else None})
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        queue.put({"node": kind, "callback_timings": timings})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=120.)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/ros_processes.json")
    args = parser.parse_args()
    context = mp.get_context("spawn")
    stop, queue = context.Event(), context.Queue()
    children, history = [], []
    started = time.monotonic()
    terminal = None
    try:
        for kind in ("driver", "perception", "brain"):
            process = context.Process(target=run_node, args=(kind, args.seed, stop, queue), name=kind)
            process.start()
            children.append(process)
            # Match the documented driver -> perception -> brain launch order.
            while time.monotonic() - started < args.seconds:
                try:
                    row = queue.get(timeout=.1)
                except Empty:
                    if process.exitcode is not None:
                        raise RuntimeError(f"{kind} exited during initialization: {process.exitcode}")
                    continue
                row["wall_seconds"] = time.monotonic() - started
                history.append(row)
                print(json.dumps(row), flush=True)
                if row.get("node") == kind and row.get("ready"):
                    break
        while time.monotonic() - started < args.seconds:
            if any(child.exitcode is not None for child in children):
                terminal = "PROCESS_EXIT"
                break
            try:
                row = queue.get(timeout=.1)
            except Empty:
                continue
            row["wall_seconds"] = time.monotonic() - started
            history.append(row)
            print(json.dumps(row), flush=True)
            if row.get("state") in ("FINISHED", "FAILED"):
                terminal = row["state"]
                break
    finally:
        stop.set()
        for child in children:
            child.join(timeout=5.)
            if child.is_alive():
                child.terminate()
                child.join()
        while True:
            try:
                history.append(queue.get(timeout=.1))
            except Empty:
                break
    result = {"state": terminal or "TIMEOUT", "history": history,
        "wall_seconds": time.monotonic() - started,
        "child_exitcodes": {child.name: child.exitcode for child in children},
        "scope": "Separate-process normal ROS timers smoke; not physical acceptance"}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return 0 if result["state"] == "FINISHED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
