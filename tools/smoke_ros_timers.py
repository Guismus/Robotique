"""Bounded simulation smoke test with unmodified ROS timers and backend factory.

This complements the deterministic evaluator; it is not a 20-seed acceptance
campaign. No hardware connection is permitted.
"""
import argparse
import json
from pathlib import Path
import sys
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

ROOT = Path(__file__).resolve().parents[1]
for package in ("so101_brain", "so101_driver", "so101_perception"):
    sys.path.insert(0, str(ROOT / "ros2_ws/src" / package))
from so101_brain.brain_node import BrainNode
from so101_driver.driver_node import DriverNode
from so101_perception.perception_node import PerceptionNode
import so101_sim


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=120.)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/ros_timers.json")
    args = parser.parse_args()
    rclpy.init(args=["--ros-args", "-p", "use_sim:=true", "-p", f"sim_seed:={args.seed}"])
    driver = DriverNode()
    assert driver._use_sim is True
    perception, brain = PerceptionNode(), BrainNode()
    nodes = (driver, perception, brain)
    executor = MultiThreadedExecutor(num_threads=3)
    for node in nodes:
        executor.add_node(node)
    started, previous = time.monotonic(), None
    history = []
    try:
        while time.monotonic() - started < args.seconds:
            executor.spin_once(timeout_sec=.02)
            state = brain._state.name
            if state != previous:
                row = {"wall_seconds": time.monotonic() - started, "state": state,
                    "failure": brain._failure_reason}
                history.append(row)
                print(json.dumps(row), flush=True)
                previous = state
            if state in ("FINISHED", "FAILED"):
                break
        result = {"state": brain._state.name, "failure": brain._failure_reason,
            "wall_seconds": time.monotonic() - started,
            "backend_module": so101_sim.__file__, "use_sim": driver._use_sim,
            "history": history,
            "scope": "Normal ROS timers smoke only; physical success requires the deterministic evaluator"}
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return 0 if result["state"] == "FINISHED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
