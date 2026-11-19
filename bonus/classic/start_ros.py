"""Attendre les capteurs avant de demander le demarrage au cerveau ROS."""

import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = Node("start_classic")
    client = node.create_client(Trigger, "/brain/start")
    end = time.monotonic() + 60
    try:
        while time.monotonic() < end:
            if not client.wait_for_service(timeout_sec=1):
                continue
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done():
                continue
            response = future.result()
            print(response.message, flush=True)
            if response.success or response.message == "Construction en cours":
                return 0
            if "Attendre" not in response.message:
                return 2
            time.sleep(1)
        print("Les capteurs ne sont pas prets apres une minute.", flush=True)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
