"""Cerveau ROS : FK/IK classiques, commandes exclusivement sur /joint_command."""

import json, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from kinematics import ClassicalIK, HOME
from calibrated_planning import TARGETS, plan_cube

NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


class Brain(Node):
    def __init__(self):
        super().__init__("classic_brain")
        self.kin = ClassicalIK()
        self.position = None
        self.velocity = None
        self.effort = None
        self.simtime = 0.0
        self.joint_at = 0.0
        self.vision_at = 0.0
        self.vision = {"cubes": [], "placements": []}
        self.goal = np.r_[HOME, 0.88]
        self.force = None
        self.cancel = threading.Event()
        self.worker = None
        self.state = "ATTENTE"
        self.pub = self.create_publisher(JointState, "/joint_command", 10)
        self.status = self.create_publisher(String, "/brain_state", 10)
        self.create_subscription(JointState, "/joint_states", self.joints, 10)
        self.create_subscription(String, "/bonus/vision_state", self.seen, 1)
        self.create_service(Trigger, "/brain/start", self.start)
        self.create_service(Trigger, "/brain/stop", self.stop)
        self.create_timer(0.2, lambda: self.status.publish(String(data=self.state)))

    @property
    def q(self):
        return self.position[:5].copy()

    def joints(self, msg):
        if (
            list(msg.name) != NAMES
            or len(msg.position) != 6
            or len(msg.velocity) != 6
            or len(msg.effort) != 6
        ):
            return
        self.position = np.array(msg.position)
        self.velocity = np.array(msg.velocity)
        self.effort = np.array(msg.effort)
        self.simtime = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.joint_at = time.monotonic()

    def seen(self, msg):
        self.vision = json.loads(msg.data)
        self.vision_at = time.monotonic()

    def send(self):
        values = self.goal.copy()
        values[5] = 100 * (values[5] + 0.174533) / (1.74533 + 0.174533)
        self.pub.publish(
            JointState(
                name=NAMES,
                position=values.tolist(),
                effort=[0.0, 0.0, 0.0, 0.0, 0.0, float(self.force or 0.0)],
            )
        )

    def poll(self):
        if self.cancel.is_set():
            raise RuntimeError("ARRET_DEMANDE")
        if time.monotonic() - self.joint_at > 3:
            raise RuntimeError("ETATS_MOTEURS_PERDUS")
        time.sleep(0.005)

    def wait(self, seconds):
        until = self.simtime + seconds
        while self.simtime < until:
            self.poll()

    def move(self, q, grip=None):
        self.poll()
        deadline = self.simtime + 3 + 1.7 * np.max(np.abs(q - self.q)) / 0.08
        self.goal[:5] = q
        if grip is not None:
            self.goal[5] = grip
            self.force = None
        self.send()
        self.wait(0.04)
        while self.simtime < deadline:
            if (
                np.max(np.abs(self.q - q)) < 0.008
                and np.max(np.abs(self.velocity[:5])) < 0.05
            ):
                return
            self.poll()
        raise RuntimeError("TRAJECTOIRE_BLOQUEE")

    def close(self):
        self.force = None
        self.goal[5] = -0.174533
        self.send()
        end = self.simtime + 15
        loaded = None
        while self.simtime < end:
            self.poll()
            if self.effort[5] < -0.4:
                if loaded is None:
                    loaded = self.simtime
                if self.simtime - loaded > 0.10:
                    break
            else:
                loaded = None
        else:
            raise RuntimeError("AUCUN_EFFORT_DE_PRISE")
        self.force = -1.2
        self.send()
        start = self.simtime
        settled = None
        while self.simtime - start < 3:
            self.poll()
            if abs(self.velocity[5]) < 0.025 and self.simtime - start > 0.5:
                if settled is None:
                    settled = self.simtime
                if self.simtime - settled > 0.25:
                    if self.position[5] < 0.15:
                        raise RuntimeError("PINCE_VIDE")
                    return
            else:
                settled = None
        raise RuntimeError("SERRAGE_NON_STABILISE")

    def start(self, request, response):
        if self.worker and self.worker.is_alive():
            response.message = "Construction en cours"
            return response
        if self.position is None or time.monotonic() - self.vision_at > 3:
            response.message = "Attendre la camera et les moteurs"
            return response
        if self.state != "ATTENTE" or any(self.vision.get("placements", [])):
            response.message = (
                "Reinitialiser les cubes avec -Reset avant un nouveau cycle."
            )
            return response
        self.cancel.clear()
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()
        response.success = True
        response.message = "Pyramide classique demarree"
        return response

    def stop(self, request, response):
        self.cancel.set()
        if self.position is not None:
            self.goal[:5] = self.q
            self.send()
        response.success = True
        response.message = "Arret demande, pince maintenue"
        return response

    def run(self):
        try:
            for slot, (color, destination) in enumerate(TARGETS):
                self.state = f"CUBE_{slot+1}_PREPARATION"
                self.move(HOME, 0.88)
                self.wait(2)
                if time.monotonic() - self.vision_at > 3:
                    raise RuntimeError("VISION_PERDUE")
                selected = None
                for c in sorted(
                    self.vision["cubes"],
                    key=lambda c: np.linalg.norm(np.array(c["xyz"][:2]) - [0.22, 0.01]),
                ):
                    if c["color"] != color:
                        continue
                    try:
                        steps, _ = plan_cube(
                            self.kin, np.array(c["xyz"]), c["yaw"], destination, self.q
                        )
                        selected = c
                        break
                    except ValueError:
                        continue
                if selected is None:
                    raise RuntimeError("AUCUN_CUBE_ACCESSIBLE")
                for name, path in steps:
                    self.state = f"CUBE_{slot+1}_{name}"
                    self.get_logger().info(self.state)
                    if name == "SAISIE":
                        self.close()
                    elif name == "OUVERTURE":
                        self.move(self.q, 0.88)
                        self.wait(6)
                    else:
                        for q in path:
                            self.move(q)
                            if self.force is not None and self.position[5] < 0.15:
                                raise RuntimeError("PERTE_DE_PRISE")
                self.move(HOME, 0.88)
                self.wait(4)
                if (
                    time.monotonic() - self.vision_at > 3
                    or not self.vision["placements"][slot]
                ):
                    raise RuntimeError("DEPOT_NON_CONFIRME")
            self.state = "PYRAMIDE_A_VERIFIER_PHYSIQUEMENT"
        except (RuntimeError, ValueError) as error:
            self.state = f"ECHEC_{error}"
            self.get_logger().error(self.state)
            if self.position is not None:
                self.goal[:5] = self.q
                self.send()


def main():
    rclpy.init()
    node = Brain()
    try:
        rclpy.spin(node)
    finally:
        node.cancel.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
