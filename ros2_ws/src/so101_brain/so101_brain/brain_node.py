"""Une etape a la fois : approcher, saisir, lever, transporter, relacher."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation
from .kinematics import Kinematics, NAMES

MAX_ESSAIS = 3
# Rotation de la base pour degager la vue camera (+90 deg). Le cube apparait entre -50 et +60 deg
# et le bac est a -79 deg : a +90 deg, le bras ne cache ni l'un ni l'autre.
PAN_OBSERVATION = 1.57


class BrainNode(Node):
    def __init__(self):
        super().__init__('so101_brain')
        path = self.declare_parameter('urdf', '/opt/so101/sim/so101_sim/assets/so101/so101_new_calib.urdf').value
        self.kin = Kinematics(path)
        self.q = None
        self.seen = {}
        self.steps = []
        self.index = 0
        self.state = 'ATTENTE'
        self.goal = None
        self.joint_time = 0.
        self.commands = self.create_publisher(JointState, '/joint_command', 10)
        self.states = self.create_publisher(String, '/brain_state', 10)
        self.tcp = self.create_publisher(PoseStamped, '/end_effector_pose', 10)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        for name in ['object', 'drop_box']:
            self.create_subscription(PoseStamped, '/' + name + '_position', lambda msg, n=name: self.position(n, msg), 10)
        self.create_service(Trigger, '/brain/start', self.start)
        self.create_service(Trigger, '/brain/stop', self.stop)
        self.create_timer(0.05, self.tick)

    def joints(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(n in values for n in NAMES):
            self.q = np.array([values[n] for n in NAMES])
            self.joint_time = time.monotonic()
            pose = self.kin.forward(self.q)
            result = PoseStamped()
            result.header.frame_id = 'base_link'
            result.header.stamp = self.get_clock().now().to_msg()
            result.pose.position.x, result.pose.position.y, result.pose.position.z = map(float, pose[:3, 3])
            quat = Rotation.from_matrix(pose[:3, :3]).as_quat()
            result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z, result.pose.orientation.w = map(float, quat)
            self.tcp.publish(result)

    def position(self, name, msg):
        p = msg.pose.position
        age = (self.get_clock().now().nanoseconds - (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)) / 1e9
        if msg.header.frame_id == 'base_link' and 0 <= age < 1 and np.isfinite([p.x, p.y, p.z]).all():
            self.seen[name] = (np.array([p.x, p.y, p.z]), time.monotonic())

    def start(self, request, response):
        if self.steps:
            response.message = 'Un cycle est deja en cours.'
            return response
        if self.q is None or time.monotonic() - self.joint_time > 1 or 'drop_box' not in self.seen or time.monotonic() - self.seen['drop_box'][1] > 1:
            response.message = 'Attendre les moteurs et le bac visibles.'
            return response
        self.attempt = 1
        if 'object' not in self.seen or time.monotonic() - self.seen['object'][1] > 1:
            self.observe()
            response.success = True
            response.message = 'Cube cache : le bras se degage pour regarder.'
            return response
        try:
            self.plan()
        except ValueError as error:
            response.message = str(error)
            return response
        response.success = True
        response.message = 'Cycle demarre.'
        return response

    def observe(self):
        q = self.q.copy()
        q[0] = PAN_OBSERVATION
        self.steps = [('DEGAGEMENT', q, 100.)]
        self.index = 0
        self.goal = None

    def plan(self):
        obj = self.seen['object'][0].copy()
        box = self.seen['drop_box'][0].copy()
        above = obj.copy(); above[2] = 0.08
        drop = box.copy(); drop[2] = 0.08
        # Calculer toutes les cibles avant de commencer a bouger (ValueError si inaccessible).
        high = self.kin.solve(above, self.q)
        low = self.kin.solve(obj, high)
        destination = self.kin.solve(drop, high)
        self.pickup, self.box = obj, box
        self.steps = [('APPROCHE', high, 100.), ('DESCENTE', low, 100.),
                      ('SAISIE', low, 0.), ('LEVAGE', high, 0.),
                      ('TRANSPORT', destination, 0.), ('DEPOT', destination, 100.),
                      ('RETRAIT', high, 100.)]
        self.index = 0
        self.goal = None

    def fresh_cube(self):
        if 'object' in self.seen and time.monotonic() - self.seen['object'][1] < 0.5:
            return self.seen['object'][0]
        return None

    def verify(self):
        """Boucle fermee : la camera confirme l'etape avant de continuer."""
        cube = self.fresh_cube()
        if self.state == 'TRANSPORT':
            if cube is not None and np.linalg.norm(cube[:2] - self.pickup[:2]) < 0.03:
                return self.retry('SAISIE_RATEE')  # cube encore a moins de 3 cm de sa place de depart
        elif self.state == 'RETRAIT':
            if cube is not None and np.linalg.norm(cube[:2] - self.box[:2]) < 0.04:
                self.steps, self.state = [], 'CYCLE_REUSSI'  # a moins de 4 cm du centre du bac
                self.get_logger().info(f'CYCLE_REUSSI (essai {self.attempt}/{MAX_ESSAIS})')
                return False
            return self.retry('PAS_DANS_LE_BAC')
        elif self.state == 'DEGAGEMENT':
            if cube is None:
                self.steps, self.state = [], 'ECHEC_CUBE_INVISIBLE'
                self.get_logger().warn('ECHEC_CUBE_INVISIBLE : cube toujours pas vu apres degagement')
                return False
            try:
                self.plan()
            except ValueError:
                self.steps, self.state = [], 'ECHEC_HORS_PORTEE'
            return False
        return True

    def retry(self, raison):
        self.get_logger().warn(f'{raison} (essai {self.attempt}/{MAX_ESSAIS})')
        if self.attempt >= MAX_ESSAIS:
            self.steps, self.state = [], 'ECHEC_' + raison
            return False
        self.attempt += 1
        if self.fresh_cube() is None:
            self.observe()
            return False
        try:
            self.plan()
        except ValueError:
            self.steps, self.state = [], 'ECHEC_HORS_PORTEE'
        return False

    def stop(self, request, response):
        self.steps = []
        self.state = 'ARRET'
        # Tenir la position mesuree sans ouvrir la pince.
        if self.q is not None:
            self.commands.publish(JointState(name=NAMES, position=self.q.tolist()))
        response.success = True
        response.message = 'Mouvement interrompu, position maintenue.'
        return response

    def tick(self):
        self.states.publish(String(data=self.state))
        if not self.steps:
            return
        now = time.monotonic()
        if now - self.joint_time > 1:
            self.stop(None, Trigger.Response())
            self.state = 'ERREUR_MOTEURS'
            return
        if self.goal is None:
            self.state, self.goal, self.grip = self.steps[self.index]
            self.started = now
            self.sent = self.q.copy()
            self.get_logger().info(self.state)
        # Limiter chaque increment pour eviter un saut brutal.
        self.sent += np.clip(self.goal - self.sent, -0.025, 0.025)
        self.commands.publish(JointState(name=NAMES + ['gripper'], position=self.sent.tolist() + [self.grip]))
        reached = np.max(np.abs(self.q - self.goal)) < 0.04
        if reached and now - self.started > 2.:
            if not self.verify():
                return
            self.index += 1
            self.goal = None
            if self.index == len(self.steps):
                self.steps = []
                self.state = 'CYCLE_TERMINE_A_VERIFIER'
        elif now - self.started > 15:
            self.stop(None, Trigger.Response())
            self.state = 'ERREUR_DELAI'


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
