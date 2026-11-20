"""Une etape a la fois : approcher, saisir, lever, transporter, relacher."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PoseArray, PointStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation
from .kinematics import Kinematics, NAMES

MAX_ESSAIS = 3
# Rotation de la base pour degager la vue camera (+90 deg). Le cube apparait entre -50 et +60 deg
# et le bac est a -79 deg : a +90 deg, le bras ne cache ni l'un ni l'autre.
PAN_OBSERVATION = 1.57

# Pile de 3 cubes : hauteurs en m. Le socle fait 2 mm, un cube 5 cm. Reglages mesures avec
# checks/stack_probe.sh (verite terrain de la sim) : 3 etages reussis sur 7 graines / 10.
SOCLE = 0.002
CUBE = 0.05
LACHER = 0.0      # on ouvre la pince quand le cube tenu est a sa hauteur exacte (1 cm plus haut : il tombe)
MARGE = 0.035     # le cube porte passe 3,5 cm au-dessus du point de lacher
REMONTEE = 0.05   # apres le lacher, la pince remonte tout droit de 5 cm avant de tourner
LIBRE = 0.08      # un cube a plus de 8 cm du socle est encore a ramasser
TOLERANCE = 0.02  # le haut de la pile doit apparaitre a moins de 2 cm de l'endroit prevu
                  # (un cube tombe contre la pile decale la tache d'au moins 2,5 cm)
# Ou est le cube dans la pince fermee, dans le repere de la pince (mesure par checks/stack_probe.py,
# identique a chaque prise) : 18 mm vers l'avant (machoires asymetriques) et 12 mm plus haut.
# Seul le decalage horizontal est compense : plus bas, les machoires touchent le cube du dessous.
DANS_PINCE = np.array([0.018, 0., 0.])


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
        self.stack = self.declare_parameter('stack', 0).value  # 0 = phase 1 (bac), 3 = empiler 3 cubes
        self.cubes = (np.zeros((0, 3)), 0.)
        self.create_subscription(PoseArray, '/objects_positions', self.objects, 10)
        self.camera = None
        self.create_subscription(PointStamped, '/perception/camera_position', self.camera_position, 1)
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
        if self.stack:
            self.stack_xy = self.seen['drop_box'][0][:2].copy()
            self.level, self.checking, self.total = 0, False, None
            self.observe()
            response.success = True
            response.message = f'Pile de {self.stack} cubes demarree.'
            return response
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


    def objects(self, msg):
        if msg.header.frame_id == 'base_link':
            points = np.array([[p.position.x, p.position.y, p.position.z] for p in msg.poses]).reshape(-1, 3)
            self.cubes = (points, time.monotonic())

    def camera_position(self, msg):
        self.camera = np.array([msg.point.x, msg.point.y, msg.point.z])

    def free_cubes(self):
        points, seen_at = self.cubes
        if time.monotonic() - seen_at > 0.5:
            return np.zeros((0, 3))
        return points[np.linalg.norm(points[:, :2] - self.stack_xy, axis=1) > LIBRE]

    def top_seen_at(self, height):
        """Ou la perception voit le cube du haut d'une pile de `height` cubes.
        Elle suppose chaque tache a 2,5 cm de haut : un cube plus haut apparait decale, loin de la camera.
        La tache comprend le dessus et les cotes du cube : son centre correspond au centre du cube."""
        top = np.r_[self.stack_xy, SOCLE + CUBE * height - CUBE / 2]
        ratio = (self.camera[2] - 0.025) / (self.camera[2] - top[2])
        return (self.camera + (top - self.camera) * ratio)[:2]

    def stack_ok(self, height):
        """Vrai si une seule tache rouge est pres du socle, et la ou doit apparaitre une pile de `height` cubes.
        Un cube tombe a cote se colle a la pile dans l'image : la tache se decale et le test echoue."""
        points, seen_at = self.cubes
        if time.monotonic() - seen_at > 0.5 or self.camera is None:
            return False
        near = points[np.linalg.norm(points[:, :2] - self.stack_xy, axis=1) <= LIBRE]
        if height == 0:
            return len(near) == 0
        return len(near) == 1 and np.linalg.norm(near[0, :2] - self.top_seen_at(height)) < TOLERANCE

    def reach(self, point, start):
        """IK pince verticale ; si c'est trop haut pour elle, on accepte de l'incliner (3e cube : 13 deg)."""
        try:
            return self.kin.solve(point, start)
        except ValueError:
            return self.kin.solve(point, start, tilt=True)

    def carry(self, point, start):
        """Angles pour que le CUBE tenu (et non la pince) soit au-dessus de point."""
        q = self.reach(point, start)
        return self.reach(point - self.kin.forward(q)[:3, :3] @ DANS_PINCE, q)

    def plan_stack(self, obj):
        """Un cycle : prendre le cube obj et le poser sur l'etage self.level de la pile."""
        center = SOCLE + CUBE / 2 + CUBE * self.level
        release = np.r_[self.stack_xy, center + LACHER]
        travel = max(0.08, release[2] + MARGE)
        above = obj.copy(); above[2] = 0.08
        high = self.kin.solve(above, self.q)
        low = self.kin.solve(obj, high)
        lifted = self.reach(np.r_[obj[:2], travel], high)
        over = self.carry(np.r_[self.stack_xy, travel], lifted)
        put = self.carry(release, over)
        up = self.reach(self.kin.forward(put)[:3, 3] + [0, 0, REMONTEE], put)
        clear = up.copy(); clear[0] = PAN_OBSERVATION
        self.pickup = obj
        self.checking = True
        self.steps = [('APPROCHE', high, 100.), ('DESCENTE', low, 100.), ('SAISIE', low, 0.),
                      ('LEVAGE', lifted, 0.), ('TRANSPORT', over, 0.), ('POSE', put, 0.),
                      ('DEPOT', put, 100.), ('RETRAIT', up, 100.), ('DEGAGEMENT', clear, 100.)]
        self.index = 0
        self.goal = None

    def verify_stack(self):
        free = self.free_cubes()
        if self.state == 'TRANSPORT':
            if (np.linalg.norm(free[:, :2] - self.pickup[:2], axis=1) < 0.03).any():
                return self.stack_retry('SAISIE_RATEE')
            return True
        if self.state != 'DEGAGEMENT':
            return True
        if self.total is None:
            self.total = len(free)
            self.get_logger().info(f'{self.total} cubes vus, socle en {np.round(self.stack_xy, 3)}')
        if self.checking:
            self.checking = False
            height = self.total - len(free)  # cubes qui ne sont plus libres = cubes sur la pile
            near = self.cubes[0][np.linalg.norm(self.cubes[0][:, :2] - self.stack_xy, axis=1) <= LIBRE]
            if self.camera is not None and height > 0:
                self.get_logger().info(f'Pile vue : {height} cubes, taches pres du socle {np.round(near[:, :2], 3).tolist()}, '
                                       f'sommet attendu {np.round(self.top_seen_at(height), 3).tolist()}')
            if height == self.level + 1 and self.stack_ok(height):
                self.level = height
                self.get_logger().info(f'PILE : {self.level}/{self.stack} cubes (essai {self.attempt}/{MAX_ESSAIS})')
                if self.level == self.stack:
                    self.steps, self.state = [], 'PILE_REUSSIE'
                    self.get_logger().info(f'PILE_REUSSIE ({self.stack} cubes)')
                    return False
                self.attempt = 1
            else:
                # Cube tombe au loin (pile intacte) : on reprend au bon etage avec le cube tombe.
                # Cube tombe contre la pile : on ne peut plus le reprendre sans la casser, on s'arrete.
                if height <= self.level and self.stack_ok(height):
                    self.level = height
                    return self.stack_retry('PAS_SUR_LA_PILE')
                self.steps, self.state = [], 'ECHEC_PILE_ABIMEE'
                self.get_logger().warn(f'ECHEC_PILE_ABIMEE : {height} cubes pres du socle, pile non reconnue')
                return False
        return self.next_cube(free)

    def next_cube(self, free):
        if len(free) == 0:
            self.steps, self.state = [], 'ECHEC_PLUS_DE_CUBE'
            self.get_logger().warn('ECHEC_PLUS_DE_CUBE : aucun cube libre vu')
            return False
        obj = free[np.argmin(np.linalg.norm(free[:, :2] - self.stack_xy, axis=1))]
        try:
            self.plan_stack(obj)
        except ValueError:
            self.steps, self.state = [], 'ECHEC_HORS_PORTEE'
            self.get_logger().warn(f'ECHEC_HORS_PORTEE : cube en {np.round(obj, 3)}')
        return False

    def stack_retry(self, raison):
        self.get_logger().warn(f'{raison} (etage {self.level + 1}, essai {self.attempt}/{MAX_ESSAIS})')
        if self.attempt >= MAX_ESSAIS:
            self.steps, self.state = [], 'ECHEC_' + raison
            return False
        self.attempt += 1
        if self.state == 'TRANSPORT':
            self.checking = False
            self.observe()
            return False
        return self.next_cube(self.free_cubes())


    def verify(self):
        """Boucle fermee : la camera confirme l'etape avant de continuer."""
        if self.stack:
            return self.verify_stack()
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
