"""Measured grasp frame and residual-checked, bounded inverse kinematics."""
from dataclasses import dataclass
import ikpy.chain
import numpy as np
from scipy.optimize import least_squares

ARM_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER_LOWER, GRIPPER_UPPER = -.174533, 1.74533
FIXED_JAW_CLEARANCE = .003
HOME = np.array([0., -1., .4, .4, 0.])


def gripper_percent(angle):
    return float(np.clip((angle - GRIPPER_LOWER) / (GRIPPER_UPPER - GRIPPER_LOWER) * 100, 0, 100))


def yaw_rotation(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])


@dataclass
class IKSolution:
    joints: np.ndarray
    position_error: float
    orientation_error: float
    target: np.ndarray
    yaw: float

    def diagnostics(self):
        return {"target": self.target.tolist(), "yaw": self.yaw,
            "position_error_m": self.position_error, "orientation_error_rad": self.orientation_error}


class Kinematics:
    def __init__(self, urdf):
        self.chain = ikpy.chain.Chain.from_urdf_file(str(urdf),
            base_elements=["base_link", "shoulder_pan", "shoulder_link", "shoulder_lift", "upper_arm_link",
                "elbow_flex", "lower_arm_link", "wrist_flex", "wrist_link", "wrist_roll", "gripper_link", "gripper_frame_joint", "gripper_frame_link"],
            active_links_mask=[False, True, True, True, True, True, False])
        self.bounds = np.array([link.bounds for link in self.chain.links[1:6]]).T

    def forward(self, joints):
        return self.chain.forward_kinematics([0, *joints, 0])

    def solve(self, target, initial, yaw, max_orientation=np.radians(28), rotation=None, quick=False):
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            return None
        desired = yaw_rotation(yaw) if rotation is None else np.asarray(rotation, dtype=float)
        def residual(q):
            pose = self.forward(q)
            return np.r_[(pose[:3, 3] - target) * 500., (pose[:3, :3] - desired).ravel() * 2.]
        starts = (initial, [initial[0], .4, -.3, 1.4, initial[4]], [initial[0], -.5, .8, 1., initial[4]])
        accepted = []
        for start in starts:
            try:
                fit = least_squares(residual, np.clip(start, *self.bounds), bounds=self.bounds, max_nfev=100)
                fk = self.forward(fit.x)
                position_error = float(np.linalg.norm(fk[:3, 3] - target))
                angle = float(np.arccos(np.clip((np.trace(desired.T @ fk[:3, :3]) - 1) / 2, -1, 1)))
                if np.all(np.isfinite(fit.x)) and position_error <= .002 and angle <= max_orientation:
                    accepted.append(IKSolution(fit.x, position_error, angle, target.copy(), float(yaw)))
                    if quick:
                        return accepted[-1]
            except (ValueError, FloatingPointError):
                continue
        return min(accepted, key=lambda s: s.orientation_error + .02 * np.linalg.norm(s.joints - initial)) if accepted else None


SHOULDER_XY = np.array([.0388353, 0.])


def cartesian_segment(start, end):
    count = max(1, int(np.ceil(np.linalg.norm(np.asarray(end) - start) / .005)))
    return list(np.linspace(start, end, count + 1)[1:])


def transport_arc(start, end):
    """Stay in the reachable annulus instead of crossing near the base."""
    start, end = np.asarray(start), np.asarray(end)
    a, b = start[:2] - SHOULDER_XY, end[:2] - SHOULDER_XY
    radius_a, radius_b = np.linalg.norm(a), np.linalg.norm(b)
    angle = np.arctan2(a[1], a[0])
    delta = (np.arctan2(b[1], b[0]) - angle + np.pi) % (2 * np.pi) - np.pi
    length_bound = max(radius_a, radius_b) * abs(delta) + abs(radius_b - radius_a) + abs(end[2] - start[2])
    count = max(1, int(np.ceil(length_bound / .005)))
    points = []
    for fraction in np.linspace(0, 1, count + 1)[1:]:
        r = radius_a + fraction * (radius_b - radius_a)
        theta = angle + fraction * delta
        points.append(np.r_[SHOULDER_XY + r * np.array([np.cos(theta), np.sin(theta)]), start[2] + fraction * (end[2] - start[2])])
    points[-1] = end.copy()
    return points


@dataclass
class MotionPlan:
    solutions: list[IKSolution]


@dataclass
class PickPlan:
    origin: np.ndarray
    box: np.ndarray
    grasp_target: np.ndarray
    yaw: float
    approach: MotionPlan
    lower: MotionPlan
    lift: MotionPlan
    transport: MotionPlan


def plan_pick(urdf, origin, box, cube_yaw, initial, cancel=None):
    """Pure preflight worker with its own chain; never reads/writes ROS state."""
    kin = Kinematics(urdf)
    def path(points, seed, yaw, final_tolerance=np.radians(28)):
        solutions = []
        for i, point in enumerate(points):
            if cancel is not None and cancel.is_set():
                return None
            solution = kin.solve(point, seed, yaw, final_tolerance if i == len(points) - 1 else np.radians(28), quick=True)
            if solution is None or np.max(np.abs(solution.joints - seed)) > .5:
                return None
            solutions.append(solution)
            seed = solution.joints
        return MotionPlan(solutions)
    candidates = []
    for yaw in cube_yaw + np.arange(4) * np.pi / 2:
        target = np.asarray(origin) + yaw_rotation(yaw)[:, 0] * FIXED_JAW_CLEARANCE
        grasp = kin.solve(target, initial, yaw, np.radians(6), quick=True)
        if grasp:
            candidates.append((float(np.linalg.norm(grasp.joints - initial)), float(yaw), target))
    for _, yaw, target in sorted(candidates, key=lambda item: item[0]):
        if cancel is not None and cancel.is_set():
            return None
        approach = kin.solve(target + [0, 0, .045], initial, yaw, quick=True)
        if approach is None:
            continue
        lower = path(cartesian_segment(approach.target, target), approach.joints, yaw, np.radians(6))
        if lower is None:
            continue
        radial = target[:2] - SHOULDER_XY
        radius = np.linalg.norm(radial)
        for inset in (0., .02, .04, .06):
            if radius - inset < .14:
                continue
            lift_target = target + [0, 0, .10]
            lift_target[:2] = SHOULDER_XY + radial * ((radius - inset) / radius)
            lift = path(cartesian_segment(target, lift_target), lower.solutions[-1].joints, yaw)
            if lift is None:
                continue
            destination = np.r_[box[:2], .13]
            transport = path(transport_arc(lift_target, destination), lift.solutions[-1].joints, yaw)
            if transport is not None:
                return PickPlan(np.asarray(origin).copy(), np.asarray(box).copy(), target, yaw,
                    MotionPlan([approach]), lower, lift, transport)
    return None
