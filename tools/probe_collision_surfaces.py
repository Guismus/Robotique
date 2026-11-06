"""Compare contact points against the original STL triangle surfaces."""
from pathlib import Path
import sys
import json
import mujoco
import numpy as np
root = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(root / "sim"), str(root / "ros2_ws/src/so101_brain"), str(root / "tools")]
from so101_sim import SO101Sim, ARM_JOINTS
from so101_brain.kinematics import Kinematics, HOME, yaw_rotation
from diagnose_geometry import vertices


def distance_to_triangles(point, triangles):
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    u, v, w = b-a, c-a, point-a
    uu, uv, vv = np.sum(u*u, 1), np.sum(u*v, 1), np.sum(v*v, 1)
    wu, wv = np.sum(w*u, 1), np.sum(w*v, 1)
    denominator = uu*vv-uv*uv
    safe = np.where(abs(denominator)>1e-20, denominator, np.nan)
    s, t = (wu*vv-wv*uv)/safe, (wv*uu-wu*uv)/safe
    normal = np.cross(u, v)
    normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-20)
    plane_distance = np.abs(np.sum(w*normal, 1))
    best = np.where((s>=0)&(t>=0)&(s+t<=1), plane_distance, np.inf)
    for start, end in ((a,b), (b,c), (c,a)):
        delta = end-start
        fraction = np.clip(np.sum((point-start)*delta, 1)/np.maximum(np.sum(delta*delta,1),1e-20), 0, 1)
        best = np.minimum(best, np.linalg.norm(point-start-fraction[:,None]*delta,axis=1))
    return float(np.min(best))


def triangles(sim, geom):
    mesh = sim.model.geom_dataid[geom]
    start, count = sim.model.mesh_faceadr[mesh], sim.model.mesh_facenum[mesh]
    return vertices(sim.model, sim.data, geom)[sim.model.mesh_face[start:start+count]]


sim = SO101Sim(seed=0, sim_dt_per_step=0)
sim.connect()
kin = Kinematics(root / "ros2_ws/src/so101_description/urdf/so101.urdf")
initial = sim.get_object_position().copy()
rotation = sim.data.body("pickup_object").xmat.reshape(3,3)
yaw = np.arctan2(rotation[1,0],rotation[0,0])+np.pi
target = initial + yaw_rotation(yaw)[:,0]*.003
solution = kin.solve(target, HOME, yaw)
for name, value in zip(ARM_JOINTS, solution.joints):
    sim.data.qpos[sim.model.joint(name).qposadr[0]] = value
    sim.data.ctrl[sim.model.actuator(name).id] = value
gaddr, gid = sim.model.joint("gripper").qposadr[0], sim.model.actuator("gripper").id
sim.data.qpos[gaddr]=sim.data.ctrl[gid]=.8814
sim.data.qvel[:]=0
mujoco.mj_forward(sim.model,sim.data)
obj = sim.model.body("pickup_object").id
moving = sim.model.body("moving_jaw_so101_v1").id
fixed = sim.model.body("gripper").id
rows=[]
for step in range(1600):
    sim.data.ctrl[gid] -= .0002
    mujoco.mj_step(sim.model,sim.data)
    if step % 50:
        continue
    for contact in sim.data.contact:
        b1,b2=sim.model.geom_bodyid[contact.geom1],sim.model.geom_bodyid[contact.geom2]
        if obj not in (b1,b2) or not (b1 in (moving,fixed) or b2 in (moving,fixed)):
            continue
        robot_geom = contact.geom2 if b1 == obj else contact.geom1
        original_geom = robot_geom
        other = sim.model.geom_bodyid[robot_geom]
        if other == fixed:
            original_geom = next(i for i in range(sim.model.ngeom) if sim.model.geom_bodyid[i] == fixed and sim.model.geom_dataid[i] == sim.model.mesh("wrist_roll_follower_so101_v1").id)
        elif other == moving:
            original_geom = next(i for i in range(sim.model.ngeom) if sim.model.geom_bodyid[i] == moving and sim.model.geom_dataid[i] == sim.model.mesh("moving_jaw_so101_v1").id)
        gap = distance_to_triangles(contact.pos, triangles(sim, original_geom))
        cube_gap = distance_to_triangles(contact.pos, triangles(sim,sim.model.geom("pickup_object_visual").id))
        rows.append({"t":step*.002,"jaw": "moving" if other == moving else "fixed", "penetration": float(-contact.dist),
            "distance_to_actual_jaw_stl_m":gap,"distance_to_actual_cube_stl_m":cube_gap,"position":contact.pos.tolist()})
print(json.dumps(rows,indent=2))
sim.disconnect()
