"""Read-only comparison of URDF FK, MJCF sites and actual collision vertices."""
import json
from pathlib import Path
import sys

import ikpy.chain
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
from so101_sim import SO101Sim, ARM_JOINTS


def vertices(model, data, geom_id):
    mesh = model.geom_dataid[geom_id]
    start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
    return model.mesh_vert[start:start + count] @ data.geom_xmat[geom_id].reshape(3, 3).T + data.geom_xpos[geom_id]


def main():
    sim = SO101Sim(scene_path=ROOT / "sim/so101_sim/assets/so101/scene_tek5.xml", seed=0)
    sim.connect()
    chain = ikpy.chain.Chain.from_urdf_file(str(ROOT / "ros2_ws/src/so101_description/urdf/so101.urdf"),
        active_links_mask=[False, True, True, True, True, True, False])
    result = {"links": [link.name for link in chain.links], "poses": [], "jaws": []}
    for q in ([0] * 5, [0, -.2, .4, .4, 0], [.4, -.5, .8, .9, .2]):
        for name, angle in zip(ARM_JOINTS, q):
            sim.data.qpos[sim.model.joint(name).qposadr[0]] = angle
        mujoco.mj_forward(sim.model, sim.data)
        fk = chain.forward_kinematics([0, *q, 0])
        site = sim.data.site("gripperframe")
        result["poses"].append({"q": q, "fk": fk.tolist(), "mj_position": site.xpos.tolist(),
            "mj_rotation": site.xmat.reshape(3, 3).tolist(), "position_error": float(np.linalg.norm(fk[:3, 3] - site.xpos))})
    # Express collision geometry in gripper body coordinates (independent of arm pose).
    for pct in (0, 25, 50, 75, 100):
        sim.data.qpos[sim.model.joint("gripper").qposadr[0]] = sim._gripper_pct_to_rad(pct)
        mujoco.mj_forward(sim.model, sim.data)
        body = sim.data.body("gripper")
        geoms = []
        for i in range(sim.model.ngeom):
            if sim.model.geom_bodyid[i] not in (sim.model.body("gripper").id, sim.model.body("moving_jaw_so101_v1").id):
                continue
            if not sim.model.geom_contype[i] or sim.model.geom_type[i] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            v = (vertices(sim.model, sim.data, i) - body.xpos) @ body.xmat.reshape(3, 3)
            tip = v[v[:, 2] < -.075]
            geoms.append({"mesh": mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_MESH, sim.model.geom_dataid[i]),
                "bounds": [v.min(0).tolist(), v.max(0).tolist()],
                "tip_bounds": [tip.min(0).tolist(), tip.max(0).tolist()] if len(tip) else None})
        result["jaws"].append({"percent": pct, "geoms": geoms})
    print(json.dumps(result, indent=2))
    sim.disconnect()


if __name__ == "__main__":
    main()
