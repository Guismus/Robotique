"""Diagnostic de la pile, sans ROS : rejoue les etapes du cerveau dans MuJoCo et mesure
ou est reellement chaque cube (position vraie de la sim, interdite dans le pipeline, OK ici pour mesurer).

    docker compose run --rm -e PYTHONPATH=/opt/so101/sim control bash -lc \
        "cd /ros2_ws/src/so101_brain && python3 /ros2_ws/checks/stack_probe.py 1"
"""
import sys
import numpy as np
from so101_sim import SO101Sim, ASSETS_DIR
from so101_brain.kinematics import Kinematics, NAMES

SOCLE, CUBE, LACHER, MARGE = 0.002, 0.05, float(sys.argv[3]) if len(sys.argv) > 3 else 0.01, 0.035  # memes valeurs que brain_node.py
DANS_PINCE = np.array([0.018, 0., -0.012])
seed = int(sys.argv[1]) if len(sys.argv) > 1 else 1
sim = SO101Sim(scene_path=ASSETS_DIR / 'scene_stack.xml', seed=seed)
sim.connect()
kin = Kinematics('/opt/so101/sim/so101_sim/assets/so101/so101_new_calib.urdf')
q = np.zeros(5)


def reach(p, start):
    try:
        return kin.solve(p, start)
    except ValueError:
        return kin.solve(p, start, tilt=True)


def carry(p, start):
    '''Comme brain_node.carry : place le cube tenu (pas la pince) en p.'''
    q1 = reach(p, start)
    offset = kin.forward(q1)[:3, :3] @ DANS_PINCE
    offset[2] = 0  # seulement l'horizontale : plus bas, les machoires touchent le cube du dessous
    return reach(p - offset, q1)


def move(goal, grip):
    """Meme rampe que le cerveau : 0,025 rad par pas de 50 ms, puis 2 s de stabilisation."""
    global q
    sent = q.copy()
    for _ in range(400):
        sent += np.clip(goal - sent, -0.025, 0.025)
        action = {f'{n}.pos': float(v) for n, v in zip(NAMES, sent)}
        action['gripper.pos'] = grip
        for _ in range(10):  # 50 ms = 10 pas de 5 ms environ
            sim.send_action(action)
        obs = sim.get_observation()
        q = np.array([obs[f'{n}.pos'] for n in NAMES])
        if np.max(np.abs(sent - goal)) < 1e-6 and np.max(np.abs(q - goal)) < 0.04:
            break
    for _ in range(40):
        sim.send_action(action)
    obs = sim.get_observation()
    q = np.array([obs[f'{n}.pos'] for n in NAMES])


stack = sim.data.body('drop_box').xpos[:2].copy()
names = ['pickup_object', 'cube_2', 'cube_3']
for level in range(3):
    free = [n for n in names if np.linalg.norm(sim.data.body(n).xpos[:2] - stack) > 0.08]
    name = min(free, key=lambda n: np.linalg.norm(sim.data.body(n).xpos[:2] - stack))
    obj = sim.data.body(name).xpos.copy(); obj[2] = float(sys.argv[2]) if len(sys.argv) > 2 else 0.025
    center = SOCLE + CUBE / 2 + CUBE * level
    release = np.r_[stack, center + LACHER]
    travel = max(0.08, release[2] + MARGE)
    high = kin.solve(np.r_[obj[:2], 0.08], q)
    low = kin.solve(obj, high)
    lifted = reach(np.r_[obj[:2], travel], high)
    over = carry(np.r_[stack, travel], lifted)
    put = carry(release, over)
    for step, goal, grip in [('APPROCHE', high, 100), ('DESCENTE', low, 100), ('SAISIE', low, 0),
                             ('LEVAGE', lifted, 0), ('TRANSPORT', over, 0), ('POSE', put, 0)]:
        move(goal, grip)
    tcp = kin.forward(q)
    cube = sim.data.body(name).xpos
    print(f'etage {level}: {name} tcp {np.round(tcp[:3, 3], 3)} cube {np.round(cube, 3)} '
          f'ecart cube-tcp {np.round((cube - tcp[:3, 3]) * 1000).astype(int)} mm, inclinaison '
          f'{np.degrees(np.arccos(-tcp[2, 2])):.0f} deg, dans le repere pince '
          f'{np.round(tcp[:3, :3].T @ (cube - tcp[:3, 3]) * 1000).astype(int)} mm')
    move(put, 100)
    if level == 2: print('   pince ouverte :', [np.round(sim.data.body(n).xpos, 3).tolist() for n in names])
    up = reach(kin.forward(put)[:3, 3] + [0, 0, 0.05], put)  # remonter tout droit de 5 cm (comme RETRAIT)
    move(up, 100)
    if level == 2: print('   remontee     :', [np.round(sim.data.body(n).xpos, 3).tolist() for n in names], 'tcp', np.round(kin.forward(q)[:3, 3], 3))
    clear = up.copy(); clear[0] = 1.57
    move(clear, 100)
    print(f'   apres depot : {name} en {np.round(sim.data.body(name).xpos, 3)} (cible {np.round(np.r_[stack, center], 3)})')

# Verite terrain : les 3 cubes empiles a 2,7 / 7,7 / 12,7 cm, a moins de 2 cm de l'axe du socle.
final = [sim.data.body(n).xpos.copy() for n in names]
heights = sorted(p[2] for p in final)
ok = np.allclose(heights, [0.027, 0.077, 0.127], atol=0.01) and all(np.linalg.norm(p[:2] - stack) < 0.02 for p in final)
print(f'RESULTAT graine {seed} : {"PILE OK" if ok else "PILE KO"} (hauteurs {np.round(heights, 3)})')
