"""Scene bonus : dix exemplaires du cube creux original, sans bac."""
from pathlib import Path
from copy import deepcopy
import xml.etree.ElementTree as ET
root=Path('/project')
p=root/'sim/so101_sim/assets/so101/scene_tek5.xml'
tree=ET.parse(p); scene=tree.getroot(); world=scene.find('worldbody')
world.remove(world.find("body[@name='drop_box']"))
original=world.find("body[@name='pickup_object']")
asset=scene.find('asset')
for name,color in [('green','0.05 0.8 0.15 1'),('blue','0.05 0.15 0.8 1')]:
 ET.SubElement(asset,'material',name=name,rgba=color)
for i in range(10):
 cube=original if i==0 else deepcopy(original)
 color=['red']*5+['green']*3+['blue']*2
 name='pickup_object' if i==0 else f'cube_{i}'
 cube.set('name',name);cube.find('freejoint').set('name',name+'_free')
 visual=cube.find('geom');visual.set('name',name+'_visual');visual.set('material',color[i])
 cube.set('pos',f'{.20+.08*(i%3)} {-.24+.08*(i//3)} .025')
 if i:world.append(cube)
scene.find("worldbody/camera").set('pos','0.15 0 0.85')
tree.write(p,encoding='unicode')
p=root/'sim/so101_sim/__init__.py'
s=p.read_text();start=s.index('        r = self._rng.uniform',s.index('    def randomize_object'));end=s.index('    def get_object_position',start)
s=s[:start]+'''        # Placement sans chevauchement, zone de construction reservee a y < -0.12.
        names = ['pickup_object'] + [f'cube_{i}' for i in range(1, 10)]
        slots = [(x, y) for x in (0.16, 0.245, 0.33) for y in (-0.07, 0.015, 0.10, 0.185)]
        self._rng.shuffle(slots)
        for name, (x, y) in zip(names, slots):
            adr = self.model.joint(name + '_free').qposadr[0]
            yaw = self._rng.uniform(0, np.pi / 2)
            self.data.qpos[adr:adr+3] = [x+self._rng.uniform(-.003,.003), y+self._rng.uniform(-.003,.003), .026]
            self.data.qpos[adr+3:adr+7] = [np.cos(yaw/2), 0, 0, np.sin(yaw/2)]
        self.data.qvel[:] = 0
        for _ in range(200):
            mujoco.mj_step(self.model, self.data)

'''+s[end:];p.write_text(s)
# La graine -1 signifie un nouveau tirage a chaque lancement.
p=root/'ros2_ws/src/so101_driver/so101_driver/driver_node.py';s=p.read_text().replace("robot = SO101Sim(seed=self.declare_parameter('seed', 1).value)","seed = self.declare_parameter('seed', -1).value\n            robot = SO101Sim(seed=None if seed < 0 else seed)");p.write_text(s)
p=root/'docker/docker-compose.yml';s=p.read_text().replace('${SIM_SEED:-1}','${SIM_SEED:--1}');p.write_text(s)
p=root/'ros2_ws/src/so101_driver/launch/driver.launch.py';s=p.read_text().replace('"seed", default_value="1"','"seed", default_value="-1"');p.write_text(s)
