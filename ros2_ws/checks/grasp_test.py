import sys
import numpy as np
import mujoco
import cv2
sys.path.insert(0,'/ros2_ws/src/so101_brain')
from so101_brain.kinematics import Kinematics, NAMES
from so101_sim import SO101Sim
k=Kinematics('/opt/so101/sim/so101_sim/assets/so101/so101_new_calib.urdf')
r=SO101Sim(seed=4); r.connect()
obj=r.get_object_position(); obj[0]=.23; obj[1]=.04
adr=r.model.joint('pickup_object_free').qposadr[0]; r.data.qpos[adr:adr+3]=obj
mujoco.mj_forward(r.model,r.data)
q=np.array([r.get_observation()[n+'.pos'] for n in NAMES])
for i,(point,grip) in enumerate([(obj+[0,0,.055],100),(obj,100),(obj,0),(obj+[0,0,.055],0),([.05,-.25,.08],0),([.05,-.25,.08],100)]):
 try: goal=k.solve(np.array(point),q)
 except ValueError as e: print(e,flush=True);break
 start=q.copy()
 for step in range(300):
  target=start+min(1,step/200)*(goal-start)
  r.send_action(dict(zip([n+'.pos' for n in NAMES]+['gripper.pos'],target.tolist()+[grip])))
 q=np.array([r.get_observation()[n+'.pos'] for n in NAMES])
 print(i,'obj',r.get_object_position(),'tcp',k.forward(q)[:3,3], 'qerr',max(abs(q-goal)),flush=True)
 cv2.imwrite('/ros2_ws/checks/step%d.png'%i,cv2.cvtColor(r.render_camera(),cv2.COLOR_RGB2BGR))
r.disconnect()
