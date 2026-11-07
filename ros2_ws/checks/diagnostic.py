"""Diagnostic hors ROS : la verite MuJoCo est reservee aux tests."""
import sys
import numpy as np
import mujoco
import cv2
sys.path.insert(0, '/ros2_ws/src/so101_brain')
from so101_brain.kinematics import Kinematics
from so101_sim import SO101Sim
k = Kinematics('/opt/so101/sim/so101_sim/assets/so101/so101_new_calib.urdf')
r = SO101Sim(seed=4)
r.connect()
print('OBJECT', r.get_object_position(), flush=True)
for p in [[.25,.05,.025],[.25,.05,.09],[.05,-.25,.09]]:
    try:
        q=k.solve(np.array(p),np.zeros(5))
        print('IK',p,q,k.forward(q)[:3,3],flush=True)
    except ValueError as e:
        print('FAIL',p,str(e),flush=True)
cv2.imwrite('/ros2_ws/checks/scene.png', cv2.cvtColor(r.render_camera(),cv2.COLOR_RGB2BGR))
r.disconnect()

