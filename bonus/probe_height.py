import sys
import numpy as np
sys.path.insert(0,'/project/ros2_ws/src/so101_brain')
from so101_brain.kinematics import Kinematics
k=Kinematics('/project/sim/so101_sim/assets/so101/so101_new_calib.urdf')
for x in [.10,.13,.16,.19,.22]:
 for y in [-.1,0,.1]:
  results=[]
  for z in [.025,.075,.125,.175]:
   try:k.solve(np.array([x,y,z]),np.zeros(5)); results.append(z)
   except ValueError: pass
  print(x,y,results)
