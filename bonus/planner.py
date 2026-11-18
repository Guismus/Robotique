"""Cibles 3 rouges + 2 verts + 1 bleu, de bas en haut."""
import sys
import numpy as np
from scipy.optimize import least_squares
sys.path.insert(0,'/project/ros2_ws/src/so101_brain')
from so101_brain.kinematics import Kinematics

TARGETS=[(1,np.array([.16+dx,-.17,.025])) for dx in (-.05,0,.05)]
TARGETS += [(2,np.array([.16+dx,-.17,.075])) for dx in (-.025,.025)]
TARGETS += [(3,np.array([.16,-.17,.125]))]

class PyramidIK(Kinematics):
 def solve(self,point,current):
  radial=np.array([point[0],point[1],0.]);radial/=np.linalg.norm(radial)
  for tilt in (0,20,40,60):
   angle=np.radians(tilt)
   for sign in (1,-1):
    direction=sign*np.sin(angle)*radial+np.array([0,0,-np.cos(angle)])
    def error(q):
     pose=self.forward(q)
     return np.r_[pose[:3,3]-point,.06*(pose[:3,2]-direction)]
    for seed in (current,np.zeros(5)):
     fit=least_squares(error,np.clip(seed,*self.bounds),bounds=self.bounds,max_nfev=200)
     pose=self.forward(fit.x)
     if np.linalg.norm(pose[:3,3]-point)<.004 and np.linalg.norm(pose[:3,2]-direction)<.12:
      return fit.x
  raise ValueError('Cible inaccessible, meme avec une approche inclinee')

if __name__=='__main__':
 k=PyramidIK('/project/sim/so101_sim/assets/so101/so101_new_calib.urdf')
 q=np.zeros(5)
 for color,p in TARGETS:
  for target in (p+[0,0,.035],p):
   try:q=k.solve(target,q);print(color,target.tolist(),'OK',flush=True)
   except ValueError as e:print(color,target.tolist(),str(e),flush=True)
