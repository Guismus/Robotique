"""Construire 3 rouges, 2 verts, 1 bleu, puis verifier chaque depot."""
import sys,json,time
import numpy as np,rclpy
from std_msgs.msg import String
sys.path.insert(0,'/project/ros2_ws/src/so101_brain')
from so101_brain.brain_node import BrainNode
from planner import PyramidIK,TARGETS

class PyramidBrain(BrainNode):
 def __init__(self):
  super().__init__();self.kin=PyramidIK('/project/sim/so101_sim/assets/so101/so101_new_calib.urdf')
  self.cubes=[];self.cubes_at=0;self.checks=[];self.checks_at=0;self.slot=0;self.running=False;self.wait_until=None
  self.create_subscription(String,'/bonus/cubes',self.cube_update,10)
  self.create_subscription(String,'/bonus/placements',self.check_update,10)
 def cube_update(self,msg):
  value=json.loads(msg.data);self.cubes=value['cubes'];self.cubes_at=time.monotonic()
 def check_update(self,msg):self.checks=json.loads(msg.data);self.checks_at=time.monotonic()
 def start(self,request,response):
  if self.running or self.steps:response.message='Construction en cours';return response
  if self.q is None or time.monotonic()-self.cubes_at>1:response.message='Attendre les cubes et les moteurs';return response
  try:
   for _,p in TARGETS:
    self.kin.solve(p,self.q);self.kin.solve(p+[0,0,.035],self.q)
   self.slot=0;self.prepare()
  except ValueError as error:response.message=str(error);return response
  self.running=True;response.success=True;response.message='Pyramide 3-2-1 demarree';return response
 def prepare(self):
  color,target=TARGETS[self.slot]
  for cube in self.cubes:
   if cube['color']!=color:continue
   source=np.array(cube['xyz'])
   try:
    high=self.kin.solve(source+[0,0,.055],self.q)
    low=self.kin.solve(source,high)
    above=self.kin.solve(target+[0,0,.035],high)
    place=self.kin.solve(target,above)
   except ValueError:continue
   self.steps=[('APPROCHE',high,100.),('DESCENTE',low,100.),('SAISIE',low,0.),
    ('LEVAGE',high,0.),('TRANSPORT',above,0.),('POSE',place,0.),('OUVERTURE',place,100.),('RETRAIT',above,100.)]
   self.index=0;self.goal=None;self.wait_until=None
   self.get_logger().info(f'Cube {self.slot+1}/6, couleur {color}')
   return
  raise ValueError(f'Aucun cube accessible de couleur {color}')
 def stop(self,request,response):
  self.running=False;self.wait_until=None
  return super().stop(request,response)
 def tick(self):
  super().tick()
  if not getattr(self,'running',False) or self.steps:return
  if self.state.startswith('ERREUR'):self.running=False;return
  if self.wait_until is None:self.wait_until=time.monotonic()+3;return
  if time.monotonic()<self.wait_until:return
  if time.monotonic()-self.checks_at>1 or len(self.checks)<=self.slot or not self.checks[self.slot]:
   self.state='ECHEC_DEPOT_VISUEL';self.running=False;return
  self.slot+=1
  if self.slot==len(TARGETS):self.state='PYRAMIDE_A_INSPECTER';self.running=False;return
  try:self.prepare()
  except ValueError as error:
   self.get_logger().error(str(error));self.state='ECHEC_CUBE_INACCESSIBLE';self.running=False

def main():
 rclpy.init();node=PyramidBrain()
 try:rclpy.spin(node)
 finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
