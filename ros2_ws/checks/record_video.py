"""Enregistre les images ROS reelles du cycle, avec le nom des etapes."""
import time
from pathlib import Path
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
rclpy.init(); node=Node('enregistrement_cycle'); bridge=CvBridge()
folder=Path('/ros2_ws/checks'); frame=None; state='ATTENTE'; history=[]
writer=cv2.VideoWriter(str(folder/'cube_vers_bac.avi'),cv2.VideoWriter_fourcc(*'MJPG'),10,(960,800))
def camera(msg):
 global frame
 frame=bridge.imgmsg_to_cv2(msg,'bgr8')
def status(msg):
 global state
 state=msg.data
 if not history or history[-1]!=state:
  history.append(state); print(state,flush=True)
def record():
 if frame is None: return
 view=cv2.resize(frame,(960,720),interpolation=cv2.INTER_NEAREST)
 view=cv2.copyMakeBorder(view,0,80,0,0,cv2.BORDER_CONSTANT,value=(28,28,28))
 cv2.putText(view,'SO-101 : cube creux vers bac',(24,752),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
 cv2.putText(view,state,(24,784),cv2.FONT_HERSHEY_SIMPLEX,.65,(100,220,255),2)
 writer.write(view)
node.create_subscription(Image,'/external_cam/image_raw',camera,1)
node.create_subscription(String,'/brain_state',status,10)
node.create_timer(.1,record)
client=node.create_client(Trigger,'/brain/start')
started=False; pending=None; next_try=time.monotonic()+5; end=time.monotonic()+120; done=None
while time.monotonic()<end:
 rclpy.spin_once(node,timeout_sec=.05)
 now=time.monotonic()
 if not started and pending is None and now>next_try and frame is not None and client.service_is_ready():
  pending=client.call_async(Trigger.Request())
 if pending is not None and pending.done():
  result=pending.result(); print(result,flush=True); started=result.success;pending=None;next_try=now+3
 if started and (state.startswith('ERREUR') or state=='CYCLE_TERMINE_A_VERIFIER'):
  if done is None: done=now
  if now-done>4: break
writer.release()
if frame is not None: cv2.imwrite(str(folder/'video_final.png'),frame)
(folder/'video_resultat.txt').write_text('\n'.join(history))
print('VIDEO',history,flush=True)
node.destroy_node(); rclpy.shutdown()
