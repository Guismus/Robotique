"""Essai ROS complet : commande uniquement via le service /brain/start."""
import time,json
from pathlib import Path
import rclpy,cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
rclpy.init();n=Node('test_bonus');bridge=CvBridge();states=[];image=None;detected=[]
def state(msg):
 if not states or states[-1]!=msg.data:states.append(msg.data);print(msg.data,flush=True)
def frame(msg):
 global image
 image=bridge.imgmsg_to_cv2(msg,'bgr8')
def cubes(msg):
 global detected
 detected=json.loads(msg.data)['cubes']
n.create_subscription(String,'/brain_state',state,10)
n.create_subscription(String,'/bonus/cubes',cubes,10)
n.create_subscription(Image,'/external_cam/image_raw',frame,1)
c=n.create_client(Trigger,'/brain/start');end=time.monotonic()+30
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.1)
 if detected and c.service_is_ready():break
print('cubes',detected,flush=True)
if c.service_is_ready():
 f=c.call_async(Trigger.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=20);print(f.result(),flush=True)
 end=time.monotonic()+240
 while f.result() and f.result().success and time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.1)
  if states and (states[-1].startswith(('ERREUR','ECHEC')) or states[-1]=='PYRAMIDE_A_INSPECTER'):break
for _ in range(20):rclpy.spin_once(n,timeout_sec=.1)
if image is not None:cv2.imwrite('/project/bonus/essai_final.png',image)
Path('/project/bonus/essai.json').write_text(json.dumps({'states':states,'remaining_detected':detected},indent=2))
n.destroy_node();rclpy.shutdown()
