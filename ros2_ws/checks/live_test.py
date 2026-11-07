import time
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
rclpy.init(); n=Node('test_cycle_camera'); bridge=CvBridge(); images=[]; states=[]; positions={}
def image(msg):
 images[:]=[bridge.imgmsg_to_cv2(msg,'bgr8')]
def state(msg):
 if not states or states[-1]!=msg.data:
  states.append(msg.data); print('STATE',msg.data,flush=True)
  if images: cv2.imwrite('/ros2_ws/checks/live_'+msg.data+'.png',images[-1])
n.create_subscription(Image,'/external_cam/image_raw',image,1)
n.create_subscription(String,'/brain_state',state,10)
for name in ['object','drop_box']:
 n.create_subscription(PoseStamped,'/'+name+'_position',lambda msg,name=name: positions.update({name:[msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]}),10)
c=n.create_client(Trigger,'/brain/start')
end=time.monotonic()+20
while time.monotonic()<end and (not c.service_is_ready() or len(positions)<2): rclpy.spin_once(n,timeout_sec=.1)
print('POSITIONS',positions,flush=True)
if c.service_is_ready() and len(positions)==2:
 f=c.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n,f,timeout_sec=10); print('START',f.result(),flush=True)
 end=time.monotonic()+70
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.1)
  if states and (states[-1].startswith('ERREUR') or states[-1]=='CYCLE_TERMINE_A_VERIFIER'): break
 for _ in range(20): rclpy.spin_once(n,timeout_sec=.1)
 if images: cv2.imwrite('/ros2_ws/checks/live_final.png',images[-1])
 print('FINAL',states,positions,flush=True)
n.destroy_node(); rclpy.shutdown()
