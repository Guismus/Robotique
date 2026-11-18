"""Detection apprise et estimation geometrique. Aucune position MuJoCo ici."""
import json,time
import cv2,numpy as np,torch,rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String,Float64MultiArray
from cv_bridge import CvBridge
from network import CubeNet
from planner import TARGETS

class Vision(Node):
 def __init__(self):
  super().__init__('bonus_vision');torch.set_num_threads(1)
  self.model=CubeNet();self.model.load_state_dict(torch.load('/project/bonus/cube_net.pt',map_location='cpu',weights_only=True));self.model.eval()
  self.k=self.t=None;self.last=0;self.bridge=CvBridge()
  self.pub=self.create_publisher(String,'/bonus/cubes',10)
  self.check=self.create_publisher(String,'/bonus/placements',10)
  self.mask=self.create_publisher(Image,'/bonus/vision',1)
  self.create_subscription(CameraInfo,'/external_cam/camera_info',self.info,1)
  self.create_subscription(Float64MultiArray,'/external_cam/pose',self.pose,1)
  self.create_subscription(Image,'/external_cam/image_raw',self.image,1)
 def info(self,msg):self.k=np.array(msg.k).reshape(3,3)
 def pose(self,msg):self.t=np.array(msg.data).reshape(4,4)
 def project(self,u,v,z):
  ray=self.t[:3,:3]@np.linalg.solve(self.k,[u,v,1]);origin=self.t[:3,3]
  return origin+(z-origin[2])/ray[2]*ray
 def image(self,msg):
  if self.k is None or self.t is None or time.monotonic()-self.last<.15:return
  self.last=time.monotonic();rgb=self.bridge.imgmsg_to_cv2(msg,'rgb8')
  with torch.no_grad():labels=self.model(torch.from_numpy(rgb.copy()).permute(2,0,1).float()[None]/255).argmax(1)[0].numpy().astype('uint8')
  cubes=[]
  for color in (1,2,3):
   mask=cv2.morphologyEx((labels==color).astype('uint8')*255,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
   contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
   for contour in contours:
    if cv2.contourArea(contour)<20:continue
    (u,v),(w,h),yaw=cv2.minAreaRect(contour)
    p=self.project(u,v,.05);p[2]=.025
    if p[1]>-.11:cubes.append({'color':color,'xyz':p.tolist(),'yaw_image_deg':yaw})
  checks=[]
  for color,p in TARGETS:
   top=np.r_[p+[0,0,.025],1.];cam=np.linalg.inv(self.t)@top
   pixel=self.k@cam[:3];u,v=np.rint(pixel[:2]/pixel[2]).astype(int)
   patch=labels[max(0,v-3):v+4,max(0,u-3):u+4]
   # Le trou peut occuper le centre : accepter une portion coloree autour.
   checks.append(bool(patch.size and np.mean(patch==color)>.15))
  self.pub.publish(String(data=json.dumps({'stamp':time.time(),'cubes':cubes})))
  self.check.publish(String(data=json.dumps(checks)))
  palette=np.array([[20,20,20],[230,40,60],[30,210,50],[40,70,230]],np.uint8)
  display=self.bridge.cv2_to_imgmsg(palette[labels],'rgb8');display.header=msg.header;self.mask.publish(display)

def main():
 rclpy.init();node=Vision()
 try:rclpy.spin(node)
 finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
