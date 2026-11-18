"""Entrainement supervise sur images MuJoCo, labels reserves a l'entrainement."""
import sys, json
from pathlib import Path
import numpy as np
import torch
import cv2
sys.path.insert(0,'/project/sim')
from so101_sim import SO101Sim
from network import CubeNet
root=Path('/project/bonus');torch.manual_seed(17);torch.set_num_threads(2)
robot=SO101Sim(scene_path='/project/sim/so101_sim/assets/so101/scene_tek5.xml',seed=17)
robot.connect();images=[];labels=[]
for index in range(48):
 robot.randomize_object()
 rgb=robot.render_camera().copy()
 robot._renderer.enable_segmentation_rendering()
 robot._renderer.update_scene(robot.data,camera='external_cam')
 segmentation=robot._renderer.render().copy();robot._renderer.disable_segmentation_rendering()
 label=np.zeros(rgb.shape[:2],np.int64)
 for i in range(10):
  name='pickup_object' if i==0 else f'cube_{i}'
  body_id=robot.model.body(name).id
  geoms=np.flatnonzero(robot.model.geom_bodyid==body_id)
  label[np.isin(segmentation[:,:,0],geoms)]=1 if i<5 else 2 if i<8 else 3
 images.append(torch.from_numpy(rgb.copy()).permute(2,0,1).float()/255)
 labels.append(torch.from_numpy(label))
 if index==47:cv2.imwrite(str(root/'scene_bonus.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
x=torch.stack(images);y=torch.stack(labels);model=CubeNet();opt=torch.optim.Adam(model.parameters(),lr=.01)
loss_fn=torch.nn.CrossEntropyLoss(weight=torch.tensor([.1,1.,1.,1.]))
pixels=x[:36].permute(0,2,3,1).reshape(-1,3); targets=y[:36].reshape(-1)
indices=torch.randperm(len(pixels))[:120000]
x_train=pixels[indices].T[None,:,None,:];y_train=targets[indices][None,None,:]
for epoch in range(180):
 model.train();opt.zero_grad();output=model(x_train);loss=loss_fn(output,y_train);loss.backward();opt.step()
 if epoch%30==0:print(epoch,float(loss.detach()),flush=True)
model.eval()
with torch.no_grad():
 pred=model(x[36:]).argmax(1);truth=y[36:];iou={str(c):float(((pred==c)&(truth==c)).sum()/((pred==c)|(truth==c)).sum().clamp_min(1)) for c in (1,2,3)}
torch.save(model.state_dict(),root/'cube_net.pt')
(root/'validation_ia.json').write_text(json.dumps({'train_images':36,'validation_images':12,'iou':iou},indent=2))
print(iou,flush=True);robot.disconnect()
