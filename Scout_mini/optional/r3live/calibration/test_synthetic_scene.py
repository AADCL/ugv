#!/usr/bin/env python3
"""Generate a known camera/LiDAR geometry dataset for solver smoke testing only."""
import argparse
from pathlib import Path
import cv2
import numpy as np
from calibrate import write_pcd,dump,digest

parser=argparse.ArgumentParser(); parser.add_argument('root',type=Path); args=parser.parse_args()
args.root.mkdir(parents=True,exist_ok=False)
camera={'width':640,'height':480,'K':[400.,0.,320.,0.,400.,240.,0.,0.,1.],
        'D':[0.]*5,'distortion_model':'plumb_bob','optical_frame':'synthetic_color_optical'}
theta=np.deg2rad(1.)
initial=np.eye(4); initial[:3,:3]=[[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]]
initial[:3,3]=[.015,-.01,.02]
for idx,(half_x,half_y,back) in enumerate([(.93,.73,3.17),(1.13,.83,3.37),(.87,.63,2.77)]):
    d=args.root/'scenes'/('synthetic_%02d'%idx); d.mkdir(parents=True)
    step=.018; clouds=[]
    xs=np.arange(-half_x,half_x+step/2,step); ys=np.arange(-half_y,half_y+step/2,step); zs=np.arange(1.,back+step/2,step)
    x,y=np.meshgrid(xs,ys); clouds.append(np.c_[x.ravel(),y.ravel(),np.full(x.size,back)])
    y,z=np.meshgrid(ys,zs)
    for side in (-half_x,half_x): clouds.append(np.c_[np.full(y.size,side),y.ravel(),z.ravel()])
    x,z=np.meshgrid(xs,zs)
    for side in (-half_y,half_y): clouds.append(np.c_[x.ravel(),np.full(x.size,side),z.ravel()])
    points=np.concatenate(clouds).astype(np.float32)
    points=np.c_[points,np.full(len(points),100,dtype=np.float32)]
    u,v=np.meshgrid(np.arange(640),np.arange(480)); dx=(u-320)/400.; dy=(v-240)/400.
    dist=np.stack([np.full(dx.shape,back),half_x/np.maximum(np.abs(dx),1e-9),half_y/np.maximum(np.abs(dy),1e-9)])
    face=np.argmin(dist,axis=0); image=np.full((480,640,3),210,np.uint8)
    image[(face==1)&(dx<0)]=[30,30,30]; image[(face==1)&(dx>=0)]=[100,100,100]
    image[(face==2)&(dy<0)]=[160,160,160]; image[(face==2)&(dy>=0)]=[60,60,60]
    cv2.imwrite(str(d/'image.bmp'),image); write_pcd(d/'cloud.pcd',points)
    dump(d/'scene.yaml',{'schema':1,'camera':camera,'lidar_frame':'synthetic_lidar',
        'initial_T_camera_lidar':initial.tolist(),'lidar_to_imu_translation':[-.011,-.02329,.04412],
        'point_count':len(points),'image_sha256':digest(d/'image.bmp'),'cloud_sha256':digest(d/'cloud.pcd'),
        'note':'Synthetic test only. Ground truth T_camera_lidar is identity, not a robot calibration.'})
print('Generated synthetic scenes at '+str(args.root))
