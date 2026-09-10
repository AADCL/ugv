#!/usr/bin/env python3
"""Scout static LiDAR-camera calibration. Never publishes TF or motion commands."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import sys
import time

import cv2
import numpy as np
import yaml
from calibration_io import rigid_matrix, camera_signature, same_camera
cv2.setNumThreads(1)

LIDAR='/livox/lidar'
IMU='/livox/imu'
IMAGE='/r3live_camera/color/image_raw'
INFO='/r3live_camera/color/camera_info'
WHEEL='/scout/odom'
TOPICS=[LIDAR,IMU,IMAGE,INFO,WHEEL,'/tf_static']
R3=Path('/home/nvidia/r3live_ws')
PKG=R3/'src/scout_r3live_bringup'
UPSTREAM=Path('/home/nvidia/lidar_camera_calib_ws/src/livox_camera_calib')


def dump(path,value):
    Path(path).write_text(yaml.safe_dump(value,sort_keys=False),encoding='utf-8')


def read(path):
    return yaml.safe_load(Path(path).read_text())


def digest(path):
    result=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''): result.update(block)
    return result.hexdigest()


def safe_name(value):
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}',value):
        raise ValueError('Name must contain 1-64 letters, digits, underscores or hyphens')
    return value


def scene_path(args,name): return args.root/'scenes'/safe_name(name)


def stop(process):
    if process and process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try: process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL); process.wait()


def confirmed(args):
    if not args.stationary:
        if not sys.stdin.isatty() or input('车辆和场景均已静止？输入 YES 开始（程序不会停车）：').strip()!='YES':
            raise ValueError('Stationary confirmation required; use --stationary only after checking the scene')


def sensors(args):
    import rosgraph
    master=rosgraph.Master('/scout_calibration_preflight')
    publishers=master.getSystemState()[0]
    active={topic for topic,nodes in publishers if nodes}
    if '/clock' in active: raise ValueError('Live sensor entry refuses a graph publishing /clock')
    start_lidar=LIDAR not in active
    start_camera=IMAGE not in active
    if start_camera and any(t.endswith('/color/image_raw') for t in active):
        raise ValueError('Another color camera is active. Stop its owning launch before opening this camera')
    if not start_lidar and not start_camera:
        print('Required sensor publishers already present; capture can reuse them.'); return
    print('Reusing existing publishers; starting missing sensors only. No chassis or estimator launch.',flush=True)
    os.execvp('roslaunch',['roslaunch','scout_r3live_bringup','sensors.launch',
        'start_lidar:='+str(start_lidar).lower(),'start_camera:='+str(start_camera).lower()])


def capture(args):
    import rospy
    from sensor_msgs.msg import Imu,Image,CameraInfo
    from nav_msgs.msg import Odometry
    from livox_ros_driver2.msg import CustomMsg
    confirmed(args)
    rospy.init_node('scout_calibration_capture',anonymous=True,disable_signals=True)
    if rospy.get_param('/use_sim_time',False): raise ValueError('Live capture refuses simulated time')
    state={}; violations=[]
    def callback(topic,msg):
        state[topic]=(time.monotonic(),msg)
        if topic==IMU:
            g=msg.angular_velocity
            if np.linalg.norm([g.x,g.y,g.z])>.035: violations.append('IMU rotation above 0.035 rad/s')
        if topic==WHEEL:
            v=msg.twist.twist.linear; w=msg.twist.twist.angular
            if np.linalg.norm([v.x,v.y,v.z])>.02 or np.linalg.norm([w.x,w.y,w.z])>.035:
                violations.append('Wheel odometry indicates motion')
    subs=[rospy.Subscriber(t,c,lambda m,t=t:callback(t,m),queue_size=2)
          for t,c in [(LIDAR,CustomMsg),(IMU,Imu),(IMAGE,Image),(INFO,CameraInfo),(WHEEL,Odometry)]]
    deadline=time.monotonic()+15
    while not all(t in state for t in [LIDAR,IMU,IMAGE,INFO]) and time.monotonic()<deadline: time.sleep(.1)
    def check():
        now=time.monotonic()
        for t in (LIDAR,IMU,IMAGE,INFO):
            if t not in state or now-state[t][0]>1: raise ValueError('Missing/stale sensor: '+t)
            stamp=state[t][1].header.stamp.to_sec()
            if abs(rospy.Time.now().to_sec()-stamp)>2: raise ValueError('Sensor clock not near ROS wall time: '+t)
        if WHEEL in state and now-state[WHEEL][0]>1: raise ValueError('Previously present wheel odometry became stale')
        if violations: raise ValueError(violations[0]+'; this program cannot stop the vehicle')
    check(); time.sleep(1); check()
    destination=scene_path(args,args.name)
    destination.mkdir(parents=True,exist_ok=False)
    marker=destination/'.incomplete'; marker.touch()
    process=None
    try:
        with (destination/'record.log').open('w') as log:
            process=subprocess.Popen(['rosbag','record','--buffsize=128','--duration='+str(args.seconds),
                '-O',str(destination/'raw.bag')]+TOPICS,stdin=subprocess.DEVNULL,
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            limit=time.monotonic()+args.seconds+20
            while process.poll() is None:
                check()
                if time.monotonic()>limit: raise TimeoutError('rosbag record did not finish')
                time.sleep(.1)
            if process.returncode: raise ValueError('rosbag record failed; inspect record.log')
        dump(destination/'capture.yaml',{'operator_stationary_confirmed':True,'wheel_observed':WHEEL in state,
            'duration_requested_s':args.seconds,'note':'Rotation/wheel gates do not prove scene or translational stationarity'})
        export_scene([destination/'raw.bag'],destination,confirmed_static=True)
        marker.unlink()
        print('CAPTURE_OK '+str(destination))
    finally: stop(process)


def initial_from_tf(transforms,optical):
    from scipy.spatial.transform import Rotation
    def rotation_matrix(value): return value.as_matrix() if hasattr(value,'as_matrix') else value.as_dcm()
    def matrix(xyz,quat):
        t=np.eye(4); t[:3,:3]=rotation_matrix(Rotation.from_quat(quat)); t[:3,3]=xyz; return t
    edges={}
    for m in transforms:
        tr=m.transform; t=matrix([tr.translation.x,tr.translation.y,tr.translation.z],
            [tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w])
        parent=m.header.frame_id.lstrip('/'); child=m.child_frame_id.lstrip('/')
        edges.setdefault(child,[]).append((parent,t)); edges.setdefault(parent,[]).append((child,np.linalg.inv(t)))
    queue=[(optical.lstrip('/'),np.eye(4))]; seen=set(); link_opt=None
    while queue:
        frame,t=queue.pop(0)
        if frame=='r3live_camera_link': link_opt=t; break
        if frame in seen: continue
        seen.add(frame)
        for parent,step in edges.get(frame,[]): queue.append((parent,step@t))
    if link_opt is None: raise ValueError('Bag lacks camera_link to color optical /tf_static chain')
    rig=read(PKG/'config/rig.yaml')
    def rig_t(key):
        d=rig[key]; t=np.eye(4); t[:3,:3]=rotation_matrix(Rotation.from_euler('xyz',d['rpy_degrees'],degrees=True)); t[:3,3]=d['xyz']; return t
    imu_camera=np.linalg.inv(rig_t('base_to_imu'))@rig_t('base_to_camera_link')@link_opt
    translation=read(PKG/'config/estimator.yaml')['r3live_lio']['lidar_to_imu_translation']
    imu_lidar=np.eye(4); imu_lidar[:3,3]=translation
    return rigid_matrix(np.linalg.inv(imu_camera)@imu_lidar),translation


def write_pcd(path,points):
    points=np.asarray(points,dtype='<f4')
    header=('VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
            'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n')%(len(points),len(points))
    with Path(path).open('wb') as f: f.write(header.encode('ascii')); f.write(points.tobytes())


def read_pcd(path):
    with Path(path).open('rb') as f:
        header=[]
        while True:
            line=f.readline()
            if not line: raise ValueError('Missing PCD data header')
            header.append(line.decode('ascii').strip())
            if line.startswith(b'DATA '): break
        if 'FIELDS x y z intensity' not in header or header[-1]!='DATA binary': raise ValueError('Expected exported XYZI binary PCD')
        return np.frombuffer(f.read(),dtype='<f4').reshape(-1,4).copy()


def export_scene(bags,destination,confirmed_static,duration=None):
    import rosbag
    if not confirmed_static: raise ValueError('Static scene must be confirmed')
    clouds=[]; stamps=[]; image_candidates=[]; infos=[]; transforms=[]; total=0
    gyro=[]; wheel=[]; frames=set(); invalid_count=0
    crop_start=None
    if duration is not None:
        for bagpath in bags:
            with rosbag.Bag(str(bagpath)) as bag:
                for _,msg,_ in bag.read_messages(topics=[LIDAR]):
                    crop_start=msg.header.stamp.to_sec(); break
            if crop_start is not None: break
        if crop_start is None: raise ValueError('No LiDAR samples')
    for bagpath in bags:
        with rosbag.Bag(str(bagpath)) as bag:
            for topic,msg,_ in bag.read_messages(topics=TOPICS):
                if topic=='/tf_static': transforms.extend(msg.transforms); continue
                stamp=msg.header.stamp.to_sec()
                if stamp<=0: raise ValueError('Zero sensor timestamp')
                if crop_start is not None and not crop_start<=stamp<=crop_start+duration: continue
                if topic==LIDAR:
                    if msg._type!='livox_ros_driver2/CustomMsg': raise ValueError('Expected driver2 CustomMsg')
                    if msg.point_num!=len(msg.points): raise ValueError('Malformed point_num')
                    frames.add(msg.header.frame_id)
                    values=np.asarray([(p.x,p.y,p.z,p.reflectivity,p.line,p.tag) for p in msg.points],dtype=np.float32).reshape(-1,6)
                    mask=np.isfinite(values).all(axis=1)&(values[:,4]<4)&((values[:,5].astype(np.int32)&0x30)<=0x10)
                    radius=np.linalg.norm(values[:,:3],axis=1)
                    mask&=(radius>=.5)&(radius<=20)
                    invalid_count+=int((~mask).sum())
                    cloud=values[mask,:4]; total+=len(cloud)
                    if total>2500000: raise ValueError('More than 2.5M points; export a shorter stationary recording')
                    clouds.append(cloud); stamps.append(stamp)
                elif topic==INFO: infos.append((stamp,msg))
                elif topic==IMAGE: image_candidates.append((stamp,str(bagpath)))
                elif topic==IMU:
                    v=msg.angular_velocity; gyro.append((stamp,float(np.linalg.norm([v.x,v.y,v.z]))))
                elif topic==WHEEL:
                    v=msg.twist.twist.linear; w=msg.twist.twist.angular
                    wheel.append((stamp,float(np.linalg.norm([v.x,v.y,v.z])),float(np.linalg.norm([w.x,w.y,w.z]))))
    if len(stamps)<10 or not infos or not image_candidates or not gyro or len(frames)!=1:
        raise ValueError('Incomplete scene: need clouds, camera info/image, IMU and one LiDAR frame')
    if any(b<=a for a,b in zip(stamps,stamps[1:])) or max(np.diff(stamps))>.5:
        raise ValueError('LiDAR time reset/duplicate or dropout')
    if stamps[-1]-stamps[0]>15: raise ValueError('Use a stationary segment of at most 15 seconds')
    start,end=stamps[0],stamps[-1]
    gs=sorted(t for t,v in gyro)
    if len(gs)<20 or gs[0]>start+.1 or gs[-1]<end-.1 or max(np.diff(gs))>.1:
        raise ValueError('Insufficient IMU coverage to check stationary rotation')
    if max(v for t,v in gyro if start<=t<=end)>.035: raise ValueError('Rotation during export interval')
    if any(v>.02 or w>.035 for t,v,w in wheel if start<=t<=end): raise ValueError('Wheel motion during export interval')
    candidates=[v for v in image_candidates if start<=v[0]<=end]
    if not candidates: raise ValueError('Image/LiDAR source times do not overlap')
    chosen=min(candidates,key=lambda v:abs(v[0]-(start+end)/2))
    if abs(chosen[0]-(start+end)/2)>.25: raise ValueError('Missing image near the middle of the static interval')
    info_stamp,info=min(infos,key=lambda v:abs(v[0]-chosen[0]))
    if abs(info_stamp-chosen[0])>.2: raise ValueError('CameraInfo is not paired with the image')
    camera=camera_signature(info)
    for _,other in infos: same_camera(camera,camera_signature(other))
    if camera['distortion_model']!='plumb_bob' or len(camera['D'])!=5: raise ValueError('Expected plumb_bob with five distortion coefficients')
    if not np.isfinite(camera['K']+camera['D']).all() or camera['K'][0]<=0 or camera['K'][4]<=0: raise ValueError('Invalid camera calibration')
    image=None
    with rosbag.Bag(chosen[1]) as bag:
        for _,msg,_ in bag.read_messages(topics=[IMAGE]):
            if abs(msg.header.stamp.to_sec()-chosen[0])<1e-8:
                if msg.header.frame_id!=camera['optical_frame']: raise ValueError('Image/CameraInfo frame mismatch')
                # Decode standard raw encodings without loading cv_bridge's C++
                # OpenCV into the Python process (Jetson's python-cv2 is separate).
                channels=1 if msg.encoding=='mono8' else 3
                if msg.encoding not in ('rgb8','bgr8','mono8') or msg.step<msg.width*channels:
                    raise ValueError('Unsupported raw image encoding/stride: '+msg.encoding)
                rows=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)
                image=rows[:,:msg.width*channels].reshape(msg.height,msg.width,channels).copy()
                if msg.encoding=='rgb8': image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                elif msg.encoding=='mono8': image=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
                break
    if image is None or image.shape[:2]!=(camera['height'],camera['width']): raise ValueError('Image resolution mismatch')
    initial,translation=initial_from_tf(transforms,camera['optical_frame'])
    points=np.concatenate(clouds)
    # Keep an expanded forward camera hemisphere; never fold rear points into the image.
    pc=points[:,:3]@initial[:3,:3].T+initial[:3,3]
    k=np.asarray(camera['K']).reshape(3,3)
    z=pc[:,2]; uv=pc[:,:2]/np.maximum(z[:,None],1e-6)
    uv=uv*np.array([k[0,0],k[1,1]])+np.array([k[0,2],k[1,2]])
    mask=(z>.3)&(uv[:,0]>-.5*camera['width'])&(uv[:,0]<1.5*camera['width'])&(uv[:,1]>-.5*camera['height'])&(uv[:,1]<1.5*camera['height'])
    points=points[mask]
    _,indices=np.unique(np.floor(points[:,:3]/.02).astype(np.int32),axis=0,return_index=True)
    points=points[np.sort(indices)]
    if len(points)<2000: raise ValueError('Too few points in expanded camera view: '+str(len(points)))
    destination=Path(destination)
    if not cv2.imwrite(str(destination/'image.bmp'),image): raise IOError('Image write failed')
    write_pcd(destination/'cloud.pcd',points)
    dump(destination/'scene.yaml',{'schema':1,'camera':camera,'lidar_frame':next(iter(frames)),
        'initial_T_camera_lidar':initial.tolist(),'lidar_to_imu_translation':translation,
        'point_count':len(points),'raw_kept_points':total,'rejected_points':invalid_count,
        'voxel_m':.02,'expanded_view_margin':.5,'source_time_interval':[start,end],'image_stamp':chosen[0],
        'max_gyro_rad_s':max(v for t,v in gyro if start<=t<=end),'wheel_observed':bool(wheel),
        'operator_stationary_confirmed':True,'bags':[str(Path(b).resolve()) for b in bags],
        'image_sha256':digest(destination/'image.bmp'),'cloud_sha256':digest(destination/'cloud.pcd'),
        'note':'Source LiDAR frame, not SLAM/map frame; static assumption is operator responsibility'})


def export(args):
    confirmed(args)
    destination=scene_path(args,args.name); destination.mkdir(parents=True,exist_ok=False)
    marker=destination/'.incomplete'; marker.touch()
    export_scene(args.bags,destination,True,args.seconds); marker.unlink()
    print('EXPORT_OK '+str(destination))


def scene(args,name):
    p=scene_path(args,name)
    if (p/'.incomplete').exists(): raise ValueError('Incomplete scene: '+name)
    data=read(p/'scene.yaml')
    if digest(p/'image.bmp')!=data['image_sha256'] or digest(p/'cloud.pcd')!=data['cloud_sha256']:
        raise ValueError('Scene assets changed: '+name)
    return p,data


def prepare(args):
    if len(args.scenes)<2 or len(set(args.scenes))!=len(args.scenes): raise ValueError('Choose at least two distinct scenes (three recommended)')
    loaded=[scene(args,n) for n in args.scenes]; reference=loaded[0][1]
    if len({d['cloud_sha256'] for _,d in loaded})!=len(loaded): raise ValueError('Duplicate clouds are not independent scenes')
    for _,data in loaded:
        same_camera(reference['camera'],data['camera'])
        if data['lidar_frame']!=reference['lidar_frame']: raise ValueError('LiDAR frame changed')
        if not np.allclose(reference['lidar_to_imu_translation'],data['lidar_to_imu_translation']): raise ValueError('Internal geometry changed')
    run=args.root/'runs'/safe_name(args.name); run.mkdir(parents=True,exist_ok=False)
    (run/'image').mkdir(); (run/'pcd').mkdir()
    initial=rigid_matrix(reference['initial_T_camera_lidar'])
    for i,(p,_) in enumerate(loaded):
        shutil.copyfile(p/'image.bmp',run/'image'/('%d.bmp'%i)); shutil.copyfile(p/'cloud.pcd',run/'pcd'/('%d.pcd'%i))
    edge_values={'Canny.gray_threshold':10,'Canny.len_threshold':40,'Voxel.size':.5,
        'Voxel.down_sample_size':.02,'Plane.min_points_size':30,'Plane.normal_theta_min':45,
        'Plane.normal_theta_max':135,'Plane.max_size':8,'Ransac.dis_threshold':.02,
        'Edge.min_dis_threshold':.03,'Edge.max_dis_threshold':.06,'Color.intensity_threshold':0}
    # OpenCV reads upstream dotted keys but its FileStorage writer rejects them.
    edge_text='%YAML:1.0\nExtrinsicMat: !!opencv-matrix\n  rows: 4\n  cols: 4\n  dt: d\n  data: ['
    edge_text+=', '.join(format(float(v),'.17g') for v in initial.reshape(-1))+']\n'
    edge_text+=''.join('%s: %s\n'%(k,v) for k,v in edge_values.items())
    (run/'edges.yaml').write_text(edge_text)
    config={'common':{'image_path':str(run/'image'),'pcd_path':str(run/'pcd'),
        'result_path':str(run/'extrinsic.txt'),'data_num':len(loaded)},'camera':{
        'camera_matrix':reference['camera']['K'],'dist_coeffs':reference['camera']['D']},
        'calib':{'calib_config_file':str(run/'edges.yaml'),'use_rough_calib':True}}
    dump(run/'multi_calib.yaml',config)
    dump(run/'manifest.yaml',{'schema':1,'training_scenes':args.scenes,'camera':reference['camera'],
        'lidar_frame':reference['lidar_frame'],'lidar_to_imu_translation':reference['lidar_to_imu_translation'],
        'initial_T_camera_lidar':initial.tolist(),'scene_hashes':[d['cloud_sha256'] for _,d in loaded],
        'status':'prepared_not_calibrated'})
    print('PREPARED '+str(run))


def solve(args):
    run=args.root/'runs'/safe_name(args.name)
    read(run/'manifest.yaml')
    if (run/'extrinsic.txt').exists() or (run/'solver.log').exists(): raise ValueError('Run already attempted; prepare a new run name')
    sock=socket.socket()
    try: sock.bind(('127.0.0.1',args.port))
    except OSError: raise ValueError('Private ROS master port is occupied')
    finally: sock.close()
    env=os.environ.copy(); env['ROS_MASTER_URI']='http://127.0.0.1:%d'%args.port
    env['ROS_IP']='127.0.0.1'; env.pop('ROS_HOSTNAME',None)
    env['SCOUT_CALIB_HEADLESS']='1'; env['SCOUT_CALIB_OUTPUT']=str(run)
    env['OMP_NUM_THREADS']='2'
    master=None; worker=None
    marker=run/'.solving'; marker.touch()
    try:
        with (run/'master.log').open('w') as mlog,(run/'solver.log').open('w') as slog:
            master=subprocess.Popen(['roscore','-p',str(args.port)],env=env,stdout=mlog,stderr=subprocess.STDOUT,start_new_session=True)
            import xmlrpc.client
            ready=False
            for _ in range(100):
                try:
                    ready=xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/calibration')[0]==1
                    if ready: break
                except OSError: pass
                if master.poll() is not None: break
                time.sleep(.1)
            if not ready: raise ValueError('Private master failed to start')
            subprocess.run(['rosparam','load',str(run/'multi_calib.yaml')],env=env,check=True)
            worker=subprocess.Popen(['rosrun','livox_camera_calib','lidar_camera_multi_calib'],env=env,
                stdout=slog,stderr=subprocess.STDOUT,start_new_session=True)
            print('Solving on private master. Log: '+str(run/'solver.log'),flush=True)
            worker.wait(timeout=args.timeout)
            if worker.returncode: raise ValueError('Solver rejected data or failed; inspect solver.log')
        transform=rigid_matrix(np.loadtxt(run/'extrinsic.txt',delimiter=','))
        dump(run/'result.yaml',{'status':'candidate_not_accepted','T_camera_lidar':transform.tolist(),
            'sha256':digest(run/'extrinsic.txt'),'note':'Convergence does not establish accuracy; project held-out scenes'})
        marker.unlink()
        print('CANDIDATE_READY: run project on an independent scene before accept')
    finally: stop(worker); stop(master)


def project(args):
    run=args.root/'runs'/safe_name(args.name); manifest=read(run/'manifest.yaml')
    if (run/'.solving').exists(): raise ValueError('Solver incomplete/failed')
    result=read(run/'result.yaml')
    if digest(run/'extrinsic.txt')!=result['sha256']: raise ValueError('Result changed after solve')
    transform=rigid_matrix(np.loadtxt(run/'extrinsic.txt',delimiter=','))
    p,data=scene(args,args.scene); same_camera(manifest['camera'],data['camera'])
    if data['lidar_frame']!=manifest['lidar_frame']: raise ValueError('LiDAR frame mismatch')
    image=cv2.imread(str(p/'image.bmp')); points=read_pcd(p/'cloud.pcd')[:,:3]
    pc=points@transform[:3,:3].T+transform[:3,3]; pc=pc[pc[:,2]>.1]
    pc=pc[::max(1,int(np.ceil(len(pc)/60000)))]
    if not len(pc): raise ValueError('All points behind camera; check transform direction')
    uv,_=cv2.projectPoints(pc,np.zeros(3),np.zeros(3),np.asarray(data['camera']['K']).reshape(3,3),np.asarray(data['camera']['D']))
    uv=uv.reshape(-1,2); h,w=image.shape[:2]
    inside=np.isfinite(uv).all(axis=1)&(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
    uv=uv[inside].astype(int); pc=pc[inside]; z=pc[:,2]
    overlay=image.copy(); depth=np.full((h,w),np.inf,np.float32)
    np.minimum.at(depth,(uv[:,1],uv[:,0]),z)
    visible=np.isfinite(depth)
    if visible.sum()<100: raise ValueError('Insufficient projected coverage')
    color=cv2.applyColorMap(np.uint8(np.clip(depth,0,10)*25.5),cv2.COLORMAP_TURBO)
    overlay[visible]=(.4*overlay[visible]+.6*color[visible]).astype(np.uint8)
    edge=cv2.Canny(image,40,100)
    inverse=255-edge; distance=cv2.distanceTransform(inverse,cv2.DIST_L2,3)
    # Depth discontinuities are an approximate diagnostic, not the optimizer's plane edges.
    valid_pair=visible[:,1:]&visible[:,:-1]
    delta=np.zeros((h,w),bool)
    with np.errstate(invalid='ignore'):
        delta[:,1:]=valid_pair&(np.abs(depth[:,1:]-depth[:,:-1])>.10)
        delta[1:,:]|=(visible[1:,:]&visible[:-1,:])&(np.abs(depth[1:,:]-depth[:-1,:])>.10)
    # Plane intersections can be depth-continuous. Also show local surface-normal
    # changes, without re-estimating or adjusting the candidate transform.
    geometric=np.zeros((h,w),bool)
    if len(pc)>=16:
        from scipy.spatial import cKDTree
        distances,neighbors=cKDTree(pc).query(pc,k=16)
        local=pc[neighbors]; local-=local.mean(axis=1,keepdims=True)
        covariance=np.einsum('nki,nkj->nij',local,local)/16
        values,vectors=np.linalg.eigh(covariance)
        normals=vectors[:,:,0]
        agreement=np.abs(np.einsum('ni,nki->nk',normals,normals[neighbors]))
        selected=(agreement.min(axis=1)<np.cos(np.deg2rad(25)))&(values[:,1]>1e-8)&(distances[:,-1]<.2)
        selected&=z<=depth[uv[:,1],uv[:,0]]+.05
        geometric[uv[selected,1],uv[selected,0]]=True
    combined=delta|geometric
    edge_overlay=image.copy(); edge_overlay[edge>0]=[0,255,0]; edge_overlay[combined]=[0,0,255]
    directory=run/'validation'/safe_name(args.scene); directory.mkdir(parents=True,exist_ok=True)
    if not cv2.imwrite(str(directory/'projection.png'),overlay) or not cv2.imwrite(str(directory/'edges.png'),edge_overlay): raise IOError('Projection write failed')
    errors=distance[combined]
    report={'scene':args.scene,'held_out':args.scene not in manifest['training_scenes'] and data['cloud_sha256'] not in manifest['scene_hashes'],
        'result_sha256':result['sha256'],'scene_cloud_sha256':data['cloud_sha256'],
        'visible_pixels':int(visible.sum()),'depth_edge_pixels':int(delta.sum()),
        'normal_change_edge_pixels':int(geometric.sum()),
        'edge_distance_median_px':float(np.median(errors)) if len(errors) else None,
        'note':'Green=image edges; red=approximate depth/normal-change edges, max 60000 projected samples. Occlusion, density and texture affect this metric; not an accuracy certificate'}
    dump(directory/'report.yaml',report)
    print(json.dumps(report,ensure_ascii=False)); print('Review '+str(directory))


def accept(args):
    if not args.confirm_validation: raise ValueError('Review independent overlays first, then explicitly use --confirm-validation')
    run=args.root/'runs'/safe_name(args.name); manifest=read(run/'manifest.yaml'); result=read(run/'result.yaml')
    if (run/'.solving').exists() or digest(run/'extrinsic.txt')!=result['sha256']: raise ValueError('Incomplete or modified result')
    reports=[read(p) for p in (run/'validation').glob('*/report.yaml')]
    reports=[r for r in reports if r['held_out'] and r['result_sha256']==result['sha256'] and r['visible_pixels']>=100]
    if not reports: raise ValueError('At least one independent scene projection is required')
    target=run/'accepted_calibration.yaml'
    if target.exists(): raise ValueError('Acceptance already recorded; use a new run for changes')
    value={'schema':1,'status':'operator_accepted','convention':'p_camera_optical = T_camera_lidar * p_lidar',
        'camera':manifest['camera'],'lidar_frame':manifest['lidar_frame'],
        'lidar_to_imu_translation':manifest['lidar_to_imu_translation'],
        'T_camera_lidar':rigid_matrix(np.loadtxt(run/'extrinsic.txt',delimiter=',')).tolist(),
        'result_sha256':result['sha256'],'independent_reviews':reports,'accepted_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
        'time_calibration':'not_estimated','note':'Operator spatial acceptance only; no automatic navigation integration'}
    dump(target,value)
    print('Saved '+str(target)); print('~/r3live_ws/start_scout_r3live.sh calibration_file:='+str(target))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=R3/'calibration')
    subs=parser.add_subparsers(dest='command',required=True)
    subs.add_parser('sensors')
    p=subs.add_parser('capture'); p.add_argument('name'); p.add_argument('--seconds',type=int,default=6); p.add_argument('--stationary',action='store_true')
    p=subs.add_parser('export'); p.add_argument('name'); p.add_argument('bags',nargs='+',type=Path); p.add_argument('--stationary',action='store_true'); p.add_argument('--seconds',type=int,default=6)
    p=subs.add_parser('prepare'); p.add_argument('name'); p.add_argument('scenes',nargs='+')
    p=subs.add_parser('solve'); p.add_argument('name'); p.add_argument('--port',type=int,default=11441); p.add_argument('--timeout',type=int,default=1800)
    p=subs.add_parser('project'); p.add_argument('name'); p.add_argument('scene')
    p=subs.add_parser('accept'); p.add_argument('name'); p.add_argument('--confirm-validation',action='store_true')
    args=parser.parse_args(); args.root=args.root.expanduser().resolve()
    if args.command in ('capture','export') and not 3<=args.seconds<=10: parser.error('Duration must be 3..10 seconds')
    try: globals()[args.command](args)
    except KeyboardInterrupt: print('Cancelled; incomplete markers retained.',file=sys.stderr); return 130
    except Exception as e: print('ERROR: '+str(e),file=sys.stderr); return 1
    return 0


if __name__=='__main__': sys.exit(main())
