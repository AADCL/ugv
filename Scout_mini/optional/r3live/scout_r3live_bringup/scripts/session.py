#!/usr/bin/env python3
"""Independent test entry; no motion commands, no automatic takeover of running nodes."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rosnode
import rospkg
import rospy
from sensor_msgs.msg import CameraInfo, Image, Imu
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg
import tf
import yaml


def transform(description):
    xyz=np.asarray(description['xyz'], dtype=float)
    rpy=np.asarray(description['rpy_degrees'], dtype=float)
    if xyz.shape!=(3,) or rpy.shape!=(3,) or not np.isfinite(np.r_[xyz,rpy]).all():
        raise ValueError('Invalid rig geometry')
    matrix=tf.transformations.euler_matrix(*np.radians(rpy))
    matrix[:3,3]=xyz
    return matrix


def main():
    rospy.init_node('scout_r3live_session')
    processes=[]
    handles=[]
    try:
        nodes=rosnode.get_node_names()
        forbidden={'laserMapping','scout_shadow_ekf','scout_fusion_guard','move_base',
                   'scout_global_localizer','r3live_mapping','r3live_lidar_front_end'}
        conflicts=[n for n in nodes if n.rsplit('/',1)[-1] in forbidden]
        if rospy.get_param('~start_lidar',True):
            conflicts += [n for n in nodes if 'livox_lidar_publisher' in n]
        if rospy.get_param('~start_camera',True):
            conflicts += [n for n in nodes if n.endswith('/realsense2_camera') or n.endswith('/camera')]
        if conflicts:
            raise RuntimeError('Stop the owning test/launch first; existing nodes: '+', '.join(sorted(set(conflicts))))
        package=Path(rospkg.RosPack().get_path('scout_r3live_bringup'))
        rospkg.RosPack().get_path('r3live')
        rospkg.RosPack().get_path('realsense2_camera')
        rig_file=Path(rospy.get_param('~rig_file',str(package/'config/rig.yaml')))
        rig=yaml.safe_load(rig_file.read_text())
        base_imu=transform(rig['base_to_imu'])
        base_camera=transform(rig['base_to_camera_link'])
        if rospy.get_param('~check_only',False):
            rospy.loginfo('PREFLIGHT_OK; no sensors or estimator started. Calibration: %s',rig['calibration_status'])
            return 0
        root=Path(rospy.get_param('~output_root',str(Path.home()/'r3live_ws/logs'))).expanduser()
        root.mkdir(parents=True,exist_ok=True)
        directory=Path(tempfile.mkdtemp(prefix=time.strftime('%Y%m%d_%H%M%S_'),dir=str(root)))
        rospy.logwarn('R3LIVE evaluation only; calibration=%s. Not connected to navigation. Logs: %s',rig['calibration_status'],directory)

        def launch(name,arguments):
            handle=(directory/(name+'.log')).open('w')
            handles.append(handle)
            process=subprocess.Popen(['roslaunch','scout_r3live_bringup',name+'.launch']+arguments,
                                     stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(process)
            return process

        start_lidar=rospy.get_param('~start_lidar',True)
        start_camera=rospy.get_param('~start_camera',True)
        sensors=None
        if start_lidar or start_camera:
            sensors=launch('sensors',['start_lidar:='+str(start_lidar).lower(),
                                      'start_camera:='+str(start_camera).lower()])
        listener=tf.TransformListener()
        rospy.loginfo('Waiting for camera calibration and image (up to 45 seconds); keep stationary.')
        info=rospy.wait_for_message('/r3live_camera/color/camera_info',CameraInfo,timeout=45)
        image=rospy.wait_for_message('/r3live_camera/color/image_raw',Image,timeout=15)
        imu=rospy.wait_for_message('/livox/imu',Imu,timeout=15)
        lidar=rospy.wait_for_message('/livox/lidar',CustomMsg,timeout=15)
        if not lidar.points or lidar.point_num!=len(lidar.points):
            raise ValueError('Missing or malformed Mid-360 cloud')
        if min(image.header.stamp.to_sec(),imu.header.stamp.to_sec(),lidar.header.stamp.to_sec())<=0:
            raise ValueError('Sensor timestamp is zero')
        if info.width!=image.width or info.height!=image.height or info.K[0]<=0 or info.K[4]<=0:
            raise ValueError('CameraInfo and image dimensions/intrinsics do not agree')
        if info.distortion_model not in ('plumb_bob','') or len(info.D) not in (0,5):
            raise ValueError('Expected a calibrated pinhole camera with 5 Brown-Conrady coefficients')
        if abs(image.header.stamp.to_sec()-imu.header.stamp.to_sec())>1.0:
            raise ValueError('Camera and IMU clocks differ by >1 second; fix synchronization')
        if not all(math.isfinite(x) for x in list(info.K)+list(info.D)):
            raise ValueError('Non-finite camera calibration')
        listener.waitForTransform('r3live_camera_link',info.header.frame_id,rospy.Time(0),rospy.Duration(10))
        xyz,quat=listener.lookupTransform('r3live_camera_link',info.header.frame_id,rospy.Time(0))
        camera_optical=tf.transformations.quaternion_matrix(quat)
        camera_optical[:3,3]=xyz
        # R3LIVE uses T_world_camera = T_world_imu * T_imu_camera.
        imu_camera=np.linalg.inv(base_imu).dot(base_camera).dot(camera_optical)
        config=yaml.safe_load((package/'config/estimator.yaml').read_text())
        config['r3live_vio']={'image_width':info.width,'image_height':info.height,
            'camera_intrinsic':list(info.K),'camera_dist_coeffs':list(info.D) or [0.]*5,
            'camera_ext_R':imu_camera[:3,:3].reshape(-1).tolist(),
            'camera_ext_t':imu_camera[:3,3].tolist()}
        config['r3live_common']['map_output_dir']=str(directory/'output')
        (directory/'output').mkdir()
        runtime=directory/'runtime.yaml'
        runtime.write_text(yaml.safe_dump(config,sort_keys=False))
        (directory/'rig_snapshot.yaml').write_text(yaml.safe_dump(rig))
        (directory/'sensor_snapshot.json').write_text(json.dumps({
            'camera_info_frame':info.header.frame_id,'camera_stamp':image.header.stamp.to_sec(),
            'imu_stamp':imu.header.stamp.to_sec(),'calibration_status':rig['calibration_status'],
            'meaning':'Startup clock sanity only; not temporal or geometric calibration'},indent=2))
        estimator=launch('estimator',['runtime_config:='+str(runtime)])
        rospy.wait_for_message('/r3live/odometry',Odometry,timeout=45)
        rospy.wait_for_message('/r3live/camera_odometry',Odometry,timeout=45)
        rospy.loginfo('LIO and camera pose outputs received. Inspect /r3live/odometry and /r3live/camera_odometry; startup is not accuracy validation.')
        while not rospy.is_shutdown():
            if estimator.poll() is not None or (sensors is not None and sensors.poll() is not None):
                raise RuntimeError('A child launch exited; inspect session logs')
            time.sleep(.2)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        if not rospy.is_shutdown():
            rospy.logerr('R3LIVE session: %s',error)
        return 1
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try: process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM)
                    try: process.wait(timeout=5)
                    except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL)
        for handle in handles: handle.close()


if __name__=='__main__':
    raise SystemExit(main())
