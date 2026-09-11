#!/usr/bin/env python3
"""Independent test entry; no motion commands, no automatic takeover of running nodes."""
import json
import math
import os
from pathlib import Path
import signal
import shutil
import sys
import subprocess
import tempfile
import time

import numpy as np
import rosnode
import rosgraph
import rospkg
import rospy
from sensor_msgs.msg import CameraInfo, Image, Imu
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg
import tf
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parent))
from calibration_io import load_accepted
from runtime_health import RuntimeHealth, SENSORS, ALL_STREAMS


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
    subscribers=[]
    directory=None
    health=RuntimeHealth()
    outcome='failed'
    error_text=None
    try:
        if rospy.get_param('/use_sim_time',False):
            raise RuntimeError('Live session requires wall time; use estimator.launch for isolated bag replay')
        nodes=rosnode.get_node_names()
        project_interface=rospy.get_param('~project_interface',True)
        forbidden={'laserMapping','scout_shadow_ekf','scout_fusion_guard','move_base',
                   'scout_global_localizer','r3live_mapping','r3live_lidar_front_end'}
        if project_interface:
            forbidden.update({'scout_r3live_project_interface','scout_tf_manager',
                              'scout_geometry_tf_publisher','scout_pose_adapter',
                              'scout_cloud_adapter','scout_pointcloud_mapper'})
        conflicts=[n for n in nodes if n.rsplit('/',1)[-1] in forbidden]
        if rospy.get_param('~start_lidar',True):
            conflicts += [n for n in nodes if 'livox_lidar_publisher' in n]
        if rospy.get_param('~start_camera',True):
            conflicts += [n for n in nodes if n.endswith('/realsense2_camera') or
                          n.endswith('/camera') or n.endswith('/realsense2_camera_manager')]
        if project_interface:
            publishers,_,_=rosgraph.Master(rospy.get_name()).getSystemState()
            owned={'/Odometry','/fastlio_odom','/cloud_registered',
                   '/cloud_registered_body','/cloud_registered_base'}
            conflicts += [topic+' owned by '+','.join(owners)
                          for topic,owners in publishers if topic in owned and owners]
        if conflicts:
            raise RuntimeError('Stop the owning test/launch first; existing nodes: '+', '.join(sorted(set(conflicts))))
        package=Path(rospkg.RosPack().get_path('scout_r3live_bringup'))
        rospkg.RosPack().get_path('r3live')
        rospkg.RosPack().get_path('realsense2_camera')
        rig_file=Path(rospy.get_param('~rig_file',str(package/'config/rig.yaml')))
        rig=yaml.safe_load(rig_file.read_text())
        base_imu=transform(rig['base_to_imu'])
        base_camera=transform(rig['base_to_camera_link'])
        calibration_file=rospy.get_param('~calibration_file','')
        if calibration_file and not Path(calibration_file).expanduser().is_file():
            raise ValueError('Selected calibration file is missing: '+calibration_file)
        record_bag=rospy.get_param('~record_bag',False)
        test_duration=float(rospy.get_param('~test_duration',600.0))
        if not math.isfinite(test_duration) or not 30 <= test_duration <= 1800:
            raise ValueError('test_duration must be 30..1800 seconds')
        if rospy.get_param('~check_only',False):
            rospy.loginfo('PREFLIGHT_OK; configuration only, no live checks. Calibration: %s; record_bag=%s',
                          calibration_file or rig['calibration_status'],record_bag)
            outcome='preflight_only'
            return 0
        root=Path(rospy.get_param('~output_root',str(Path.home()/'r3live_ws/logs'))).expanduser()
        root.mkdir(parents=True,exist_ok=True)
        directory=Path(tempfile.mkdtemp(prefix=time.strftime('%Y%m%d_%H%M%S_'),dir=str(root)))
        minimum_free=3*1024**3+int((test_duration+240)*24*1024**2) if record_bag else 512*1024**2
        if shutil.disk_usage(directory).free < minimum_free:
            raise RuntimeError('Insufficient free disk space for bounded test recording')
        rospy.logwarn('R3LIVE evaluation; project_interface=%s; NDT/navigation are not started. Logs: %s',
                      project_interface,directory)

        def spawn(name,command):
            handle=(directory/(name+'.log')).open('w')
            handles.append(handle)
            process=subprocess.Popen(command,
                                     stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(process)
            return process

        def launch(name,arguments):
            return spawn(name,['roslaunch','scout_r3live_bringup',name+'.launch']+arguments)

        def receive(name,msg):
            # Copy receipt/source scalars immediately; never edit the ROS message.
            now=rospy.Time.now().to_sec()
            mono=time.monotonic()
            pose=None
            if name in ('lio','camera'):
                p=msg.pose.pose.position; q=msg.pose.pose.orientation
                pose=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
            health.observe(name,msg.header.stamp.to_sec(),now,mono,msg.header.frame_id,
                           getattr(msg,'child_frame_id',''),pose)

        for name,topic,kind in (
            ('image','/r3live_camera/color/image_raw',Image),('imu','/livox/imu',Imu),
            ('lidar','/livox/lidar',CustomMsg),('lio','/r3live/odometry',Odometry),
            ('camera','/r3live/camera_odometry',Odometry)):
            subscribers.append(rospy.Subscriber(topic,kind,lambda m,n=name:receive(n,m),
                                               queue_size=10,buff_size=4*1024**2,tcp_nodelay=True))

        def wait_stable(names,seconds):
            deadline=time.monotonic()+seconds
            while not rospy.is_shutdown() and time.monotonic()<deadline:
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError('A child process exited during startup')
                problems=health.problems(names,time.monotonic(),require_stable=True)
                if not problems:
                    return
                rospy.loginfo_throttle(5,'WARMUP: '+'; '.join(problems))
                time.sleep(.1)
            raise RuntimeError('Continuous stream checks not ready: '+
                               '; '.join(health.problems(names,time.monotonic(),True)))

        start_lidar=rospy.get_param('~start_lidar',True)
        start_camera=rospy.get_param('~start_camera',True)
        sensors=None
        if start_lidar or start_camera:
            sensors=launch('sensors',['start_lidar:='+str(start_lidar).lower(),
                                      'start_camera:='+str(start_camera).lower()])
        recorder=None
        if record_bag:
            recorder=spawn('rosbag',['rosbag','record','--split','--size=1024','--buffsize=128',
                '--min-space=2G','-O',str(directory/'sensors'),
                '/livox/lidar','/livox/imu','/r3live_camera/color/image_raw',
                '/r3live_camera/color/camera_info','/r3live_camera/color/metadata','/tf_static',
                '/r3live/odometry','/r3live/camera_odometry','/rosout'] +
                (['/tf','/r3live/tf_raw','/Odometry','/fastlio_odom'] if project_interface else []))
        wait_stable(SENSORS,60)
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
        # Concurrent health subscriptions check each source against receipt time.
        # Sequential wait_for_message samples need not represent the same instant.
        if not all(math.isfinite(x) for x in list(info.K)+list(info.D)):
            raise ValueError('Non-finite camera calibration')
        listener.waitForTransform('r3live_camera_link',info.header.frame_id,rospy.Time(0),rospy.Duration(10))
        xyz,quat=listener.lookupTransform('r3live_camera_link',info.header.frame_id,rospy.Time(0))
        camera_optical=tf.transformations.quaternion_matrix(quat)
        camera_optical[:3,3]=xyz
        # R3LIVE uses T_world_camera = T_world_imu * T_imu_camera.
        imu_camera=np.linalg.inv(base_imu).dot(base_camera).dot(camera_optical)
        config=yaml.safe_load((package/'config/estimator.yaml').read_text())
        calibration_record=None
        if calibration_file:
            imu_camera,calibration_record=load_accepted(
                calibration_file,info,config['r3live_lio']['lidar_to_imu_translation'])
            if calibration_record['lidar_frame']!=lidar.header.frame_id:
                raise ValueError('LiDAR frame changed since spatial calibration')
            rospy.loginfo('Using operator-accepted spatial calibration: %s; time offset is not calibrated by this file',calibration_file)
        config['r3live_vio']={'image_width':info.width,'image_height':info.height,
            'camera_intrinsic':list(info.K),'camera_dist_coeffs':list(info.D) or [0.]*5,
            'camera_ext_R':imu_camera[:3,:3].reshape(-1).tolist(),
            'camera_ext_t':imu_camera[:3,3].tolist()}
        # Reuse the exact selected camera calibration, including the optical/link
        # rotation read from the camera driver. Never substitute measured mount values.
        config['scout_project_interface']={
            'imu_to_camera_optical':imu_camera.reshape(-1).tolist(),
            'imu_to_camera_link':(imu_camera@np.linalg.inv(camera_optical)).reshape(-1).tolist()}
        config['r3live_common']['map_output_dir']=str(directory/'output')
        (directory/'output').mkdir()
        runtime=directory/'runtime.yaml'
        runtime.write_text(yaml.safe_dump(config,sort_keys=False))
        (directory/'rig_snapshot.yaml').write_text(yaml.safe_dump(rig))
        if calibration_record is not None:
            (directory/'calibration_snapshot.yaml').write_text(yaml.safe_dump(calibration_record))
        (directory/'sensor_snapshot.json').write_text(json.dumps({
            'camera_info_frame':info.header.frame_id,'camera_stamp':image.header.stamp.to_sec(),
            'imu_stamp':imu.header.stamp.to_sec(),'calibration_status':
                'operator_accepted_spatial_only' if calibration_record else rig['calibration_status'],
            'meaning':'Startup clock sanity only; not temporal or geometric calibration'},indent=2))
        estimator=launch('estimator',['runtime_config:='+str(runtime),
                                     'project_interface:='+str(project_interface).lower()])
        wait_stable(ALL_STREAMS,60)
        if project_interface:
            aligned=rospy.wait_for_message('/fastlio_odom',Odometry,timeout=5)
            if (aligned.header.frame_id,aligned.child_frame_id)!=('odom','base_link'):
                raise RuntimeError('Project interface odometry frame mismatch')
            listener.waitForTransform('odom','base_link',aligned.header.stamp,rospy.Duration(5))
        if recorder is not None and not any(p.stat().st_size > 1024
                                           for p in directory.glob('*.bag*')):
            raise RuntimeError('Recorder has not created a non-empty bag')
        health.arm(time.monotonic())
        ready_time=time.monotonic()
        (directory/'ready.json').write_text(json.dumps(health.snapshot(ready_time),indent=2))
        rospy.loginfo('READY: continuous LIO/visual poses and source clocks checked. Manual low-speed test only; no accuracy certification. Recording=%s; duration=%.0fs',record_bag,test_duration)
        report_time=ready_time
        while not rospy.is_shutdown():
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('A child launch/recorder exited; inspect session logs')
            now=time.monotonic()
            fault=health.check(now)
            if fault:
                raise RuntimeError('RUNTIME_INVALID: '+json.dumps(fault))
            if record_bag and shutil.disk_usage(directory).free < 3*1024**3:
                raise RuntimeError('Disk reserve reached; stopping test and closing bag')
            if now-ready_time >= test_duration:
                outcome='duration_complete'
                rospy.logwarn('Test duration complete; ending estimation and recording. Park manually.')
                break
            if now-report_time >= 5:
                (directory/'health.json').write_text(json.dumps(health.snapshot(now),indent=2))
                report_time=now
            time.sleep(.2)
        if outcome!='duration_complete':
            outcome='operator_stopped'
        return 0
    except (Exception, KeyboardInterrupt) as error:
        error_text=str(error)
        if rospy.is_shutdown():
            outcome='operator_stopped'
        if not rospy.is_shutdown():
            rospy.logerr('R3LIVE session: %s. Park manually; this node cannot stop the vehicle.',error)
        return 1
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try: process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM)
                    try: process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGKILL)
                        process.wait(timeout=5)
        for handle in handles: handle.close()
        for subscriber in subscribers: subscriber.unregister()
        if directory is not None:
            report=health.snapshot(time.monotonic())
            report.update(outcome=outcome,error=error_text,
                          bags=[p.name for p in directory.glob('*.bag')],
                          incomplete_bags=[p.name for p in directory.glob('*.bag.active')])
            (directory/'session_result.json').write_text(json.dumps(report,indent=2))


if __name__=='__main__':
    raise SystemExit(main())
