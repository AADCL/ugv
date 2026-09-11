#!/usr/bin/env python3
"""Synthetic ROS integration test. Uses a private master; never starts hardware or records bags."""
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest
import xmlrpc.client

import numpy as np
import yaml


class InterfaceTest(unittest.TestCase):
    def test_frames_clouds_timestamps_and_fault(self):
        import rospy
        import tf
        import roslaunch
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2, PointField
        from tf2_msgs.msg import TFMessage

        workspace = Path('/home/nvidia/r3live_ws')
        package = workspace/'src/scout_r3live_bringup'
        accepted = yaml.safe_load((workspace/'config/accepted_20260911/trial_calibration.yaml').read_text())
        internal = np.eye(4)
        internal[:3, 3] = accepted['lidar_to_imu_translation']
        imu_optical = internal @ np.linalg.inv(accepted['T_camera_lidar'])
        # Synthetic, nontrivial camera factory transform tests the optical/link inversion.
        link_optical = tf.transformations.euler_matrix(-np.pi/2, 0, -np.pi/2)
        link_optical[:3, 3] = [0.001, 0.014, -0.002]
        runtime = {
            'r3live_lio': {'lidar_to_imu_translation': accepted['lidar_to_imu_translation']},
            'scout_project_interface': {'imu_to_camera_optical': imu_optical.reshape(-1).tolist(),
                'imu_to_camera_link': (imu_optical @ np.linalg.inv(link_optical)).reshape(-1).tolist()}}
        with tempfile.TemporaryDirectory(prefix='scout_interface_test_') as directory:
            directory = Path(directory)
            runtime_path = directory/'runtime.yaml'
            runtime_path.write_text(yaml.safe_dump(runtime))
            # Resolve both launch modes without starting R3LIVE or sensor drivers.
            for enabled, expected in ((True, '/r3live/tf_raw'), (False, '/tf')):
                resolved = roslaunch.config.load_config_default([
                    (str(package/'launch/estimator.launch'), ['runtime_config:='+str(runtime_path),
                     'project_interface:='+str(enabled).lower()])], None)
                mapping = next(n for n in resolved.nodes if n.name == 'r3live_mapping')
                self.assertIn(('/tf', expected), [tuple(pair) for pair in mapping.remap_args])
                self.assertEqual(any(n.name == 'scout_r3live_project_interface' for n in resolved.nodes), enabled)
            log = (directory/'launch.log').open('w')
            process = subprocess.Popen(['roslaunch', 'scout_r3live_bringup', 'project_interface.launch',
                                        'runtime_config:='+str(runtime_path)],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                rospy.init_node('scout_interface_test', anonymous=True, disable_signals=True)
                received = {}
                edges = {}
                def receive(topic, msg):
                    received.setdefault(topic, []).append(msg)
                def receive_tf(msg):
                    for tr in msg.transforms:
                        edges.setdefault(tr.child_frame_id, set()).add(tr.header.frame_id)
                subscribers = [rospy.Subscriber(topic, kind, lambda m,t=topic:receive(t,m), queue_size=50)
                    for topic,kind in [('/Odometry',Odometry),('/fastlio_odom',Odometry),
                        ('/cloud_registered',PointCloud2),('/cloud_registered_body',PointCloud2),
                        ('/cloud_registered_base',PointCloud2)]]
                subscribers += [rospy.Subscriber(topic,TFMessage,receive_tf,queue_size=50)
                                for topic in ('/tf','/tf_static')]
                cloud_pub = rospy.Publisher('/r3live/cloud_registered',PointCloud2,queue_size=5)
                odom_pub = rospy.Publisher('/r3live/odometry',Odometry,queue_size=5)
                listener = tf.TransformListener()
                deadline=time.monotonic()+15
                while time.monotonic()<deadline and (not cloud_pub.get_num_connections() or
                        not odom_pub.get_num_connections() or not all(s.get_num_connections() for s in subscribers[:5])):
                    time.sleep(.05)
                self.assertTrue(cloud_pub.get_num_connections() and odom_pub.get_num_connections())
                self.assertTrue(all(s.get_num_connections() for s in subscribers[:5]))
                # Two unrelated yaw frames and a translated IMU mount expose relabel-only bugs.
                base_imu=tf.transformations.euler_matrix(0,np.pi/4,0)
                base_imu[:3,3]=[.25,0,.20]
                world_odom=tf.transformations.euler_matrix(.05,-.12,.73)
                world_odom[:3,3]=[12,-7,2]
                points=np.array([[2,.5,-.2,1],[1,-.7,.1,1]])
                stamp=rospy.Time.now()
                def messages(base_pose,stamp):
                    world_imu=world_odom@base_pose@base_imu
                    odom=Odometry()
                    odom.header.stamp=stamp; odom.header.frame_id='r3live_world'; odom.child_frame_id='r3live_imu'
                    odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=world_imu[:3,3]
                    q=tf.transformations.quaternion_from_matrix(world_imu)
                    odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w=q
                    pc=PointCloud2(); pc.header.stamp=stamp; pc.header.frame_id='r3live_world'
                    pc.height=2; pc.width=1; pc.point_step=16; pc.row_step=24; pc.is_dense=True
                    pc.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                               for i,n in enumerate(('x','y','z','intensity'))]
                    wp=(world_odom@base_pose@points.T).T
                    pc.data=b''.join(struct.pack('<ffff',*p[:3],float(41+i))+b'PADDING!'
                                     for i,p in enumerate(wp))
                    return pc,odom
                def publish(base_pose,stamp):
                    pc,od=messages(base_pose,stamp)
                    cloud_pub.publish(pc)  # R3LIVE publishes scan before matching odometry.
                    time.sleep(.02)
                    odom_pub.publish(od)
                    return pc
                def wait_topic(topic,count):
                    deadline=time.monotonic()+2
                    while time.monotonic()<deadline and len(received.get(topic,[]))<count:
                        time.sleep(.01)
                    self.assertGreaterEqual(len(received.get(topic,[])),count,topic)
                    return received[topic][-1]
                publish(np.eye(4),stamp)
                initial=wait_topic('/fastlio_odom',1)
                self.assertTrue(np.allclose([initial.pose.pose.position.x,initial.pose.pose.position.y,
                                            initial.pose.pose.position.z],0,atol=1e-8))
                self.assertAlmostEqual(abs(initial.pose.pose.orientation.w),1,places=7)
                # Y motion and a turn: forward direction is not the startup world X axis.
                base_pose=tf.transformations.euler_matrix(0,0,np.pi/2)
                base_pose[:3,3]=[0,3,0]
                stamp2=stamp+rospy.Duration(.1)
                original=publish(base_pose,stamp2)
                for topic,expected,frame in (
                    ('/cloud_registered_base',points[:,:3],'base_link'),
                    ('/cloud_registered_body',(np.linalg.inv(base_imu)@points.T).T[:,:3],'body'),
                    ('/cloud_registered',(np.linalg.inv(base_imu)@base_pose@points.T).T[:,:3],'camera_init')):
                    pc=wait_topic(topic,2)
                    actual=[struct.unpack_from('<ffff',pc.data,row*pc.row_step) for row in range(pc.height)]
                    self.assertTrue(np.allclose(np.array(actual)[:,:3],expected,atol=3e-6),(topic,actual,expected))
                    self.assertEqual([a[3] for a in actual],[41.,42.])
                    self.assertEqual(pc.data[16:24],b'PADDING!')
                    self.assertEqual(pc.data[40:48],b'PADDING!')
                    self.assertEqual(pc.fields,original.fields)
                    self.assertEqual((pc.width,pc.height,pc.point_step,pc.row_step),(1,2,16,24))
                    self.assertEqual(pc.header.stamp,stamp2); self.assertEqual(pc.header.frame_id,frame)
                base=wait_topic('/fastlio_odom',2)
                self.assertEqual((base.header.frame_id,base.child_frame_id),('odom','base_link'))
                self.assertTrue(np.allclose([base.pose.pose.position.x,base.pose.pose.position.y,
                                            base.pose.pose.position.z],[0,3,0],atol=1e-8))
                self.assertEqual(base.twist.covariance[0],1e6)
                self.assertEqual(wait_topic('/Odometry',2).header.frame_id,'camera_init')
                listener.waitForTransform('odom','base_link',stamp2,rospy.Duration(1))
                xyz,q=listener.lookupTransform('odom','base_link',stamp2)
                self.assertTrue(np.allclose(xyz,[0,3,0],atol=1e-8))
                listener.waitForTransform('body','r3live_camera_optical',rospy.Time(0),rospy.Duration(1))
                xyz,q=listener.lookupTransform('body','r3live_camera_optical',rospy.Time(0))
                self.assertTrue(np.allclose(xyz,imu_optical[:3,3]))
                self.assertEqual(edges['body'],{'camera_init'})
                self.assertEqual(edges['r3live_imu'],{'body'})
                self.assertEqual(edges['base_link'],{'body'})
                self.assertEqual(edges['terrain_sensor'],{'base_link'})
                self.assertEqual(edges['livox'],{'livox_frame'})
                self.assertNotIn('map',edges)
                self.assertTrue(all(len(parents)==1 for parents in edges.values()))
                # A mismatched pair must never be transformed using latest pose.
                pc,od=messages(base_pose,stamp+rospy.Duration(.2))
                od.header.stamp=stamp+rospy.Duration(.25)
                cloud_pub.publish(pc); odom_pub.publish(od); time.sleep(.15)
                self.assertEqual(len(received['/fastlio_odom']),2)
                # A zero-norm quaternion is rejected before any public outputs/TF.
                pc,od=messages(base_pose,stamp+rospy.Duration(.3))
                od.pose.pose.orientation.w=od.pose.pose.orientation.x=0
                od.pose.pose.orientation.y=od.pose.pose.orientation.z=0
                cloud_pub.publish(pc); odom_pub.publish(od)
                deadline=time.monotonic()+5
                while process.poll() is None and time.monotonic()<deadline: time.sleep(.05)
                self.assertIsNotNone(process.poll())
                self.assertEqual(len(received['/fastlio_odom']),2)
                print('PASS: rotated startup, Y motion/turn, exact time pairing, three cloud frames, '
                      'point fields/padding, calibrated camera TF, unique TF parents, invalid-pose shutdown')
                for sub in subscribers: sub.unregister()
            finally:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
                    process.wait(timeout=15)
                log.close()
                print((directory/'launch.log').read_text()[-2000:])


if __name__=='__main__':
    # Refuse an occupied port; never attach the test publishers to the robot master.
    with socket.socket() as probe:
        probe.bind(('127.0.0.1',11329))
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:11329'
    os.environ['ROS_HOSTNAME']='127.0.0.1'
    os.environ.pop('ROS_IP',None)
    with tempfile.TemporaryFile(mode='w+') as log:
        master=subprocess.Popen(['roscore','-p','11329'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            for _ in range(100):
                try:
                    if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/test')[0]==1: break
                except OSError: pass
                time.sleep(.05)
            result=unittest.main(exit=False).result
        finally:
            os.killpg(master.pid,signal.SIGINT)
            master.wait(timeout=15)
    sys.exit(0 if result.wasSuccessful() else 1)
