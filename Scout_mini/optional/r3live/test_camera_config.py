#!/usr/bin/env python3
"""Read factory calibration on a private master; emit replay runtime config."""
import json
from pathlib import Path
import sys
import numpy as np
import rospy
import rospkg
import tf
import yaml
from sensor_msgs.msg import CameraInfo, Image

rospy.init_node('r3live_camera_contract_test')
listener=tf.TransformListener()
info=rospy.wait_for_message('/r3live_camera/color/camera_info',CameraInfo,45)
image=rospy.wait_for_message('/r3live_camera/color/image_raw',Image,15)
assert (info.width,info.height)==(640,480)
assert (image.width,image.height)==(640,480)
assert info.K[0]>0 and info.K[4]>0 and len(info.D)==5
listener.waitForTransform('r3live_camera_link',info.header.frame_id,rospy.Time(0),rospy.Duration(10))
xyz,quat=listener.lookupTransform('r3live_camera_link',info.header.frame_id,rospy.Time(0))
package=Path(rospkg.RosPack().get_path('scout_r3live_bringup'))
sys.path.insert(0,str(package/'scripts'))
from session import transform
rig=yaml.safe_load((package/'config/rig.yaml').read_text())
optical=tf.transformations.quaternion_matrix(quat); optical[:3,3]=xyz
ext=np.linalg.inv(transform(rig['base_to_imu'])).dot(transform(rig['base_to_camera_link'])).dot(optical)
config=yaml.safe_load((package/'config/estimator.yaml').read_text())
config['r3live_vio']={'image_width':info.width,'image_height':info.height,
    'camera_intrinsic':list(info.K),'camera_dist_coeffs':list(info.D),
    'camera_ext_R':ext[:3,:3].reshape(-1).tolist(),'camera_ext_t':ext[:3,3].tolist()}
config['r3live_common']['map_output_dir']='/tmp/scout_r3live_replay_output'
Path('/tmp/scout_r3live_replay_output').mkdir(exist_ok=True)
Path('/tmp/scout_r3live_test_runtime.yaml').write_text(yaml.safe_dump(config))
print(json.dumps({'result':'PASS','image_encoding':image.encoding,'size':[info.width,info.height],
    'optical_frame':info.header.frame_id,'K':list(info.K),'D':list(info.D),
    'extrinsic_status':'approximate, not calibrated'}))
