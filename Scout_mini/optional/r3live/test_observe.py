#!/usr/bin/env python3
"""Read-only finite pose/stamp/frame check. This is NOT an accuracy metric."""
import json
import argparse
import math
import time
from pathlib import Path
import rospy
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Image

parser=argparse.ArgumentParser()
parser.add_argument('--duration',type=float,default=65)
parser.add_argument('--require-vision',action='store_true')
parser.add_argument('--output',default='/tmp/scout_r3live_replay_result.json')
args=parser.parse_args()
rospy.init_node('r3live_pose_contract_test')
samples=[]
paths=[]
cameras=[]
images=[]
rospy.Subscriber('/r3live/odometry',Odometry,samples.append,queue_size=1000)
rospy.Subscriber('/r3live/path',RosPath,lambda m: paths.append(m.header.frame_id),queue_size=1)
rospy.Subscriber('/r3live/camera_odometry',Odometry,cameras.append,queue_size=1000)
rospy.Subscriber('/r3live/track_image',Image,lambda m: images.append(m.header.stamp.to_sec()),queue_size=2)
end=time.monotonic()+args.duration
while time.monotonic()<end and not rospy.is_shutdown(): time.sleep(.1)
assert len(samples)>20,'Too few odometry samples: '+str(len(samples))
stamps=[m.header.stamp.to_sec() for m in samples]
assert all(b>a for a,b in zip(stamps,stamps[1:])),'Non-monotonic stamps'
assert all(m.header.frame_id=='r3live_world' and m.child_frame_id=='r3live_imu' for m in samples)
if args.require_vision:
    assert len(cameras)>10 and len(images)>10,'Missing visual outputs'
    assert all(m.header.frame_id=='r3live_world' and m.child_frame_id=='r3live_camera_optical' for m in cameras)
    cam_stamps=[m.header.stamp.to_sec() for m in cameras]
    assert all(b>a for a,b in zip(cam_stamps,cam_stamps[1:])),'Non-monotonic camera stamps'
assert paths and set(paths)=={'r3live_world'},'Wrong or missing path frame'
for m in samples+cameras:
    p=m.pose.pose.position; q=m.pose.pose.orientation
    assert all(math.isfinite(x) for x in [p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    assert abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<1e-3
result={'result':'PASS','odom_samples':len(samples),'source_time_span_s':stamps[-1]-stamps[0],
    'frames':['r3live_world','r3live_imu'],'path_frames':sorted(set(paths)),
    'camera_pose_samples':len(cameras),'tracking_image_samples':len(images),
    'scope':'Odometry contract only; this script does not assess visual updates or accuracy'}
Path(args.output).write_text(json.dumps(result,indent=2))
print(json.dumps(result))
