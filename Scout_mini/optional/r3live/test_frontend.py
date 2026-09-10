#!/usr/bin/env python3
"""Run only against an isolated master; exercise Mid-360 conversion contract."""
import math
import time
import rospy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

rospy.init_node('r3live_frontend_contract_test')
received=[]
sub=rospy.Subscriber('/r3live/laser_cloud_flat',PointCloud2,received.append)
pub=rospy.Publisher('/livox/lidar',CustomMsg,queue_size=2)
deadline=time.monotonic()+10
while pub.get_num_connections()==0 and time.monotonic()<deadline:
    time.sleep(.05)
assert pub.get_num_connections()>0,'No front-end subscriber'
msg=CustomMsg()
msg.header.stamp=rospy.Time.now()
msg.header.frame_id='livox_frame'
for i in range(120):
    p=CustomPoint()
    p.x=-2.; p.y=float(i%4); p.z=.1
    p.line=i%4; p.tag=0x10; p.reflectivity=42
    p.offset_time=i*100000
    msg.points.append(p)
for value,tag,line in [(float('nan'),0,0),(.1,0,0),(2.,0x20,0),(2.,0,4)]:
    p=CustomPoint(); p.x=value; p.tag=tag; p.line=line
    msg.points.append(p)
msg.point_num=len(msg.points)
time.sleep(.5)
pub.publish(msg)
deadline=time.monotonic()+5
while not received and time.monotonic()<deadline: time.sleep(.05)
assert received,'No cloud returned'
cloud=received[-1]
points=list(point_cloud2.read_points(cloud,field_names=['x','curvature'],skip_nans=False))
assert len(points)==120,len(points)
assert all(p[0]==-2. for p in points),'Rear hemisphere was cropped'
assert abs(points[-1][1]-11.9)<1e-4,points[-1]
assert cloud.header.stamp==msg.header.stamp,'Timestamp changed'
count=len(received)
msg.point_num+=1
pub.publish(msg)
time.sleep(.3)
assert len(received)==count,'Malformed point count accepted'
print('PASS: rear hemisphere, range/tag/line/NaN rejection, ns-to-ms timing, stamp, malformed count')
