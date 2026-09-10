#!/usr/bin/env python3
"""Inject faults into a short recorded scene, never publish them to ROS."""
import argparse
from pathlib import Path
import tempfile
import rosbag
from calibrate import export_scene,LIDAR,IMU,INFO,TOPICS

parser=argparse.ArgumentParser(); parser.add_argument('bag',type=Path); args=parser.parse_args()
for fault,expected in [('motion','Rotation during'),('missing_info','Incomplete scene'),('bad_count','Malformed point_num')]:
    with tempfile.TemporaryDirectory(prefix='scout_calib_reject_') as temporary:
        root=Path(temporary); output=root/'export'; output.mkdir()
        with rosbag.Bag(str(args.bag)) as source,rosbag.Bag(str(root/'fault.bag'),'w') as target:
            for topic,msg,t in source.read_messages(topics=TOPICS):
                if fault=='motion' and topic==IMU: msg.angular_velocity.x=1.
                if fault=='missing_info' and topic==INFO: continue
                if fault=='bad_count' and topic==LIDAR: msg.point_num+=1
                target.write(topic,msg,t)
        try: export_scene([root/'fault.bag'],output,True)
        except ValueError as error:
            assert expected in str(error),(fault,str(error))
        else: raise AssertionError('Fault not rejected: '+fault)
        assert not (output/'scene.yaml').exists()
        print('PASS '+fault)
