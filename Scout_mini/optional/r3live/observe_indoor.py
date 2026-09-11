#!/usr/bin/env python3
"""Bounded stationary diagnostics; never commands the vehicle or claims accuracy."""
import argparse
import json
from pathlib import Path
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--seconds', type=float, default=30)
p.add_argument('--output', required=True)
a = p.parse_args()
if not 10 <= a.seconds <= 120:
    p.error('--seconds must be between 10 and 120')
rospy.init_node('r3live_indoor_observer', anonymous=True)
lock = threading.Lock()
samples = {name: [] for name in ('lio', 'camera', 'image', 'imu', 'tracking')}

def receive(name, msg):
    row = [time.monotonic(), msg.header.stamp.to_sec(), rospy.Time.now().to_sec()]
    if name in ('lio', 'camera'):
        pose = msg.pose.pose
        row += [pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    with lock:
        samples[name].append(row)

for name, topic, kind in (
    ('lio', '/r3live/odometry', Odometry),
    ('camera', '/r3live/camera_odometry', Odometry),
    ('image', '/r3live_camera/color/image_raw', Image),
    ('imu', '/livox/imu', Imu),
    ('tracking', '/r3live/track_image', Image),
):
    rospy.Subscriber(topic, kind, lambda m, n=name: receive(n, m), queue_size=100)
deadline = time.monotonic() + a.seconds
while time.monotonic() < deadline and not rospy.is_shutdown():
    time.sleep(.1)
with lock:
    snapshot = {k: list(v) for k, v in samples.items()}
report = {'scope': 'Stationary output stability, not ground-truth accuracy; operator must keep vehicle still',
          'observation_seconds': a.seconds, 'topics': {}}
healthy = True
for name, rows in snapshot.items():
    stat = {'count': len(rows)}
    if len(rows) > 1:
        data = np.asarray(rows)
        stamp_diff = np.diff(data[:, 1])
        stat.update(rate_hz=float((len(rows)-1)/(data[-1, 0]-data[0, 0])),
                    non_increasing_stamps=int(np.count_nonzero(stamp_diff <= 0)),
                    max_arrival_gap_s=float(np.diff(data[:, 0]).max()),
                    median_age_s=float(np.median(data[:, 2]-data[:, 1])),
                    first_source_stamp=float(data[0, 1]),
                    last_source_stamp=float(data[-1, 1]),
                    last_arrival_age_s=float(time.monotonic()-data[-1, 0]))
        healthy &= bool(np.isfinite(data).all() and (stamp_diff > 0).all()
                        and stat['last_arrival_age_s'] < 2 and stat['max_arrival_gap_s'] < 2)
        if name in ('lio', 'camera'):
            displacement = np.linalg.norm(data[:, 3:6]-data[0, 3:6], axis=1)
            quat = data[:, 6:10]
            norms = np.linalg.norm(quat, axis=1)
            if (norms > 0).all():
                quat = quat/norms[:, None]
                angle = 2*np.arccos(np.clip(np.abs(quat @ quat[0]), 0, 1))
                stat['max_rotation_from_first_deg'] = float(np.degrees(angle).max())
            stat.update(max_displacement_from_first_m=float(displacement.max()),
                        final_displacement_m=float(displacement[-1]),
                        max_step_m=float(np.linalg.norm(np.diff(data[:, 3:6], axis=0), axis=1).max()))
            healthy &= bool(np.all(np.abs(norms-1) < 1e-3))
    else:
        healthy = False
    report['topics'][name] = stat
report['continuous_finite_outputs'] = bool(healthy)
report['sensor_clock_sanity'] = bool(all(
    'median_age_s' in report['topics'][name]
    and -.1 < report['topics'][name]['median_age_s'] < .5
    for name in ('image', 'imu')))
with Path(a.output).open('x') as handle:
    json.dump(report, handle, indent=2, allow_nan=False)
print(json.dumps(report, indent=2, allow_nan=False))
raise SystemExit(0 if healthy and report['sensor_clock_sanity'] else 1)
