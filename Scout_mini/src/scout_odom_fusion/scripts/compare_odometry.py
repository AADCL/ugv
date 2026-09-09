#!/usr/bin/env python3
"""Read-only, timestamp-aligned comparison. Wheel odometry is a reference, not truth."""
import bisect
import csv
import json
import math
from pathlib import Path
import tempfile
import threading
import time
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

NAMES = ('wheel', 'lio', 'fused')


def rotation_from_matrix(matrix):
    # Ubuntu 20.04 ships SciPy 1.3, whose API used the older DCM names.
    return Rotation.from_matrix(matrix) if hasattr(Rotation, 'from_matrix') else Rotation.from_dcm(matrix)


def rotation_matrix(rotation):
    return rotation.as_matrix() if hasattr(rotation, 'as_matrix') else rotation.as_dcm()


def interpolate(samples, stamp, max_gap=.20):
    items = list(samples)
    times = [s[0] for s in items]
    k = bisect.bisect_left(times, stamp)
    if k < len(items) and abs(times[k]-stamp) < 1e-8:
        return items[k][1].copy()
    if k == 0 or k == len(items) or times[k]-times[k-1] > max_gap:
        raise ValueError('No valid interpolation bracket')
    a, b = items[k-1], items[k]
    alpha = (stamp-a[0])/(b[0]-a[0])
    out = np.eye(4)
    out[:3,3] = (1-alpha)*a[1][:3,3] + alpha*b[1][:3,3]
    out[:3,:3] = rotation_matrix(Slerp([0.,1.], rotation_from_matrix(np.stack([a[1][:3,:3], b[1][:3,:3]])))([alpha]))[0]
    return out


def relative(origin, pose):
    result = np.eye(4)
    result[:3,:3] = origin[:3,:3].T.dot(pose[:3,:3])
    result[:3,3] = origin[:3,:3].T.dot(pose[:3,3]-origin[:3,3])
    return result


def yaw(pose):
    return math.atan2(pose[1,0], pose[0,0])


def wrapped_degrees(value):
    return math.degrees(math.atan2(math.sin(value), math.cos(value)))


class Comparison:
    def __init__(self):
        import rospy
        from nav_msgs.msg import Odometry
        self.ros = rospy
        self.lock = threading.RLock()
        root = Path(rospy.get_param('~output_root', str(Path.home()/'livox_fastlio/logs/fusion_tests'))).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        self.directory = Path(tempfile.mkdtemp(prefix=time.strftime('%Y%m%d_%H%M%S_'), dir=str(root)))
        self.buffers = {n: deque(maxlen=300) for n in NAMES}
        self.arrived, self.frames = {}, {}
        self.origin = None
        self.started = self.last = None
        self.fault = ''
        self.last_clock = None
        self.last_report = 0.
        self.rejected = {n: 0 for n in NAMES}
        self.count = 0
        self.stats = {n: {'sum_sq_xy': 0., 'max_xy_m': 0., 'max_abs_yaw_deg': 0.} for n in ('lio','fused')}
        self.path_length = {n: 0. for n in NAMES}
        self.previous = None
        self.plot_rows = deque(maxlen=20000)
        self.last_row = None
        self.file = open(str(self.directory/'samples.csv'), 'w', newline='')
        fields = ['stamp', 'elapsed_s']
        for n in NAMES:
            fields += [n+'_'+x for x in ('x_m','y_m','z_m','yaw_deg','net_xy_m','path_xy_m')]
        for n in ('lio','fused'):
            fields += [n+'_wheel_xy_difference_m', n+'_wheel_yaw_difference_deg']
        self.writer = csv.DictWriter(self.file, fieldnames=fields)
        self.writer.writeheader()
        self.subs = []
        for name, default in zip(NAMES, ('/scout/odom','/fastlio_odom','/scout/fused_odom')):
            topic = rospy.get_param('~'+name+'_topic', default)
            self.subs.append(rospy.Subscriber(topic, Odometry, self.receive, callback_args=name, queue_size=100))
        print('[COMPARE] Output: %s' % self.directory, flush=True)
        print('[COMPARE] WAIT: keep stationary until READY. Wheel differences are NOT ground-truth errors.', flush=True)

    def receive(self, msg, name):
        with self.lock:
            if self.fault:
                return
            stamp = msg.header.stamp.to_sec()
            now = self.ros.Time.now().to_sec()
            buf = self.buffers[name]
            if (stamp <= 0 or not math.isfinite(stamp) or not -.05 <= now-stamp <= .5
                    or (buf and stamp <= buf[-1][0])):
                self.rejected[name] += 1
                return
            frame = (msg.header.frame_id, msg.child_frame_id)
            worlds = ('odom','scout_odom') if name == 'wheel' else ('odom',)
            if (frame[0] not in worlds or frame[1] != 'base_link'
                    or (name in self.frames and frame != self.frames[name])):
                self.fault = name + ': wrong/changed frame; expected FLU base_link'
                return
            self.frames[name] = frame
            p,q = msg.pose.pose.position,msg.pose.pose.orientation
            xyz = np.array([p.x,p.y,p.z])
            quat = np.array([q.x,q.y,q.z,q.w])
            if not np.isfinite(xyz).all() or not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1)>0.01:
                self.fault = name + ': invalid pose'
                return
            mat = np.eye(4)
            mat[:3,3] = xyz
            mat[:3,:3] = rotation_matrix(Rotation.from_quat(quat))
            if buf and self.origin is not None:
                dt = stamp-buf[-1][0]
                delta = relative(buf[-1][1],mat)
                if (dt > .5 or np.linalg.norm(delta[:3,3]) > .30+2*dt
                        or np.linalg.norm(rotation_from_matrix(delta[:3,:3]).as_rotvec()) > .20+3*dt):
                    self.fault = name + ': gap/reset/jump; comparison stopped, restart to rebaseline'
                    return
            buf.append((stamp,mat))
            self.arrived[name] = time.monotonic()

    def sample(self):
        now, wall = self.ros.Time.now().to_sec(), time.monotonic()
        if self.last_clock is not None and now < self.last_clock-.001:
            self.fault = 'ROS clock moved backwards'
        self.last_clock = now
        if self.fault:
            return
        fresh = all(len(self.buffers[n]) >= 2 and wall-self.arrived.get(n,0) <= .5
                    and -.05 <= now-self.buffers[n][-1][0] <= .5 for n in NAMES)
        if not fresh:
            if self.origin is not None:
                self.fault = 'Input missing/stale; previous comparison retained, no extrapolation'
            return
        stamp = min(self.buffers[n][-1][0] for n in NAMES)-.02
        if self.last is not None and stamp-self.last < .09:
            return
        try:
            poses = {n: interpolate(self.buffers[n],stamp) for n in NAMES}
        except ValueError:
            if self.last is not None and stamp-self.last > .5:
                self.fault = 'Cannot synchronize three streams within interpolation limits'
            return
        if self.origin is None:
            self.origin, self.started = poses, stamp
            print('[COMPARE] READY: common timestamp %.6f; origins aligned. You may start the manual test.' % stamp, flush=True)
        transforms = {n: relative(self.origin[n],poses[n]) for n in NAMES}
        row = {'stamp': stamp, 'elapsed_s': stamp-self.started}
        for n in NAMES:
            t = transforms[n]
            if self.previous is not None:
                self.path_length[n] += float(np.linalg.norm(t[:2,3]-self.previous[n][:2,3]))
            for key,value in zip(('x_m','y_m','z_m'),t[:3,3]):
                row[n+'_'+key] = float(value)
            row[n+'_yaw_deg'] = math.degrees(yaw(t))
            row[n+'_net_xy_m'] = float(np.linalg.norm(t[:2,3]))
            row[n+'_path_xy_m'] = self.path_length[n]
        for n in ('lio','fused'):
            distance = float(np.linalg.norm(transforms[n][:2,3]-transforms['wheel'][:2,3]))
            angle = wrapped_degrees(yaw(transforms[n])-yaw(transforms['wheel']))
            row[n+'_wheel_xy_difference_m'],row[n+'_wheel_yaw_difference_deg'] = distance,angle
            st = self.stats[n]
            st['sum_sq_xy'] += distance**2
            st['max_xy_m'] = max(st['max_xy_m'],distance)
            st['max_abs_yaw_deg'] = max(st['max_abs_yaw_deg'],abs(angle))
        self.count += 1
        self.writer.writerow(row)
        self.plot_rows.append(row)
        self.previous,self.last,self.last_row = transforms,stamp,row

    def report(self):
        if self.fault:
            print('[COMPARE] INVALID: '+self.fault+'; no fresh comparison, no vehicle stop command sent.', flush=True)
        elif self.last_row:
            r = self.last_row
            print('\n[COMPARE] t=%.1fs | relative to common start; NOT absolute accuracy' % r['elapsed_s'], flush=True)
            print('source   dx(m)    dy(m)   yaw(deg)  net(m)  path(m)  dXY/wheel(m) dYaw/wheel(deg)', flush=True)
            for n in NAMES:
                print('%-6s %8.3f %8.3f %9.2f %7.3f %8.3f %12.3f %15.2f' % (
                    n,r[n+'_x_m'],r[n+'_y_m'],r[n+'_yaw_deg'],r[n+'_net_xy_m'],r[n+'_path_xy_m'],
                    r.get(n+'_wheel_xy_difference_m',0),r.get(n+'_wheel_yaw_difference_deg',0)), flush=True)
        else:
            print('[COMPARE] WAIT: samples '+str({n:len(v) for n,v in self.buffers.items()}), flush=True)
        self.file.flush()

    def finish(self):
        self.file.flush()
        self.file.close()
        summary = {'valid_samples':self.count,'invalid_reason':self.fault,'frames':self.frames,
                   'last':self.last_row,'rejected_samples':self.rejected,
                   'meaning':'Differences to wheel reference, NOT independent accuracy. Path length includes stationary noise.',
                   'plot_last_samples_limit':20000,'difference_statistics':{}}
        for n,st in self.stats.items():
            summary['difference_statistics'][n] = {'rms_xy_m':math.sqrt(st['sum_sq_xy']/self.count) if self.count else None,
                                                   'max_xy_m':st['max_xy_m'],'max_abs_yaw_deg':st['max_abs_yaw_deg']}
        (self.directory/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
        print('[COMPARE] SUMMARY '+json.dumps(summary['difference_statistics']), flush=True)
        if self.plot_rows:
            try:
                import matplotlib
                matplotlib.use('Agg')
                import matplotlib.pyplot as plt
                rows = list(self.plot_rows)
                fig,axes = plt.subplots(1,3,figsize=(15,4))
                for n in NAMES:
                    axes[0].plot([r[n+'_x_m'] for r in rows],[r[n+'_y_m'] for r in rows],label=n)
                    axes[1].plot([r['elapsed_s'] for r in rows],[r[n+'_yaw_deg'] for r in rows],label=n)
                for n in ('lio','fused'):
                    axes[2].plot([r['elapsed_s'] for r in rows],[r[n+'_wheel_xy_difference_m'] for r in rows],label=n)
                for ax,title in zip(axes,('Aligned XY (m)','Relative yaw (deg), wrapped','Difference to wheel (m), NOT truth')):
                    ax.set_title(title);ax.legend();ax.grid(True)
                axes[0].axis('equal')
                axes[1].set_xlabel('Time (s)');axes[2].set_xlabel('Time (s)')
                fig.tight_layout();fig.savefig(str(self.directory/'comparison.png'),dpi=150);plt.close(fig)
            except Exception as error:
                print('[COMPARE] Plot unavailable: '+str(error),flush=True)
        print('[COMPARE] SAVED '+str(self.directory),flush=True)

    def run(self):
        try:
            while not self.ros.is_shutdown():
                with self.lock:
                    self.sample()
                    if time.monotonic()-self.last_report>=1:
                        self.report();self.last_report=time.monotonic()
                time.sleep(.03)
        finally:
            for sub in self.subs:
                sub.unregister()
            with self.lock:
                self.finish()


if __name__ == '__main__':
    import rospy
    rospy.init_node('scout_odom_comparison')
    Comparison().run()
