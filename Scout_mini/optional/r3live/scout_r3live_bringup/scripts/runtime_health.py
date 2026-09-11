"""Bounded stream checks, using source stamps without rewriting them. No ROS dependency."""
from collections import deque
import math
import statistics
import threading

SENSORS = ('image', 'imu', 'lidar')
OUTPUTS = ('lio', 'camera')
ALL_STREAMS = SENSORS + OUTPUTS


class RuntimeHealth:
    def __init__(self, stale_seconds=2.0, stable_seconds=5.0):
        self.lock = threading.RLock()
        self.stale_seconds = stale_seconds
        self.stable_seconds = stable_seconds
        self.streams = {}
        self.armed = False
        self.fault = None

    def observe(self, name, stamp, ros_now, mono_now, frame, child='', pose=None):
        with self.lock:
            old = self.streams.get(name)
            ages = old['ages'] if old else deque(maxlen=256)
            reason = None
            if not all(math.isfinite(v) for v in (stamp, ros_now, mono_now)) or stamp <= 0:
                reason = 'invalid source/receipt time'
            age = ros_now-stamp
            age_limit = .5 if name in SENSORS else 1.0
            if reason is None and not -.1 <= age <= age_limit:
                reason = 'source clock/latency outside limits (age={:.6f}s)'.format(age)
            if not frame:
                reason = 'empty frame'
            if old:
                if stamp <= old['stamp']:
                    reason = 'source stamp repeated or moved backwards'
                elif (frame, child) != (old['frame'], old['child']):
                    reason = 'frame changed'
                elif mono_now-old['mono'] > self.stale_seconds:
                    reason = 'stream interrupted'
                elif abs((ros_now-old['ros'])-(mono_now-old['mono'])) > .2:
                    reason = 'ROS/system clock jumped relative to monotonic clock'
            if name in OUTPUTS:
                expected_child = 'r3live_imu' if name == 'lio' else 'r3live_camera_optical'
                if (frame, child) != ('r3live_world', expected_child):
                    reason = 'unexpected odometry frames'
                if pose is None or len(pose) != 7 or not all(math.isfinite(v) for v in pose):
                    reason = 'invalid odometry pose'
                elif abs(sum(v*v for v in pose[3:])-1) > 1e-3:
                    reason = 'invalid quaternion'
            if math.isfinite(age):
                ages.append(age)
            good_since = old['good_since'] if old and old['good_since'] is not None else mono_now
            if reason:
                good_since = None
                if self.armed and self.fault is None:
                    self.fault = {'stream': name, 'reason': reason, 'stamp': stamp, 'ros_now': ros_now}
            self.streams[name] = dict(stamp=stamp, ros=ros_now, mono=mono_now,
                                      frame=frame, child=child, ages=ages,
                                      count=(old['count'] if old else 0)+1,
                                      invalid_count=(old['invalid_count'] if old else 0)+int(reason is not None),
                                      reason=reason, good_since=good_since)

    def problems(self, names, mono_now, require_stable=False):
        with self.lock:
            problems = []
            for name in names:
                row = self.streams.get(name)
                if row is None:
                    problems.append(name+': no messages')
                elif mono_now-row['mono'] > self.stale_seconds:
                    problems.append(name+': no fresh messages')
                elif row['reason']:
                    problems.append(name+': '+row['reason'])
                elif require_stable and (row['good_since'] is None or
                        mono_now-row['good_since'] < self.stable_seconds or row['count'] < 3):
                    problems.append(name+': waiting for continuous valid samples')
            return problems

    def arm(self, mono_now):
        with self.lock:
            problems = self.problems(ALL_STREAMS, mono_now, require_stable=True)
            if problems:
                raise ValueError('; '.join(problems))
            self.armed = True

    def check(self, mono_now):
        with self.lock:
            if self.armed and self.fault is None:
                problems = self.problems(ALL_STREAMS, mono_now)
                if problems:
                    self.fault = {'reason': '; '.join(problems)}
            return self.fault

    def snapshot(self, mono_now):
        with self.lock:
            return dict(armed=self.armed, fault=self.fault, streams={name: dict(
                count=row['count'], invalid_count=row['invalid_count'], source_stamp=row['stamp'],
                receipt_age_s=mono_now-row['mono'], frame=row['frame'], child=row['child'],
                source_age_median_s=statistics.median(row['ages']) if row['ages'] else None,
                source_age_min_s=min(row['ages']) if row['ages'] else None,
                source_age_max_s=max(row['ages']) if row['ages'] else None,
                reason=row['reason']) for name, row in self.streams.items()})
