#!/usr/bin/env python3
"""Exercise failures observed on the real chain, without hardware or ROS."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).parent/'scout_r3live_bringup/scripts'))
from runtime_health import RuntimeHealth, ALL_STREAMS, SENSORS


class StreamHealthTests(unittest.TestCase):
    def feed(self, health, t, names=ALL_STREAMS):
        for name in names:
            frame = 'r3live_world' if name in ('lio', 'camera') else 'sensor'
            child = {'lio': 'r3live_imu', 'camera': 'r3live_camera_optical'}.get(name, '')
            health.observe(name, 100+t-.04, 100+t, t, frame, child, [0, 0, 0, 0, 0, 0, 1])

    def healthy(self):
        health = RuntimeHealth()
        for i in range(61):
            self.feed(health, i/10)
        health.arm(6)
        return health

    def test_warmup_requires_continuous_samples(self):
        h = RuntimeHealth()
        self.feed(h, 0)
        self.assertTrue(h.problems(SENSORS, .1, True))
        self.assertTrue(h.problems(SENSORS, 6, True))  # old data is not warmup

    def test_future_camera_then_recovery_before_arming(self):
        h = RuntimeHealth()
        h.observe('image', 102, 100, 0, 'sensor')
        self.assertTrue(h.problems(('image',), 0))
        for i in range(1, 81):
            self.feed(h, i/10)
        h.arm(8)
        self.assertIsNone(h.check(8))

    def test_future_camera_latches_after_ready(self):
        h = self.healthy()
        h.observe('image', 108, 106.1, 6.1, 'sensor')
        self.feed(h, 9)
        self.assertIsNotNone(h.check(9))

    def test_one_stream_dropout_not_hidden_by_other_updates(self):
        h = self.healthy()
        for i in range(61, 85):
            self.feed(h, i/10, tuple(n for n in ALL_STREAMS if n != 'camera'))
        self.assertIn('camera', h.check(8.4)['reason'])

    def test_duplicate_stamp_and_frame_reset(self):
        for failure in ('duplicate', 'frame'):
            h = self.healthy()
            h.observe('image', 105.96 if failure == 'duplicate' else 106.06,
                      106.1, 6.1, 'other' if failure == 'frame' else 'sensor')
            self.assertIsNotNone(h.check(6.1))

    def test_system_clock_jump(self):
        h = self.healthy()
        h.observe('image', 107.06, 107.1, 6.1, 'sensor')
        self.assertIn('clock jumped', h.check(6.1)['reason'])

    def test_invalid_pose_and_wrong_frame(self):
        for pose, frame in (([float('nan')]*7, 'r3live_world'),
                            ([0]*7, 'r3live_world'), ([0,0,0,0,0,0,1], 'odom')):
            h = self.healthy()
            h.observe('lio', 106.06, 106.1, 6.1, frame, 'r3live_imu', pose)
            self.assertIsNotNone(h.check(6.1))

    def test_memory_bounded(self):
        h = self.healthy()
        for i in range(61, 1000):
            self.feed(h, i/10)
        self.assertEqual(len(h.streams['imu']['ages']), 256)
        self.assertIsNone(h.check(99.9))


if __name__ == '__main__':
    unittest.main()
