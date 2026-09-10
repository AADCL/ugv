#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as S
import unittest
import threading

spec = importlib.util.spec_from_file_location('fusion_guard', Path(__file__).resolve().parents[1] / 'scripts/fusion_guard.py')
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)
try:
    from nav_msgs.msg import Odometry
except ImportError:
    Odometry = None


class GuardTests(unittest.TestCase):
    def test_stamp_window_and_duplicates(self):
        gate = g.StreamGate(.3, .05)
        self.assertFalse(gate.accept(0, 10, 1))
        self.assertFalse(gate.accept(float('nan'), 10, 1))
        self.assertFalse(gate.accept(9, 10, 1))
        self.assertFalse(gate.accept(10.1, 10, 1))
        self.assertTrue(gate.accept(9.9, 10, 1))
        self.assertFalse(gate.accept(9.9, 10, 1.1))
        self.assertFalse(gate.accept(9.8, 10, 1.1))
        self.assertTrue(gate.accept(10, 10.1, 1.1))

    def test_stopped_clock_and_reset(self):
        gate = g.StreamGate(.3, .05)
        gate.accept(10, 10, 1)
        self.assertTrue(gate.fresh(10.1, 1.1, .5))
        self.assertFalse(gate.fresh(10, 2, .5))
        self.assertFalse(gate.fresh(9, 1.1, .5))

    def test_covariance_order(self):
        result = g.diagonal([1, 2, 3, 4, 5, 6])
        self.assertEqual(len(result), 36)
        self.assertEqual([result[i*7] for i in range(6)], [1,2,3,4,5,6])
        self.assertEqual(sum(result), 21)

    def test_quaternion_sign_and_flu_yaw(self):
        a = S(x=0, y=0, z=0, w=1)
        b = S(x=0, y=0, z=math.sin(math.pi/4), w=math.cos(math.pi/4))
        self.assertTrue(g.valid_quaternion(b))
        self.assertAlmostEqual(g.rotation_distance(a,b), math.pi/2)
        self.assertAlmostEqual(g.rotation_distance(a,S(x=0,y=0,z=0,w=-1)), 0)
        self.assertFalse(g.valid_quaternion(S(x=0,y=0,z=0,w=0)))


@unittest.skipIf(Odometry is None, 'ROS message types require ROS Noetic')
class CallbackTests(unittest.TestCase):
    def setUp(self):
        self.node = n = g.FusionGuard.__new__(g.FusionGuard)
        n.lock = threading.RLock()
        n.world, n.body = 'odom', 'base_link'
        n.age, n.timeout, n.vmax, n.wmax = .3, .5, 2., 3.
        n.jump, n.angle_jump, n.pose_var = .3, .2, [.01]*6
        n.initial_pose_var = [.01]*6
        n.speed_var, n.turn_scale, n.max_scale = .0025, .3, 25.
        n.lio, n.wheel, n.output = [g.StreamGate(.3, .05) for _ in range(3)]
        n.previous_pose, n.first_lio = None, None
        n.started, n.ekf_seeded, n.fault = False, False, ''
        n.Odometry = Odometry
        n.ros = S(logerr=lambda *args: None)
        self.now = 10.
        n.clock = lambda: (self.now, self.now)
        self.lio, self.wheel, self.out = [], [], []
        n.lio_pub, n.wheel_pub, n.pub = [S(publish=v.append) for v in (self.lio,self.wheel,self.out)]

    def message(self, x=12., stamp=None):
        import rospy
        m = Odometry()
        m.header.stamp = rospy.Time.from_sec(self.now if stamp is None else stamp)
        m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
        m.pose.pose.position.x = x
        m.pose.pose.orientation.w = 1.
        return m

    def seed(self):
        m = self.message()
        self.node.on_lio(m)
        self.node.on_output(m)

    def test_pose_seed_and_zero_reset_does_not_enter_filter(self):
        n = self.node
        m = self.message(x=100.)
        m.twist.twist.linear.x = .2
        n.on_wheel(m)
        self.assertEqual(len(self.wheel), 0)
        self.seed()
        n.on_wheel(m)
        self.now += .02
        reset = self.message(x=0.)
        reset.twist.twist.linear.x = .2
        n.on_wheel(reset)
        self.assertEqual([v.twist.twist.linear.x for v in self.wheel], [.2,.2])
        self.assertEqual([v.pose.pose.position.x for v in self.wheel], [0.,0.])
        self.assertEqual(self.lio[0].pose.pose.position.x, 12.)

    def test_backward_speed_and_turn_downweight(self):
        self.seed()
        m = self.message()
        m.twist.twist.linear.x = -.5
        m.twist.twist.angular.z = 1.
        self.node.on_wheel(m)
        self.assertEqual(self.wheel[0].twist.twist.linear.x, -.5)
        self.assertGreater(self.wheel[0].twist.covariance[0], .0025)
        self.assertEqual(self.wheel[0].twist.twist.angular.z, 0.)

    def test_initial_origin_covariance_then_weak_translation(self):
        self.node.pose_var = [100.,100.,100.,.0025,.0025,.0025]
        self.seed()
        self.now += .1
        self.node.on_lio(self.message(x=12.01))
        self.assertEqual(self.lio[0].pose.covariance[0], .01)
        self.assertEqual(self.lio[1].pose.covariance[0], 100.)
        self.assertEqual(self.lio[1].pose.covariance[7], 100.)
        self.assertEqual(self.lio[1].pose.covariance[14], 100.)
        self.assertEqual(self.lio[1].pose.covariance[35], .0025)

    def test_wrong_raw_frame_fails_closed(self):
        m = self.message()
        m.header.frame_id = 'camera_init'
        m.child_frame_id = 'body'
        self.node.on_lio(m)
        self.assertTrue(self.node.fault)
        self.assertEqual(len(self.lio), 0)

    def test_duplicate_pose_and_reset_latches(self):
        self.seed()
        self.node.on_lio(self.message())
        self.assertEqual(len(self.lio), 1)
        self.now += .1
        self.node.on_lio(self.message(x=0.))
        self.assertTrue(self.node.fault)
        self.assertEqual(len(self.lio), 1)

    def test_output_preserves_frame_stamp_and_requires_both_sources(self):
        self.seed()
        self.assertEqual(len(self.out), 0)
        m = self.message()
        self.node.on_wheel(m)
        self.node.on_output(m)
        self.assertEqual(self.out[0].pose.pose.position.x, 12.)
        self.assertEqual(self.out[0].header.frame_id, 'odom')
        self.assertEqual(self.out[0].header.stamp, m.header.stamp)
        self.now += 1.
        self.node.on_output(self.message())
        self.assertEqual(len(self.out), 1)


if __name__ == '__main__':
    unittest.main()
