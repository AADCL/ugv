#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import math
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('compare',Path(__file__).resolve().parents[1]/'scripts/compare_odometry.py')
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)


def pose(x,y,angle):
    t=np.eye(4)
    t[:3,:3]=c.rotation_matrix(c.Rotation.from_euler('z',angle))
    t[:2,3]=[x,y]
    return t


class CompareTests(unittest.TestCase):
    def test_different_origins_same_forward_motion(self):
        a0=pose(0,0,0);a1=pose(1,0,0)
        b0=pose(10,20,math.pi/2);b1=pose(10,21,math.pi/2)
        np.testing.assert_allclose(c.relative(a0,a1),c.relative(b0,b1),atol=1e-12)

    def test_common_time_interpolation(self):
        samples=[(1.,pose(0,0,0)),(1.1,pose(.2,0,math.pi/2))]
        result=c.interpolate(samples,1.05)
        self.assertAlmostEqual(result[0,3],.1)
        self.assertAlmostEqual(c.yaw(result),math.pi/4)

    def test_wrap_short_arc(self):
        samples=[(1.,pose(0,0,math.radians(179))),(1.1,pose(0,0,math.radians(-179)))]
        self.assertAlmostEqual(abs(c.yaw(c.interpolate(samples,1.05))),math.pi)
        self.assertAlmostEqual(c.wrapped_degrees(math.radians(-358)),2)

    def test_no_extrapolation_or_long_gap(self):
        samples=[(1.,pose(0,0,0)),(2.,pose(1,0,0))]
        for stamp in (.9,1.5,2.1):
            with self.assertRaises(ValueError): c.interpolate(samples,stamp)


if __name__=='__main__':
    unittest.main()
