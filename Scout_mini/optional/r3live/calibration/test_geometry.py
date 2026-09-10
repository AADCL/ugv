#!/usr/bin/env python3
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
import yaml
from calibration_io import rigid_matrix,load_accepted,camera_signature
from calibrate import write_pcd,read_pcd,safe_name


class GeometryTests(unittest.TestCase):
    def test_inverse_and_lever_arm(self):
        theta=.6
        t=np.eye(4); t[:3,:3]=[[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]]
        t[:3,3]=[.2,-.1,.05]
        info=SimpleNamespace(width=640,height=480,K=[600.,0,320,0,600,240,0,0,1],D=[0.]*5,
            distortion_model='plumb_bob',header=SimpleNamespace(frame_id='color_optical'))
        data={'schema':1,'status':'operator_accepted','convention':'p_camera_optical = T_camera_lidar * p_lidar',
            'camera':camera_signature(info),'T_camera_lidar':t.tolist(),'lidar_to_imu_translation':[-.011,-.02329,.04412]}
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'calibration.yaml'; path.write_text(yaml.safe_dump(data))
            result,_=load_accepted(path,info,data['lidar_to_imu_translation'])
            p_l=np.array([1.,2.,3.,1.]); p_c=t@p_l
            expected=p_l.copy(); expected[:3]+=data['lidar_to_imu_translation']
            np.testing.assert_allclose(result@p_c,expected,atol=1e-12)
            info.width=1280
            with self.assertRaises(ValueError): load_accepted(path,info,data['lidar_to_imu_translation'])
            info.width=640; data['status']='candidate'; path.write_text(yaml.safe_dump(data))
            with self.assertRaises(ValueError): load_accepted(path,info,data['lidar_to_imu_translation'])

    def test_invalid_transform(self):
        for t in (np.zeros((4,4)),np.full((4,4),np.nan),np.diag([-1.,1.,1.,1.])):
            with self.assertRaises(ValueError): rigid_matrix(t)

    def test_binary_pcd(self):
        points=np.array([[1,2,3,42],[-1,.1,.2,7]],dtype=np.float32)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'cloud.pcd'; write_pcd(path,points)
            np.testing.assert_array_equal(read_pcd(path),points)

    def test_names(self):
        for name in ('../escape','/tmp/a','..','a/b',''):
            with self.assertRaises(ValueError): safe_name(name)


if __name__=='__main__': unittest.main()
