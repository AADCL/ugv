"""Shared, strict camera/LiDAR calibration contract. Transforms map child to parent."""
from pathlib import Path
import numpy as np
import yaml


def rigid_matrix(value):
    matrix=np.asarray(value,dtype=float)
    if matrix.shape!=(4,4) or not np.isfinite(matrix).all():
        raise ValueError('Expected a finite 4x4 transform')
    if not np.allclose(matrix[3],[0,0,0,1],atol=1e-7):
        raise ValueError('Invalid homogeneous bottom row')
    rotation=matrix[:3,:3]
    if not np.allclose(rotation.T@rotation,np.eye(3),atol=2e-4) or abs(np.linalg.det(rotation)-1)>2e-4:
        raise ValueError('Transform rotation is not in SO(3)')
    return matrix


def camera_signature(info):
    return {'width':int(info.width),'height':int(info.height),
            'K':list(info.K),'D':list(info.D) or [0.]*5,
            'distortion_model':info.distortion_model,'optical_frame':info.header.frame_id}


def same_camera(a,b):
    if any(a[k]!=b[k] for k in ('width','height','distortion_model','optical_frame')):
        raise ValueError('Camera resolution/model/frame changed since calibration')
    for key in ('K','D'):
        if len(a[key])!=len(b[key]) or not np.allclose(a[key],b[key],rtol=0,atol=1e-5):
            raise ValueError('Camera intrinsics changed since calibration: '+key)


def load_accepted(path,info,lidar_to_imu):
    data=yaml.safe_load(Path(path).expanduser().read_text())
    if data.get('schema')!=1 or data.get('status')!='operator_accepted':
        raise ValueError('Calibration must be explicitly accepted after independent projection review')
    if data.get('convention')!='p_camera_optical = T_camera_lidar * p_lidar':
        raise ValueError('Unexpected extrinsic convention')
    same_camera(data['camera'],camera_signature(info))
    if not np.allclose(data['lidar_to_imu_translation'],lidar_to_imu,rtol=0,atol=1e-8):
        raise ValueError('LiDAR/IMU internal geometry differs from the calibration export')
    camera_lidar=rigid_matrix(data['T_camera_lidar'])
    imu_lidar=np.eye(4); imu_lidar[:3,3]=lidar_to_imu
    imu_camera=imu_lidar@np.linalg.inv(camera_lidar)
    rigid_matrix(imu_camera)
    return imu_camera,data
