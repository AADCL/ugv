#!/usr/bin/env python3
import unittest
import tempfile
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from prior_refine import pose,perpendicular,huber_residual,fit,project_points,center_hypothesis,evaluate


class PriorTest(unittest.TestCase):
    def test_lost_projection_is_penalized(self):
        camera=dict(K=[100.,0,50.,0,100.,50.,0,0,1.],D=[0.]*5,width=100,height=100)
        pixels=np.array([[50.,50.]])
        g=dict(name='fixed',points=np.array([[0.,0.,2.]]),pixels=pixels,tree=cKDTree(pixels),
            camera=camera,image=np.zeros((100,100,3),np.uint8),extraction_sha256='test')
        with tempfile.TemporaryDirectory() as d:
            a=evaluate(g,np.eye(4),Path(d)/'baseline')
            b=evaluate(g,pose(np.eye(4),np.array([0,0,0,5,0,0])),Path(d)/'lost')
        self.assertEqual(a['fixed_geometric_samples'],b['fixed_geometric_samples'])
        self.assertEqual(a['median_px'],0.)
        self.assertEqual(b['median_px'],50.)
        self.assertEqual(b['visible_samples'],0)

    def test_oblique_line(self):
        d=np.array([[1.,1.]])/np.sqrt(2)
        self.assertAlmostEqual(float(perpendicular(3*d,d)[0]),0.)
        self.assertAlmostEqual(float(perpendicular(np.array([[-1.,1.]])/np.sqrt(2),d)[0]),1.)

    def test_pose_direction_and_units(self):
        initial=np.eye(4); initial[:3,3]=[.1,.2,.3]
        result=pose(initial,np.array([0,0,np.pi/2,.01,0,0]))
        np.testing.assert_allclose(result[:3,:3]@[1,0,0],[0,1,0],atol=1e-12)
        np.testing.assert_allclose(result[:3,3],[.11,.2,.3])
        arm=np.array([.01,-.02,.04])
        stored=result.copy(); stored[:3,3]+=stored[:3,:3]@arm
        np.testing.assert_allclose(center_hypothesis(stored,arm),result,atol=1e-12)
        np.testing.assert_allclose(huber_residual(np.array([-10.,0.,10.]))**2,[36,0,36])

    def test_recover_nonzero_offset_with_bad_image_edges(self):
        camera=dict(K=[400.,0,320.,0,400.,240.,0,0,1.],D=[0.]*5,width=640,height=480)
        truth=np.eye(4); initial=pose(truth,np.array([.006,-.009,.004,.009,-.006,.004]))
        geometries=[]
        for z in (2.,3.,4.):
            points=[]; directions=[]
            for x in (-.6,0,.6):
                points.extend([[x,y,z] for y in np.linspace(-.5,.5,70)])
                directions.extend([[0.,1.]]*70)
            for y in (-.5,0,.5):
                points.extend([[x,y,z] for x in np.linspace(-.6,.6,70)])
                directions.extend([[1.,0.]]*70)
            points=np.asarray(points); pixels=project_points(points,truth,camera)[0]
            # A limited false-edge subset checks robustness to inconsistent evidence.
            pixels[::17]+=[8.,-5.]
            geometries.append(dict(points=points,pixels=pixels,tree=cKDTree(pixels),
                directions=np.asarray(directions),reliable=np.ones(len(pixels),bool),camera=camera))
        result=fit(geometries,initial,1.,.01)
        self.assertEqual(result['status'],'candidate_not_accepted')
        estimate=np.asarray(result['T_camera_lidar'])
        before=sum(np.mean(np.linalg.norm(project_points(g['points'],initial,camera)[0]-project_points(g['points'],truth,camera)[0],axis=1)) for g in geometries)
        after=sum(np.mean(np.linalg.norm(project_points(g['points'],estimate,camera)[0]-project_points(g['points'],truth,camera)[0],axis=1)) for g in geometries)
        self.assertLess(after,before*.35)
        rejected=fit([dict(geometries[0],reliable=np.zeros(len(geometries[0]['pixels']),bool))],initial,1.,.01)
        self.assertEqual(rejected['status'],'rejected')


if __name__=='__main__': unittest.main()
