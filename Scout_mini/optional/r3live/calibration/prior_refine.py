#!/usr/bin/env python3
"""Offline sensitivity study around measured geometry; never accepts a result."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from calibrate import R3, prepare, solve, scene, read, dump, digest, safe_name
from calibration_io import rigid_matrix,same_camera


def rotation_matrix(r):
    return r.as_matrix() if hasattr(r,'as_matrix') else r.as_dcm()


def pose(initial,delta):
    result=initial.copy()
    result[:3,:3]=rotation_matrix(Rotation.from_rotvec(delta[:3]))@initial[:3,:3]
    result[:3,3]=initial[:3,3]+delta[3:]
    return rigid_matrix(result)


def center_hypothesis(stored,imu_lidar_translation):
    """Treat measured mounting center as LiDAR origin, not IMU origin.

    The actual casing-to-LiDAR offset is unknown; this is an audited hypothesis.
    stored = T_camera_imu * T_imu_lidar; remove the previously added lever arm.
    """
    result=stored.copy()
    result[:3,3]-=stored[:3,:3]@np.asarray(imu_lidar_translation)
    return rigid_matrix(result)


def project_points(points,transform,camera):
    xyz=points@transform[:3,:3].T+transform[:3,3]
    uv,_=cv2.projectPoints(xyz,np.zeros(3),np.zeros(3),
        np.asarray(camera['K']).reshape(3,3),np.asarray(camera['D']))
    return uv.reshape(-1,2),xyz[:,2]


def perpendicular(error,direction):
    """Signed point-to-line residual: invariant to tangent displacement."""
    return -direction[:,1]*error[:,0]+direction[:,0]*error[:,1]


def huber_residual(value,threshold=2.):
    # Apply Huber only to data. The Gaussian measurement prior stays quadratic.
    a=np.abs(value)
    return np.sign(value)*np.sqrt(np.where(a<=threshold,a*a,2*threshold*a-threshold**2))


def load_geometry(run,index,name,source,initial):
    path,data=source
    edges=np.atleast_2d(np.loadtxt(run/'image'/('%d_edges.csv'%index),delimiter=','))
    pixels=np.atleast_2d(np.loadtxt(run/'image'/('%d_image_edges.csv'%index),delimiter=','))
    if edges.shape[1]!=3 or pixels.shape[1]!=2 or not np.isfinite(edges).all() or not np.isfinite(pixels).all():
        raise ValueError('Invalid extracted geometry: '+name)
    # Equal spatial sampling, not one independent observation per accumulated hit.
    _,indices=np.unique(np.floor(edges/.02).astype(np.int64),axis=0,return_index=True)
    edges=edges[np.sort(indices)]
    edges=edges[::max(1,int(np.ceil(len(edges)/2000)))]
    camera=data['camera']; uv,z=project_points(edges,initial,camera)
    fixed=(z>.3)&(uv[:,0]>=0)&(uv[:,0]<camera['width'])&(uv[:,1]>=0)&(uv[:,1]<camera['height'])
    edges=edges[fixed]
    if len(edges)<30: raise ValueError('Fewer than 30 independent geometric samples in measured FOV: '+name)
    tree=cKDTree(pixels)
    _,neighbors=tree.query(pixels,k=5)
    local=pixels[neighbors]; local-=local.mean(axis=1,keepdims=True)
    cov=np.einsum('nki,nkj->nij',local,local)/5
    values,vectors=np.linalg.eigh(cov)
    directions=vectors[:,:,-1]
    reliable=values[:,1]>4*np.maximum(values[:,0],.01)
    image=cv2.imread(str(path/'image.bmp'))
    return dict(name=name,points=edges,pixels=pixels,tree=tree,directions=directions,
        reliable=reliable,camera=camera,image=image,source=data,
        extraction_sha256=digest(run/'image'/('%d_edges.csv'%index)))


def associations(geometry,transform,gate=20.):
    uv,z=project_points(geometry['points'],transform,geometry['camera'])
    distance,idx=geometry['tree'].query(uv)
    c=geometry['camera']
    valid=(z>.3)&(uv[:,0]>=0)&(uv[:,0]<c['width'])&(uv[:,1]>=0)&(uv[:,1]<c['height'])
    valid&=(distance<gate)&geometry['reliable'][idx]
    return geometry['points'][valid],geometry['pixels'][idx[valid]],geometry['directions'][idx[valid]]


def fit(geometries,initial,rotation_sigma_deg,translation_sigma_m):
    sigma=np.array([np.deg2rad(rotation_sigma_deg)]*3+[translation_sigma_m]*3)
    if not np.isfinite(sigma).all() or np.any(sigma<=0): raise ValueError('Positive finite prior scales required')
    counts=[len(associations(g,initial)[0]) for g in geometries]
    if min(counts)<30:
        return dict(status='rejected',reason='fewer than 30 matches in a training scene',counts=counts)
    trees=[cKDTree(g['pixels'][g['reliable']]) for g in geometries]
    # Fixed-support robust Chamfer objective. Never remove points as matching
    # changes: a shrinking association set is not evidence of improvement.
    # Distance to a fixed set is continuous across nearest-neighbor switches;
    # independently re-solving signed tangent matches need not be monotonic.
    def residual(normalized):
        transform=pose(initial,normalized*sigma); terms=[]
        for geometry,tree in zip(geometries,trees):
            uv,z=project_points(geometry['points'],transform,geometry['camera'])
            # Grossly incompatible edges have constant capped cost, not a force
            # pulling the rig toward unrelated image structures. They stay in
            # the denominator and retain the cap when a candidate loses them.
            distances=np.minimum(tree.query(uv)[0],20.)
            terms.extend(huber_residual(distances/2.)*np.sqrt(200./len(geometries)/len(uv)))
            terms.extend(np.maximum(.1-z,0)*100)
        return np.r_[terms,normalized]
    solution=least_squares(residual,np.zeros(6),bounds=(-3.,3.),
        max_nfev=500,ftol=1e-6,xtol=1e-6,gtol=1e-6)
    delta=solution.x*sigma
    final_counts=[len(associations(g,pose(initial,delta))[0]) for g in geometries]
    radial_rotation=float(np.linalg.norm(delta[:3])/sigma[0])
    radial_translation=float(np.linalg.norm(delta[3:])/sigma[3])
    status='candidate_not_accepted'
    if not solution.success or min(final_counts)<30 or max(radial_rotation,radial_translation)>=2.99:
        status='rejected'
    return dict(status=status,T_camera_lidar=pose(initial,delta).tolist(),
        rotation_change_deg=float(np.rad2deg(np.linalg.norm(delta[:3]))),
        translation_change_m=float(np.linalg.norm(delta[3:])),
        rotation_sigma_deg=rotation_sigma_deg,translation_sigma_m=translation_sigma_m,
        prior_assumption='Sensitivity setting, not measured uncertainty or accuracy',
        radial_sigma=[radial_rotation,radial_translation],
        solver=dict(success=bool(solution.success),message=solution.message,nfev=int(solution.nfev),
            initial_cost=float(np.sum(residual(np.zeros(6))**2)/2),final_cost=float(solution.cost),
            initial_matches=counts,final_matches=final_counts),
        reason='Non-convergence, insufficient final matches or 3-sigma boundary' if status=='rejected' else 'Requires independent review')


def evaluate(geometry,transform,directory):
    directory.mkdir(parents=True,exist_ok=False)
    uv,z=project_points(geometry['points'],transform,geometry['camera'])
    c=geometry['camera']; n=len(uv)
    valid=(z>.3)&np.isfinite(uv).all(axis=1)&(uv[:,0]>=0)&(uv[:,0]<c['width'])&(uv[:,1]>=0)&(uv[:,1]<c['height'])
    # Fixed measured-FOV sample set for every candidate. Lost coverage is penalized,
    # never dropped to make a candidate's median artificially improve.
    distances=np.full(n,50.)
    distances[valid]=np.minimum(geometry['tree'].query(uv[valid])[0],50.)
    image=geometry['image'].copy()
    p=geometry['pixels'].astype(int)
    image[p[:,1],p[:,0]]=[0,255,0]
    for u,v in uv[valid].astype(int): cv2.circle(image,(int(u),int(v)),1,(0,0,255),-1)
    if not cv2.imwrite(str(directory/'geometric_edges.png'),image): raise IOError('Overlay write failed')
    report=dict(scene=geometry['name'],fixed_geometric_samples=n,visible_samples=int(valid.sum()),
        median_px=float(np.median(distances)),p90_px=float(np.percentile(distances,90)),
        fraction_within_3px=float(np.mean(distances<=3)),fraction_within_5px=float(np.mean(distances<=5)),
        extracted_geometry_sha256=geometry['extraction_sha256'],
        note='Actual extracted plane intersections vs extracted image edges. Fixed baseline-FOV samples; missing projection costs 50px. Occlusion and wrong geometric edges can bias this diagnostic; not physical accuracy.')
    dump(directory/'metrics.yaml',report)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name'); parser.add_argument('--root',type=Path,default=R3/'calibration')
    parser.add_argument('--training',nargs='+',required=True)
    parser.add_argument('--validation',nargs='+',required=True)
    parser.add_argument('--port',type=int,default=11443)
    parser.add_argument('--mount-reference',choices=['stored-imu','lidar-center-hypothesis'],default='stored-imu')
    parser.add_argument('--compare-runs',nargs='*',default=[])
    args=parser.parse_args(); safe_name(args.name)
    names=args.training+args.validation
    if len(args.training)<3 or len(set(names))!=len(names):
        raise ValueError('At least 3 training scenes, distinct from held-out validation')
    sources=[scene(args,name) for name in names]
    # prepare verifies camera/frame/translation and distinct cloud hashes across all scenes.
    prepare(SimpleNamespace(root=args.root,name=args.name,scenes=names))
    run=args.root/'runs'/args.name
    marker=run/'.study_incomplete'; marker.touch()
    manifest=read(run/'manifest.yaml')
    manifest.update(training_scenes=args.training,validation_scenes=args.validation,
        purpose='Independent prior sensitivity study; extraction includes validation, fitting never does',
        scene_hashes=[data['cloud_sha256'] for _,data in sources[:len(args.training)]])
    dump(run/'manifest.yaml',manifest)
    solve(SimpleNamespace(root=args.root,name=args.name,port=args.port,timeout=1800,extract_only=True))
    stored_initial=rigid_matrix(manifest['initial_T_camera_lidar'])
    for _,data in sources:
        if not np.allclose(stored_initial,data['initial_T_camera_lidar'],atol=1e-8):
            raise ValueError('Measured initial geometry changed across scenes')
    initial=stored_initial.copy()
    if args.mount_reference=='lidar-center-hypothesis':
        initial=center_hypothesis(stored_initial,manifest['lidar_to_imu_translation'])
    manifest.update(mount_reference=args.mount_reference,study_prior_T_camera_lidar=initial.tolist(),
        origin_caveat='Casing/geometric center was measured; physical offset to manufacturer LiDAR origin is unknown. Never assume it was IMU origin.')
    dump(run/'manifest.yaml',manifest)
    geometries=[load_geometry(run,i,n,s,initial) for i,(n,s) in enumerate(zip(names,sources))]
    training=geometries[:len(args.training)]
    results={}
    candidates={'measured':dict(status='measured_baseline',T_camera_lidar=initial.tolist())}
    if args.mount_reference!='stored-imu':
        candidates['legacy_imu_interpretation']=dict(status='legacy_baseline',T_camera_lidar=stored_initial.tolist())
    for old_name in args.compare_runs:
        old=args.root/'runs'/safe_name(old_name)
        record=read(old/'result.yaml'); old_manifest=read(old/'manifest.yaml')
        if (old/'.solving').exists() or digest(old/'extrinsic.txt')!=record['sha256']:
            raise ValueError('Incomplete or modified comparison result: '+old_name)
        same_camera(manifest['camera'],old_manifest['camera'])
        if old_manifest['lidar_frame']!=manifest['lidar_frame'] or not np.allclose(old_manifest['lidar_to_imu_translation'],manifest['lidar_to_imu_translation']):
            raise ValueError('Comparison frame/internal geometry mismatch')
        if set(args.validation)&set(old_manifest['training_scenes']) or any(
            d['cloud_sha256'] in old_manifest['scene_hashes'] for _,d in sources[len(args.training):]):
            raise ValueError('Comparison result trained on held-out validation data')
        candidates['previous_'+old_name]=dict(status='previous_unaccepted_comparison',
            T_camera_lidar=rigid_matrix(np.loadtxt(old/'extrinsic.txt',delimiter=',')).tolist(),
            source_result_sha256=record['sha256'],training_scenes=old_manifest['training_scenes'])
    for label,rs,ts in [('tight',.5,.005),('medium',1.,.01),('loose',2.,.02)]:
        print('FITTING '+label,flush=True)
        candidates[label]=fit(training,initial,rs,ts)
    # Refit fixed medium prior while leaving each training scene out. Never tune
    # scales or refit using a held-out validation scene.
    stability=[]
    for i in range(len(training)):
        fitted=fit(training[:i]+training[i+1:],initial,1.,.01)
        stability.append(dict(omitted=training[i]['name'],fit=fitted))
    for label,candidate in candidates.items():
        dump(run/(label+'.yaml'),candidate)
        if 'T_camera_lidar' not in candidate: continue
        transform=rigid_matrix(candidate['T_camera_lidar'])
        results[label]=[evaluate(g,transform,run/'comparison'/label/g['name']) for g in geometries]
    comparison=dict(training=args.training,validation=args.validation,candidates=candidates,
        metrics=results,leave_one_out=stability,
        status='review_required_no_automatic_acceptance',
        data_weighting='Fixed-support robust Chamfer /2px; 200 effective observations total, equal scene weights; independent quadratic measured prior',
        limitations='Prior scales are sensitivity assumptions. Nearest edges and extracted intersections may be wrong; no calibrated covariance. Translation is camera-frame offset, rotation is a left correction in camera frame. Physical IMU/LiDAR origin interpretation still requires verification.')
    dump(run/'study.yaml',comparison)
    marker.unlink()
    print('STUDY_READY '+str(run/'study.yaml'),flush=True)
    for label,metrics in results.items():
        print(json.dumps({'candidate':label,'status':candidates[label]['status'],
            'held_out':[m for m in metrics if m['scene'] in args.validation]},ensure_ascii=False),flush=True)


if __name__=='__main__':
    main()
