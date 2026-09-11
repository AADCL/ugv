#!/usr/bin/env python3
"""Back up installation defaults and package an explicitly authorized trial."""
import argparse
from datetime import datetime,timezone
from pathlib import Path
import shlex
import shutil
import sys
import numpy as np
sys.path.insert(0,'/home/nvidia/r3live_ws/src/scout_r3live_bringup/scripts')
from calibration_io import rigid_matrix
from calibrate import read,dump,digest,safe_name

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('name'); p.add_argument('--confirm-trial',action='store_true')
a=p.parse_args()
if not a.confirm_trial: p.error('Explicit authorization for indoor trial is required')
ws=Path('/home/nvidia/r3live_ws')
run=ws/'calibration/runs/prior_robust_final_20260911'
if not (run/'loose.yaml').is_file():
    p.error('Historical study data was cleaned. Use ~/r3live_ws/start_scout_r3live_test.sh '
            'with the preserved config/accepted_20260911, or collect a new study.')
candidate=read(run/'loose.yaml'); manifest=read(run/'manifest.yaml')
if (run/'.study_incomplete').exists() or candidate['status']!='candidate_not_accepted':
    raise ValueError('Incomplete or rejected study candidate')
t=rigid_matrix(candidate['T_camera_lidar'])
# Guard against selecting a later corner candidate by accident.
expected=np.array([[.014183137267383399,-.998928415965608,.04405517442589314,.004579043956485155],
    [.7287335940743439,-.019841731744050316,-.684509791419295,-.10586853916919002],
    [.6846504126083209,.04181298192952107,.72767127678466,.004208719896754375],[0,0,0,1]])
if not np.allclose(t,expected,rtol=0,atol=1e-12): raise ValueError('Selected candidate differs from user-reviewed matrix')
trial=ws/'calibration/trials'/safe_name(a.name); trial.mkdir(parents=True,exist_ok=False)
backup=trial/'backup'; backup.mkdir()
for path in [ws/'src/scout_r3live_bringup/config/rig.yaml',
             ws/'src/scout_r3live_bringup/config/estimator.yaml',ws/'start_scout_r3live.sh',
             ws/'src/scout_r3live_bringup/launch/scout_r3live.launch']:
    shutil.copy2(path,backup/path.name)
shutil.copy2(run/'loose.yaml',trial/'source_candidate.yaml')
shutil.copy2(run/'manifest.yaml',trial/'source_manifest.yaml')
dump(backup/'default_T_camera_lidar.yaml',{'T_camera_lidar':manifest['initial_T_camera_lidar'],
    'scope':'Recorded default geometry; original IMU interpretation, not corrected casing origin'})
record={'schema':1,'status':'operator_accepted','approval_scope':'indoor_trial_only',
    'convention':'p_camera_optical = T_camera_lidar * p_lidar',
    'T_camera_lidar':t.tolist(),'camera':manifest['camera'],'lidar_frame':manifest['lidar_frame'],
    'lidar_to_imu_translation':manifest['lidar_to_imu_translation'],
    'accepted_utc':datetime.now(timezone.utc).isoformat(),'source_candidate_sha256':digest(run/'loose.yaml'),
    'time_calibration':'not_estimated','independent_accuracy_validation':'not_passed',
    'note':'User explicitly authorized this reviewed candidate for an indoor trial. Operator acceptance is deployment permission, not a claim of calibration accuracy. Original defaults are backed up and unchanged.'}
dump(trial/'trial_calibration.yaml',record)
imu_lidar=np.eye(4); imu_lidar[:3,3]=record['lidar_to_imu_translation']
dump(trial/'expected_runtime_extrinsic.yaml',{'T_imu_camera':rigid_matrix(imu_lidar@np.linalg.inv(t)).tolist()})
start='#!/usr/bin/env bash\nset -eo pipefail\nexec /home/nvidia/r3live_ws/start_scout_r3live.sh calibration_file:='+shlex.quote(str(trial/'trial_calibration.yaml'))+' output_root:='+shlex.quote(str(trial/'sessions'))+' record_bag:=true test_duration:=600 "$@"\n'
(trial/'start.sh').write_text(start); (trial/'start.sh').chmod(0o755)
dump(trial/'backup_hashes.yaml',{f.name:digest(f) for f in backup.iterdir()})
print('TRIAL_READY '+str(trial),flush=True)
