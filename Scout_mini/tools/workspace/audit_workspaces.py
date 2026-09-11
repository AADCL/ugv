#!/usr/bin/env python3
"""Read-only Scout source/deployment inventory. Does not inspect credentials."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

BASE = Path('/home/nvidia')
REPO = BASE/'github_upload/ugv'
WORKSPACES = ('livox_fastlio','r3live_ws','lidar_camera_calib_ws','realsense_ws')

def run(args):
    result = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            universal_newlines=True, timeout=45)
    return result.stdout.strip() if result.returncode == 0 else 'UNAVAILABLE'

def digest(path):
    data = path.read_bytes()
    if b'\0' not in data:
        data = data.replace(b'\r\n', b'\n')
    return hashlib.sha256(data).hexdigest()

def inventory():
    report = {'repository':str(REPO), 'revision':run(['git','-C',str(REPO),'rev-parse','HEAD']),
              'workspaces':{}, 'deployments':{}}
    for name in WORKSPACES:
        ws = BASE/name
        packages=[]
        for package in sorted((ws/'src').rglob('package.xml')):
            try:
                packages.append({'name':ET.parse(str(package)).getroot().findtext('name'),
                                 'path':str(package.parent)})
            except ET.ParseError:
                packages.append({'error':'invalid package.xml','path':str(package)})
        underlay=ws/'devel/.catkin'
        repos=[]
        for git in sorted((ws/'src').glob('*/.git')):
            folder=git.parent
            repos.append({'path':str(folder),'revision':run(['git','-C',str(folder),'rev-parse','HEAD']),
                          'changes':run(['git','-C',str(folder),'status','--porcelain'])})
        report['workspaces'][name]={'packages':packages,'upstream_repositories':repos,
            'size':run(['du','-sh',str(ws.resolve())]).split('\t')[0],
            'catkin_sources':underlay.read_text() if underlay.exists() else None}
    mappings = {
        'navigation':('Scout_mini/src/',BASE/'livox_fastlio/src'),
        'r3live_bringup':('Scout_mini/optional/r3live/scout_r3live_bringup/',BASE/'r3live_ws/src/scout_r3live_bringup'),
        'calibration_tools':('Scout_mini/optional/r3live/calibration/',BASE/'r3live_ws/calibration_tools')}
    for name,(prefix,destination) in mappings.items():
        files=run(['git','-C',str(REPO),'ls-files',prefix]).splitlines()
        comparison={'equal':0,'different':[],'missing':[]}
        for file in files:
            if name=='calibration_tools' and not file.endswith('.py'):
                continue
            relative=file[len(prefix):]
            original=REPO/file; deployed=destination/relative
            if not deployed.is_file():
                comparison['missing'].append(relative)
            elif digest(original)!=digest(deployed):
                comparison['different'].append(relative)
            else:
                comparison['equal']+=1
        report['deployments'][name]=comparison
    return report

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    result=inventory()
    with Path(args.output).open('x') as output:
        json.dump(result,output,ensure_ascii=False,indent=2)
    print(json.dumps({'revision':result['revision'],'deployments':result['deployments'],
                      'workspaces':{k:{'packages':len(v['packages']),'size':v['size']} for k,v in result['workspaces'].items()}}))
