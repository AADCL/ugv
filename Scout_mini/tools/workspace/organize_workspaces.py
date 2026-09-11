#!/usr/bin/env python3
"""One-time Scout 120 cleanup; dry-run by default. CCS is outside scope."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil

BASE = Path('/home/nvidia')
NAV = BASE / 'livox_fastlio'
R3 = BASE / 'r3live_ws'
WORKSPACES = ('r3live_ws', 'realsense_ws', 'lidar_camera_calib_ws')
DATA = [R3 / 'calibration', R3 / 'calibration_test',
        R3 / 'calibration_synthetic_20260910', R3 / 'calibration_synthetic_20260910b',
        R3 / 'logs', NAV / 'logs/navigation', NAV / 'logs/corridor_VveuzrBr',
        NAV / 'logs/fusion_tests', NAV / 'logs/fusion_script_verification',
        BASE / 'image1', NAV / 'src/FAST_LIO/PCD/scans.pcd']
TRIAL = R3 / 'calibration/trials/indoor_20260911_01'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def scope(path):
    resolved = path.resolve(strict=True)
    if BASE not in resolved.parents or resolved in (NAV.resolve(), BASE):
        raise ValueError('Unsafe scope: ' + str(path))
    if BASE / 'ccs_edge_ws' in resolved.parents or resolved == BASE / 'ccs_edge_ws':
        raise ValueError('CCS protected: ' + str(path))
    if NAV / 'maps' == resolved or NAV / 'maps' in resolved.parents:
        raise ValueError('Production maps protected: ' + str(path))
    if path.is_symlink():
        raise ValueError('Deletion/move symlink refused: ' + str(path))


def inventory(path):
    scope(path)
    files = [path] if path.is_file() else [p for p in path.rglob('*') if p.is_file() and not p.is_symlink()]
    return {'path': str(path), 'files': len(files), 'bytes': sum(p.stat().st_size for p in files)}


def plan():
    targets = [p for p in DATA if p.exists()]
    for root, dirs, names in os.walk(BASE, followlinks=False):
        root = Path(root)
        dirs[:] = [d for d in dirs if not (root / d).is_symlink()
                   and d not in ('.git', 'ccs_edge_ws')
                   and root / d != NAV / 'maps']
        for name in names:
            p = root / name
            if (name.endswith(('.bag', '.bag.active', '.bag.orig')) and not p.is_symlink()
                    and not any(t == p or t in p.parents for t in targets)):
                targets.append(p)
    return [inventory(p) for p in targets]


def idle(paths):
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            args = [x.decode(errors='replace') for x in (proc / 'cmdline').read_bytes().split(b'\0') if x]
            direct = args[:2] if args and Path(args[0]).name.startswith('python') else args[:1]
            if any(Path(x).name in {'roslaunch', 'roscore', 'rosmaster', 'rosbag', 'rviz'}
                   or '/devel/lib/' in x for x in direct):
                raise RuntimeError('Stop ROS first: pid ' + proc.name)
            for fd in (proc / 'fd').iterdir():
                try:
                    target = fd.resolve(strict=True)
                except OSError:
                    continue
                if any(target == p or p in target.parents for p in paths):
                    raise RuntimeError('Open target: ' + str(target) + ' pid ' + proc.name)
        except (OSError, PermissionError):
            continue


def apply(items):
    moves = [(BASE / name, NAV / 'optional' / name) for name in WORKSPACES]
    for source, dest in moves:
        scope(source)
        if dest.exists() or dest.is_symlink():
            raise FileExistsError(str(dest))
        if source.stat().st_dev != NAV.stat().st_dev:
            raise ValueError('Cross-filesystem move refused')
    idle([Path(i['path']).resolve() for i in items] + [p for p, _ in moves])
    entry = (R3 / 'start_scout_r3live_test.sh').read_text()
    if 'config/accepted_20260911' not in entry:
        raise RuntimeError('Deploy the updated test entry before cleanup')
    # Preserve the selected calibration and ALL original configuration backups.
    config = R3 / 'config/accepted_20260911'
    config.mkdir(parents=True, exist_ok=False)
    keep = ['trial_calibration.yaml', 'source_candidate.yaml', 'source_manifest.yaml',
            'backup_hashes.yaml', 'expected_runtime_extrinsic.yaml']
    hashes = {}
    for name in keep:
        source = TRIAL / name
        shutil.copy2(source, config / name)
        hashes[name] = digest(source)
        assert digest(config / name) == hashes[name]
    shutil.copytree(TRIAL / 'backup', config / 'backup')
    for source in (TRIAL / 'backup').rglob('*'):
        if source.is_file():
            assert digest(source) == digest(config / 'backup' / source.relative_to(TRIAL / 'backup'))
    shutil.copy2(R3 / 'calibration/runtime_hardening_20260911/before_fix.tar.gz',
                 config / 'runtime_before_fix.tar.gz')
    assert digest(config / 'runtime_before_fix.tar.gz') == digest(R3 / 'calibration/runtime_hardening_20260911/before_fix.tar.gz')
    records = NAV / 'maintenance' / datetime.now().strftime('cleanup_%Y%m%d_%H%M%S')
    records.mkdir(parents=True, exist_ok=False)
    (records / 'plan.json').write_text(json.dumps(items, indent=2))
    (records / 'calibration_sha256.json').write_text(json.dumps(hashes, indent=2))
    with (records / 'journal.jsonl').open('x') as journal:
        def event(data):
            journal.write(json.dumps(data) + '\n')
            journal.flush()
            os.fsync(journal.fileno())
        for item in items:
            p = Path(item['path'])
            scope(p)
            event(dict(item, action='delete', state='planned'))
            if p.is_dir():
                shutil.rmtree(p)
            else:
                p.unlink()
                if p.parent == BASE / '.local/share/Trash/files':
                    info = BASE / '.local/share/Trash/info' / (p.name + '.trashinfo')
                    if info.is_file() and not info.is_symlink():
                        info.unlink()
            event(dict(item, action='delete', state='complete'))
        for source, dest in moves:
            dest.parent.mkdir(parents=True, exist_ok=True)
            identity = (source.stat().st_dev, source.stat().st_ino)
            event(dict(action='move', source=str(source), destination=str(dest), state='planned'))
            source.rename(dest)
            assert (dest.stat().st_dev, dest.stat().st_ino) == identity
            source.symlink_to(dest, target_is_directory=True)
            event(dict(action='move', source=str(source), destination=str(dest), state='complete'))
    print(json.dumps({'deleted_bytes': sum(i['bytes'] for i in items),
                      'deleted_files': sum(i['files'] for i in items),
                      'manifest': str(records), 'moved': WORKSPACES}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    items = plan()
    if args.apply:
        apply(items)
    else:
        print(json.dumps({'delete': items, 'bytes': sum(i['bytes'] for i in items),
                          'move': WORKSPACES, 'preserve': ['ccs_edge_ws', 'livox_fastlio/maps',
                          'selected calibration and configuration backups']}, indent=2))
