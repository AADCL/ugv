#!/usr/bin/env python3
"""Start the no-NDT test only when its hardware/node names are unused."""
import argparse
import os
import signal
import socket
import subprocess
import sys

import rosgraph
import rosnode

CONFLICTS = {'laserMapping','livox_lidar_publisher2','scout_base_node','move_base',
             'scout_global_localizer','scout_tf_manager','scout_geometry_tf_publisher',
             'scout_pose_adapter','scout_shadow_ekf','scout_fusion_guard','scout_odom_comparison'}


def main():
    parser = argparse.ArgumentParser(description='Manual driving comparison; no NDT, navigation or motion commands')
    parser.add_argument('--check-only',action='store_true',help='Read-only preflight, do not launch hardware')
    args = parser.parse_args()
    socket.setdefaulttimeout(3)
    try:
        if rosgraph.is_master_online():
            nodes = rosnode.get_node_names()
            conflicts = [n for n in nodes if n.rsplit('/',1)[-1] in CONFLICTS]
            if conflicts:
                print('REFUSED: existing localization/navigation/hardware nodes: '+', '.join(conflicts))
                print('Park, stop their owning launch sessions first. No nodes were killed or replaced.')
                return 2
        elif os.environ.get('ROS_MASTER_URI','http://localhost:11311').split('://')[-1].split(':')[0] not in ('localhost','127.0.0.1'):
            print('REFUSED: configured remote master unavailable; check ROS_MASTER_URI')
            return 2
    except Exception as error:
        print('REFUSED: unable to check ROS graph: '+str(error))
        return 2
    finally:
        socket.setdefaulttimeout(None)
    if args.check_only:
        print('PREFLIGHT_OK; hardware was NOT started')
        return 0
    print('Keep stationary until [COMPARE] READY. Manual driving only; park before Ctrl+C.',flush=True)
    process = subprocess.Popen(['roslaunch','scout_system_bringup','scout_fusion_test.launch'],start_new_session=True)
    try:
        return process.wait()
    except KeyboardInterrupt:
        os.killpg(process.pid,signal.SIGINT)
        try:
            return process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM)
            return process.wait(timeout=10)


if __name__ == '__main__':
    sys.exit(main())
