#!/usr/bin/env python3
"""Verify operational composition and public check_only entries without starting hardware."""
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import unittest

import rospkg
import roslaunch

PACK = Path(rospkg.RosPack().get_path('scout_r3live_bringup'))
SYSTEM = Path(rospkg.RosPack().get_path('scout_system_bringup'))
sys.path.insert(0, str(PACK/'scripts'))
from operation_contract import prepare_pipeline, validate_mode


class OperationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='scout_operations_')
        self.root = Path(self.temp.name)
        self.map = self.root/'map'
        # Copy a read-only production bundle; all mutation tests touch only this copy.
        source = Path(os.environ.get('SCOUT_TEST_MAP_DIRECTORY','/home/nvidia/livox_fastlio/maps/indor'))
        shutil.copytree(source,self.map)

    def tearDown(self):
        self.temp.cleanup()

    def config(self,mode):
        root=self.root/'new_mapping' if mode=='mapping' else self.map
        args=prepare_pipeline(PACK,SYSTEM,mode,'test_map',str(root),True)
        return roslaunch.config.load_config_default([(str(PACK/'launch/operation_pipeline.launch'),args)],None)

    def test_composition_and_unchanged_navigation(self):
        for mode in ('local','mapping','localization','navigation'):
            c=self.config(mode); names=[n.name for n in c.nodes]
            self.assertEqual(len(names),len(set(names)))
            self.assertIn('scout_base_node',names)
            self.assertNotIn('laserMapping',names)
            self.assertNotIn('scout_pose_adapter',names)
            self.assertNotIn('scout_tf_manager',names)  # already owned by estimator/interface
            self.assertEqual('scout_pointcloud_mapper' in names,mode=='mapping')
            self.assertEqual('scout_global_localizer' in names,mode in ('localization','navigation'))
            self.assertEqual('move_base' in names,mode=='navigation')
            self.assertFalse(c.params['/scout_base_node/pub_tf'].value)
            self.assertEqual(c.params['/scout_base_node/odom_topic_name'].value,'/scout/odom')
        nav=self.config('navigation')
        baseline=roslaunch.config.load_config_default([(
            str(Path(rospkg.RosPack().get_path('scout_navigation'))/'launch/navigation_teb.launch'),
            ['map_name:=test_map','map_dir:='+str(self.map)])],None)
        values=lambda c:{k:v.value for k,v in c.params.items() if k.startswith('/move_base/')}
        self.assertEqual(values(nav),values(baseline))
        self.assertIn('/scout_r3live_localization_map_bundle_guard/required_files',nav.params)
        self.assertIn('/scout_navigation_map_bundle_guard/required_files',nav.params)

    def test_refuses_overwrite_invalid_map_and_invalid_mode(self):
        with self.assertRaisesRegex(ValueError,'nonempty'):
            prepare_pipeline(PACK,SYSTEM,'mapping','test_map',str(self.map),True)
        (self.map/'.finalization_incomplete').touch()
        with self.assertRaisesRegex(ValueError,'incomplete'):
            self.config('localization')
        (self.map/'.finalization_incomplete').unlink()
        (self.map/'terrain_2p5d_cost.u8').unlink()
        with self.assertRaisesRegex(ValueError,'missing'):
            self.config('navigation')
        for mode in ('local','mapping','localization','navigation'):
            validate_mode(mode,0,True)
            with self.assertRaises(ValueError): validate_mode(mode,0,False)
        with self.assertRaises(ValueError): validate_mode('test',0,True)
        with self.assertRaises(ValueError): validate_mode('unknown',600,True)

    def test_public_entries_check_only(self):
        # Each roslaunch owns only a temporary private master and the check-only wrapper.
        # No pipeline or sensors are allowed to start, even for navigation mode.
        env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:11339',ROS_HOSTNAME='127.0.0.1',
                 ROS_LOG_DIR=str(self.root/'roslogs'))
        env.pop('ROS_IP',None)
        with socket.socket() as probe: probe.bind(('127.0.0.1',11339))
        for mode in ('local','mapping','localization','navigation'):
            root=self.root/'new_mapping' if mode=='mapping' else self.map
            result=subprocess.run(['roslaunch','scout_system_bringup','scout_r3live_'+mode+'.launch',
                'check_only:=true','map_name:=test_map','map_dir:='+str(root)],
                env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=35)
            self.assertEqual(result.returncode,0,result.stdout)
            self.assertIn('PREFLIGHT_OK',result.stdout)
            self.assertNotIn('process[livox_lidar_publisher2',result.stdout)
            self.assertNotIn('process[scout_base_node',result.stdout)
            self.assertNotIn('process[move_base',result.stdout)
            self.assertFalse((self.root/'new_mapping').exists())


if __name__=='__main__': unittest.main()
