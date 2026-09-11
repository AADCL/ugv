"""Lifecycle checks use mocks: no arm service or chassis is contacted."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

path = Path(__file__).resolve().parents[1]/'scripts/explore_supervisor.py'
spec = importlib.util.spec_from_file_location('supervisor', str(path))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SupervisorStart(unittest.TestCase):
    def setUp(self):
        with patch.multiple(module.rospy, get_param=lambda name, default: default,
                            Publisher=Mock(), Subscriber=Mock(), ServiceProxy=Mock()):
            self.node = module.Supervisor()
        self.node.latched = False
        self.node.frontier_valid = True
        self.node.available = True
        self.node.frontier_since = 97.0
        self.fresh(100.0)

    def fresh(self, now):
        self.node.safety_stamp = self.node.status_stamp = self.node.frontier_stamp = now
        self.node.map_stamp = now-0.1
        self.node.refresh_ok = True

    def test_launch_auto_starts_only_when_ready(self):
        self.assertTrue(self.node.should_arm(100))
        self.assertFalse(self.node.can_start(100))  # wait for gate acknowledgement
        self.node.armed = True
        self.assertTrue(self.node.can_start(100))

    def test_diagnostic_mode_remains_locked(self):
        self.node.auto_start = False
        self.assertFalse(self.node.should_arm(100))
        self.node.available = False
        self.assertFalse(self.node.completed(100))

    def test_no_start_on_stale_missing_invalid_inputs(self):
        for field, bad in [('map_stamp', 0), ('safety_stamp', 98),
                           ('status_stamp', 98), ('frontier_stamp', 96),
                           ('frontier_valid', False), ('refresh_ok', False),
                           ('latched', True)]:
            with self.subTest(field=field):
                good = getattr(self.node, field)
                setattr(self.node, field, bad)
                self.assertFalse(self.node.should_arm(100))
                setattr(self.node, field, good)

    def test_no_automatic_rearm_after_stop(self):
        self.node.arm_requested = True
        self.assertFalse(self.node.should_arm(100))
        self.node.arm_requested = False
        self.node.ever_armed = True
        self.assertFalse(self.node.should_arm(100))

    def test_successful_auto_authorization_is_consumed_once(self):
        self.node.arm_service = Mock(return_value=SimpleNamespace(success=True))
        with patch.object(module.rospy, 'loginfo'):
            self.node.request_arm(100)
        self.assertTrue(self.node.arm_requested)
        self.assertFalse(self.node.should_arm(103))
        self.node.arm_service.assert_called_once_with()

    def test_completion_requires_time_and_two_new_maps(self):
        self.node.available = False
        self.assertFalse(self.node.completed(100))
        self.fresh(131)
        self.node.map_generation = 1
        self.assertFalse(self.node.completed(131))
        self.node.map_generation = 2
        self.assertTrue(self.node.completed(131))

    def test_active_goal_frontiers_and_invalid_data_reset_completion(self):
        for field, bad in [('available', True), ('active', True),
                           ('frontier_valid', False), ('latched', True),
                           ('refresh_ok', False), ('safety_stamp', 80)]:
            with self.subTest(field=field):
                self.node.available = False
                self.node.empty_since = 60
                self.node.empty_generation = 0
                self.node.map_generation = 4
                good = getattr(self.node, field)
                setattr(self.node, field, bad)
                self.assertFalse(self.node.completed(100))
                self.assertIsNone(self.node.empty_since)
                setattr(self.node, field, good)

    def test_duplicate_ready_message_is_not_new_map(self):
        message = module.String(data='ready: one_generation')
        self.node.map_status(message)
        self.node.map_status(message)
        self.assertEqual(self.node.map_generation, 1)
        self.node.map_status(module.String(data='refresh failed: no data'))
        self.assertFalse(self.node.refresh_ok)

    def test_online_memory_staleness_blocks_without_waiting_45_seconds(self):
        self.node.map_timeout = 2.0
        self.node.map_stamp = 97.5
        self.assertFalse(self.node.data_ready(100))
        self.node.map_stamp = 99.0
        self.assertTrue(self.node.data_ready(100))

    def test_fault_parks_session_without_saving_or_exiting(self):
        self.node.latched = True
        self.node.stop_child = Mock()
        self.node.stop_service = Mock()
        self.node.save_service = Mock()
        with patch.object(module.rospy, 'loginfo'):
            self.node.hold_session(fault=True)
        self.assertEqual(self.node.last_state,'FAULT_STOPPED')
        self.node.stop_child.assert_called_once_with()
        self.node.stop_service.assert_not_called()
        self.node.save_service.assert_not_called()
        self.assertFalse(self.node.should_arm(100))

    def test_data_pause_requires_stable_recovery_and_no_active_goal(self):
        self.node.pause_service = Mock(return_value=SimpleNamespace(success=True))
        self.node.stop_child = Mock()
        self.node.resume_service = Mock(return_value=SimpleNamespace(success=True))
        self.node.armed = True
        with patch.object(module.rospy, 'loginfo'):
            self.node.hold_session(fault=False)
        self.assertTrue(self.node.paused)
        self.assertFalse(self.node.completed(100))
        self.assertFalse(self.node.resume_if_ready(100))
        self.fresh(103)
        self.assertTrue(self.node.resume_if_ready(103))
        self.assertFalse(self.node.paused)
        self.node.resume_service.assert_called_once_with()

    def test_valid_empty_is_distinct_from_missing_tf(self):
        self.node.frontier(module.String(data='INVALID'))
        self.assertFalse(self.node.frontier_valid)
        self.node.frontier(module.String(data='EMPTY'))
        self.assertTrue(self.node.frontier_valid)
        self.assertFalse(self.node.available)

    def test_finish_stops_before_snapshot_and_never_converts(self):
        order = []
        self.node.stop_service = Mock(side_effect=lambda: order.append('stop'))
        self.node.stop_child = Mock(side_effect=lambda: order.append('child'))
        self.node.save_service = Mock(side_effect=lambda: (order.append('snapshot') or
                                                          SimpleNamespace(success=True)))
        self.node.result = 'COMPLETED'
        with patch.object(module.rospy, 'is_shutdown', return_value=False), \
             patch.object(module.rospy, 'loginfo'), patch.object(module.subprocess, 'Popen') as spawn:
            self.node.finish()
        self.assertEqual(order, ['stop', 'child', 'snapshot'])
        self.assertEqual(self.node.result, 'COMPLETED')
        spawn.assert_not_called()

    def test_snapshot_failure_is_not_reported_as_success(self):
        self.node.stop_service = Mock()
        self.node.stop_child = Mock()
        self.node.save_service = Mock(return_value=SimpleNamespace(success=False, message='disk full'))
        self.node.result = 'COMPLETED'
        with patch.object(module.rospy, 'is_shutdown', return_value=False), \
             patch.object(module.rospy, 'loginfo'), patch.object(module.rospy, 'logerr'):
            self.node.finish()
        self.assertEqual(self.node.result, 'SAVE_FAILED')


adapter_path = path.parent/'frontier_costmap_adapter.py'
adapter_spec = importlib.util.spec_from_file_location('adapter', str(adapter_path))
adapter_module = importlib.util.module_from_spec(adapter_spec)
adapter_spec.loader.exec_module(adapter_module)


class FrontierEvidence(unittest.TestCase):
    def setUp(self):
        self.node = adapter_module.FrontierCostmapAdapter.__new__(adapter_module.FrontierCostmapAdapter)
        self.node.min_frontier_cells = 5
        self.node.robot_base_frame = 'base_link'
        self.node.tf_buffer = Mock()

    def test_missing_tf_is_invalid_not_empty(self):
        self.node.tf_buffer.lookup_transform.side_effect = adapter_module.tf2_ros.TransformException('missing')
        grid = adapter_module.OccupancyGrid()
        with patch.object(adapter_module.rospy, 'logwarn_throttle'):
            self.assertIsNone(self.node.has_reachable_frontier_cluster(grid))

    def test_physical_frontier_size_and_row_edges(self):
        self.assertFalse(self.node.has_large_cluster(set(range(9)), 20, required=10))
        self.assertTrue(self.node.has_large_cluster(set(range(10)), 20, required=10))
        # Right edge of one row must not connect to left edge of the next.
        self.assertFalse(self.node.has_large_cluster({9, 10}, 10, required=2))


if __name__ == '__main__':
    unittest.main()
