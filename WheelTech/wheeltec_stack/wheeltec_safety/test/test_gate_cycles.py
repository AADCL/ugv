"""Actual gate callbacks/cycles with no ROS transport, services or threads."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock, patch
import yaml
import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('gate_cycles', str(ROOT/'scripts/cmd_vel_safety_gate.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class GateCycles(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        config = yaml.safe_load((ROOT/'config/cmd_vel_safety.yaml').read_text())
        config['allow_stationary_command_wait'] = True
        self.identity = TransformStamped()
        self.identity.header.stamp = rospy.Time(100)
        self.identity.transform.rotation.w = 1
        patches = [
            patch.multiple(module.rospy, get_name=lambda:'/wheeltec_safety',
                remap_name=lambda n:n, get_param=lambda n,d:config.get(n.lstrip('~'),d),
                Publisher=Mock(side_effect=lambda *a,**k:Mock()), Subscriber=Mock(), Service=Mock(), on_shutdown=Mock(),
                logwarn=Mock(), logerr=Mock(), loginfo=Mock()),
            patch.object(module.threading.Thread, 'start'),
            patch.object(module.tf2_ros, 'Buffer', return_value=Mock(lookup_transform=Mock(return_value=self.identity))),
            patch.object(module.tf2_ros, 'TransformListener'),
            patch.object(module.time, 'monotonic', side_effect=lambda:self.now),
            patch.object(module.rospy.Time, 'now', side_effect=lambda:rospy.Time.from_sec(self.now)),
        ]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        self.gate = module.CmdVelSafetyGate()
        self.gate.armed = True  # isolated object, never a ROS service
        self.goal = MoveBaseActionGoal()
        self.goal.goal_id.id = 'isolated_test_goal'
        self.goal.goal_id.stamp = rospy.Time(1)
        self.gate.move_base_goal_callback(self.goal)
        self.gate.move_base_status_callback(GoalStatusArray(status_list=[
            GoalStatus(goal_id=self.goal.goal_id,status=GoalStatus.ACTIVE)]))
        self.command = self.twist(.2)
        self.gate.command_callback(self.command)
        self.gate.local_plan_callback(NavPath(poses=[PoseStamped(),PoseStamped()]))
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = 'base_link'
        self.grid.info.width = self.grid.info.height = 120
        self.grid.info.resolution = .05
        self.grid.info.origin.position.x = self.grid.info.origin.position.y = -3
        self.grid.info.origin.orientation.w = 1
        self.grid.data = [0]*14400
        self.gate.costmap_callback(self.grid)
        self.empty = self.cloud([])
        self.floor = self.cloud([(.5+i*.01,.5,-.15) for i in range(20)])
        self.gate.cloud_callback(self.empty)
        self.gate.raw_cloud_callback(self.floor)
        self.gate.output_publisher.reset_mock()

    @staticmethod
    def twist(v, w=0):
        result=Twist()
        result.linear.x, result.angular.z = v,w
        result._connection_header={'callerid':'/move_base'}
        return result

    def cloud(self, points):
        return point_cloud2.create_cloud_xyz32(Header(frame_id='base_link',stamp=rospy.Time(100)),points)

    def inject(self, callback, repeat=False):
        original = self.gate.costmap_stop_check
        used = [False]
        def check(*args):
            result = original(*args)
            if not used[0] or repeat:
                used[0] = True
                callback()
            return result
        self.gate.costmap_stop_check = check

    def test_normal_refreshes_do_not_insert_any_zero(self):
        def refresh():
            self.gate.command_callback(self.command)
            self.gate.cloud_callback(self.empty)
            self.gate.raw_cloud_callback(self.floor)
            self.gate.costmap_callback(self.grid)
        self.inject(refresh, repeat=True)
        for _ in range(500):
            self.now += .05
            self.identity.header.stamp = rospy.Time.from_sec(self.now)
            self.gate.local_plan_callback(NavPath(poses=[PoseStamped(),PoseStamped()]))
            self.gate.move_base_status_callback(GoalStatusArray(status_list=[
                GoalStatus(goal_id=self.goal.goal_id,status=GoalStatus.ACTIVE)]))
            refresh()
            self.gate.run_cycle()
            self.assertEqual(self.gate.last_output.linear.x, .2)
            self.assertFalse(self.gate.latched_stop)
        outputs=[c.args[0] for c in self.gate.output_publisher.publish.call_args_list]
        self.assertEqual(len(outputs),500)
        self.assertTrue(all(o.linear.x==.2 for o in outputs))

    def test_changed_command_is_checked_again_in_same_cycle(self):
        self.inject(lambda:self.gate.command_callback(self.twist(.15,.1)))
        self.gate.run_cycle()
        self.assertEqual(self.gate.last_recheck_count,1)
        self.assertEqual(self.gate.last_output.linear.x,.15)
        self.assertEqual(self.gate.last_output.angular.z,.1)
        self.assertEqual(self.gate.output_publisher.publish.call_count,1)

    def test_new_hazard_during_check_stops_in_same_cycle(self):
        self.inject(lambda:self.gate.cloud_callback(self.cloud([(.35,0,.1)])))
        self.gate.run_cycle()
        self.assertTrue(self.gate.latched_stop)
        self.assertEqual(self.gate.last_output.linear.x,0)
        self.assertEqual(self.gate.latched_reason,'obstacle_in_cloud_stop_region')

    def test_new_costmap_hazard_is_rechecked(self):
        def obstacle():
            self.grid.data[60*120+67]=100
            self.gate.costmap_callback(self.grid)
        self.inject(obstacle)
        self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'obstacle_in_costmap_stop_region')

    def test_stop_and_zero_commands_cannot_be_overwritten(self):
        for callback in [lambda:self.gate.command_callback(self.twist(0)),
                         lambda:self.gate.stop_callback(None)]:
            self.inject(callback)
            self.gate.run_cycle()
            self.assertEqual(self.gate.last_output.linear.x,0)

    def test_first_latch_reason_survives_refresh_and_operator_stop(self):
        self.gate.cloud_callback(self.cloud([(.35,0,.1)]))
        self.gate.run_cycle()
        reason=self.gate.latched_reason
        self.inject(lambda:self.gate.costmap_callback(self.grid),repeat=True)
        self.gate.run_cycle()
        self.gate.stop_callback(None)
        self.assertEqual(self.gate.last_reason,reason)
        self.assertEqual(self.gate.latched_reason,reason)

    def test_stale_command_is_still_latched(self):
        self.now += .21
        self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'command_stream_stale')

    def test_reset_cannot_use_clearance_from_an_outdated_snapshot(self):
        self.gate.stop_callback(None)
        self.inject(lambda:self.gate.cloud_callback(self.cloud([(.35,0,.1)])))
        self.gate.run_cycle()
        self.assertFalse(self.gate.reset_callback(None).success)
        self.assertTrue(self.gate.latched_stop)

    def test_reverse_arc_is_not_executed_as_rotation(self):
        self.gate.command_callback(self.twist(-.01,.2))
        self.gate.run_cycle()
        self.assertEqual(self.gate.latched_reason,'reverse_command_inhibited')
        self.assertEqual(self.gate.last_output.angular.z,0)

    def test_unbounded_changing_data_has_bounded_retries_and_stops(self):
        n=[0]
        def update():
            n[0]+=1
            self.gate.cloud_callback(self.cloud([(.8,.8,n[0]*.01)]))
        self.inject(update,repeat=True)
        self.gate.run_cycle()
        self.assertEqual(n[0],3)
        self.assertEqual(self.gate.last_output.linear.x,0)
        self.assertEqual(self.gate.last_reason,'collision_check_budget_exhausted')

    def test_pause_cancels_without_resetting_or_rearming(self):
        self.gate.pause_callback(None)
        self.gate.move_base_status_callback(GoalStatusArray(status_list=[
            GoalStatus(goal_id=self.goal.goal_id,status=GoalStatus.PREEMPTED)]))
        self.gate.run_cycle()
        self.assertTrue(self.gate.armed)
        self.assertFalse(self.gate.latched_stop)
        self.assertEqual(self.gate.last_output.linear.x,0)
        self.assertTrue(self.gate.resume_callback(None).success)
        self.gate.run_cycle()
        self.assertEqual(self.gate.last_output.linear.x,0)


if __name__=='__main__':
    unittest.main()
