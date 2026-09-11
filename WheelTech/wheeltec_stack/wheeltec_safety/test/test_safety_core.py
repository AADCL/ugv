#!/usr/bin/env python3

import math
import unittest

from wheeltec_safety.safety_core import (
    DRIVER_OUTPUT_TOPIC,
    DirectionalStopRegion,
    EXPECTED_NAV_CALLER_ID,
    EXPECTED_NODE_NAME,
    MoveBaseGoalTracker,
    NAV_INPUT_TOPIC,
    SafetyInterlock,
    apply_grid_update,
    command_caller_is_expected,
    goal_handshake_fault,
    motion_fault_reason,
    snapshot_is_current,
    vertical_span_obstacle_cells,
)


def region():
    return DirectionalStopRegion(
        front=0.25,
        rear=0.25,
        half_width=0.20,
        footprint_margin=0.02,
        min_forward_clearance=0.18,
        min_reverse_clearance=0.18,
        reaction_time=0.20,
        linear_deceleration=0.20,
        angular_deceleration=0.40,
        max_prediction_horizon=1.50,
        prediction_samples=12,
        linear_deadband=0.005,
        angular_deadband=0.01,
    )


def fault(**overrides):
    values = dict(
        linear_x=0.20,
        angular_z=0.0,
        command_valid=True,
        command_age=0.05,
        command_timeout=0.25,
        has_live_goal=True,
        status_age=0.05,
        status_timeout=1.0,
        allow_reverse=False,
        linear_deadband=0.005,
        angular_deadband=0.01,
        require_local_plan=True,
        local_plan_age=0.05,
        local_plan_timeout=0.35,
        local_plan_poses=8,
        cloud_blocked=False,
        costmap_blocked=False,
    )
    values.update(overrides)
    return motion_fault_reason(**values)


class SafetyCoreTest(unittest.TestCase):
    def test_vectorized_geometry_matches_scalar_including_reverse_and_turns(self):
        import numpy as np
        rng = np.random.RandomState(42)
        points = np.vstack((rng.uniform(-1, 1, (1500, 2)),
                            [[.43, .22], [.27, 0], [-.43, -.22], [0, 0]]))
        r = region()
        for v, w in [(0,0), (.2,0), (-.2,0), (0,.4), (0,-.4), (.2,.4), (.1,-.3)]:
            poses = r.prediction_poses(v, w)
            distances = [math.hypot(x,y) for x,y in points
                         if poses and r.contains(x,y,v,w,poses)]
            blocked, distance = r.clearance(points, v, w)
            self.assertEqual(blocked, bool(distances))
            if distances:
                self.assertAlmostEqual(distance, min(distances), places=12)
            else:
                self.assertEqual(distance, math.inf)
        self.assertEqual(r.clearance([], .2, 0), (False, math.inf))

    def test_planner_startup_wait_still_expires_and_motion_timeout_unchanged(self):
        self.assertIsNone(goal_handshake_fault(True, 10.0, 12.0, False, False, 0))
        self.assertEqual(goal_handshake_fault(True, 12.1, 12.0, False, False, 0),
                         'goal_handshake_timeout')
        self.assertEqual(fault(command_age=.21, command_timeout=.20),
                         'command_stream_stale')

    def test_startup_is_locked_and_live_goal_blocks_arm(self):
        interlock = SafetyInterlock()
        self.assertFalse(interlock.armed)
        self.assertFalse(interlock.latched)
        self.assertEqual(interlock.reason, "startup_lock")
        success, _ = interlock.arm(health_ok=True, has_live_goal=True)
        self.assertFalse(success)
        self.assertFalse(interlock.armed)
        success, _ = interlock.arm(health_ok=True, has_live_goal=False)
        self.assertTrue(success)
        self.assertTrue(interlock.armed)

    def test_nonzero_command_timeout_is_a_latched_fault_reason(self):
        self.assertEqual(
            fault(command_age=0.30),
            "command_stream_stale",
        )

    def test_stopped_planner_gap_keeps_goal_without_latching(self):
        # Recorded NX123 fault: zero command age .22866 s, active goal,
        # stale TEB path while the planner reports an infeasible trajectory.
        self.assertIsNone(fault(linear_x=0.0, angular_z=0.0,
                                allow_stationary_command_wait=True,
                                command_age=.22866, command_timeout=.20,
                                local_plan_age=1.351, local_plan_poses=51))
        self.assertEqual(fault(linear_x=.1, command_age=.22866,
                               allow_stationary_command_wait=True,
                               command_timeout=.20), 'command_stream_stale')
        self.assertEqual(fault(linear_x=0.0, angular_z=.1,
                               allow_stationary_command_wait=True,
                               command_age=.22866, command_timeout=.20),
                         'command_stream_stale')
        self.assertEqual(fault(linear_x=0.0, command_age=.22866,
                               allow_stationary_command_wait=True,
                               command_timeout=.20, status_age=2.0),
                         'move_base_status_stale')
        # Existing consumers retain their original policy unless they opt in.
        self.assertEqual(fault(linear_x=0.0, angular_z=0.0,
                               command_age=.22866, command_timeout=.20),
                         'command_stream_stale')

    def test_forward_chair_leg_enters_stop_region(self):
        stop_region = region()
        self.assertTrue(stop_region.contains(0.35, 0.0, 0.20, 0.0))
        self.assertFalse(stop_region.contains(0.35, 0.30, 0.20, 0.0))

    def test_rotation_checks_swept_side_of_rectangular_body(self):
        stop_region = region()
        # Outside the static half-width, but inside the CCW swept footprint.
        self.assertTrue(stop_region.contains(0.07, 0.28, 0.0, 0.40))
        self.assertFalse(stop_region.contains(0.0, 0.40, 0.0, 0.40))

    def test_reverse_is_inhibited_without_rear_coverage(self):
        self.assertEqual(
            fault(linear_x=-0.05),
            "reverse_command_inhibited",
        )

    def test_stale_or_empty_local_plan_stops_nonzero_command(self):
        self.assertEqual(
            fault(local_plan_age=0.40),
            "local_plan_stale",
        )
        self.assertEqual(
            fault(local_plan_poses=0),
            "local_plan_empty",
        )

    def test_nonzero_command_requires_a_live_move_base_goal(self):
        self.assertEqual(
            fault(has_live_goal=False),
            "motion_without_live_goal",
        )

    def test_zero_command_does_not_require_a_live_goal_or_plan(self):
        self.assertIsNone(
            fault(
                linear_x=0.0,
                angular_z=0.0,
                has_live_goal=False,
                local_plan_age=float("inf"),
                local_plan_poses=0,
            )
        )

    def test_costmap_update_accepts_corner_and_rejects_overflow(self):
        updated = apply_grid_update(
            [0] * 9,
            width=3,
            height=3,
            x=2,
            y=2,
            update_width=1,
            update_height=1,
            update_data=[100],
        )
        self.assertEqual(updated[-1], 100)
        with self.assertRaises(ValueError):
            apply_grid_update(
                [0] * 9,
                width=3,
                height=3,
                x=2,
                y=2,
                update_width=2,
                update_height=1,
                update_data=[100, 100],
            )

    def test_first_goal_is_immediate_and_waits_for_matching_status(self):
        tracker = MoveBaseGoalTracker()
        self.assertTrue(tracker.accept_goal("goal-a", now=1.0))
        self.assertEqual(tracker.generation, 1)
        self.assertEqual(tracker.current_id, "goal-a")
        self.assertEqual(tracker.live_since, 1.0)
        self.assertFalse(tracker.status_confirmed)
        self.assertEqual(tracker.observe_status(set(), set()), "missing")
        self.assertEqual(tracker.current_id, "goal-a")
        self.assertEqual(tracker.live_since, 1.0)
        self.assertEqual(
            tracker.observe_status({"goal-a"}, set()), "confirmed"
        )
        self.assertTrue(tracker.status_confirmed)

    def test_a_to_b_goal_preemption_ignores_late_a_status(self):
        tracker = MoveBaseGoalTracker()
        tracker.accept_goal("goal-a", now=1.0)
        tracker.observe_status({"goal-a"}, set())
        self.assertTrue(tracker.accept_goal("goal-b", now=2.0))
        self.assertEqual(tracker.generation, 2)
        self.assertEqual(tracker.current_id, "goal-b")
        self.assertEqual(tracker.live_since, 2.0)
        self.assertFalse(tracker.status_confirmed)
        self.assertEqual(
            tracker.observe_status({"goal-a"}, set()), "missing"
        )
        self.assertEqual(tracker.current_id, "goal-b")
        self.assertEqual(tracker.generation, 2)
        self.assertEqual(
            tracker.observe_status({"goal-a", "goal-b"}, set()),
            "confirmed",
        )
        self.assertTrue(tracker.status_confirmed)
        self.assertEqual(
            tracker.observe_status(set(), {"goal-a", "goal-b"}),
            "terminal",
        )
        self.assertFalse(tracker.has_current_goal)
        self.assertEqual(tracker.generation, 3)

    def test_goal_without_local_plan_times_out_despite_post_goal_commands(self):
        self.assertIsNone(
            goal_handshake_fault(
                has_live_goal=True,
                goal_age=0.75,
                handshake_timeout=0.75,
                command_matches_goal=True,
                local_plan_matches_goal=False,
                local_plan_poses=0,
            )
        )
        self.assertEqual(
            goal_handshake_fault(
                has_live_goal=True,
                goal_age=0.751,
                handshake_timeout=0.75,
                command_matches_goal=True,
                local_plan_matches_goal=False,
                local_plan_poses=0,
            ),
            "goal_handshake_timeout",
        )

    def test_new_command_generation_invalidates_an_old_output_snapshot(self):
        self.assertTrue(snapshot_is_current(4, 10, 7, 4, 10, 7))
        self.assertFalse(snapshot_is_current(4, 10, 7, 4, 11, 7))
        self.assertFalse(snapshot_is_current(4, 10, 7, 5, 10, 7))

    def test_new_safety_input_invalidates_an_old_output_snapshot(self):
        self.assertFalse(snapshot_is_current(4, 10, 7, 4, 10, 8))

    def test_velocity_wiring_and_caller_identity_are_fixed(self):
        self.assertEqual(NAV_INPUT_TOPIC, "/nav_cmd_vel")
        self.assertEqual(DRIVER_OUTPUT_TOPIC, "/wheeltec_driver/cmd_vel")
        self.assertEqual(EXPECTED_NAV_CALLER_ID, "/move_base")
        self.assertEqual(EXPECTED_NODE_NAME, "/wheeltec_safety")
        self.assertTrue(command_caller_is_expected({"callerid": "/move_base"}))
        self.assertFalse(command_caller_is_expected({"callerid": "/teleop"}))
        self.assertFalse(command_caller_is_expected({}))
        self.assertFalse(command_caller_is_expected(None))

    def test_vertical_span_grid_accepts_ramp_and_detects_chair_leg(self):
        ramp = []
        slope = math.tan(math.radians(20.0))
        for cell in range(8):
            start = 0.30 + cell * 0.05
            ramp.extend(
                (
                    (start + 0.006, 0.01, -0.15 + slope * (start + 0.006)),
                    (start + 0.042, 0.01, -0.15 + slope * (start + 0.042)),
                )
            )
        args = dict(
            cell_size=0.05,
            min_points=2,
            min_vertical_span=0.08,
            min_z=-0.40,
            max_z=1.50,
            body_front=0.25,
            body_rear=0.25,
            body_half_width=0.20,
        )
        self.assertEqual(vertical_span_obstacle_cells(ramp, **args), ())

        chair_leg = (
            (0.356, 0.011, -0.15),
            (0.358, 0.012, -0.04),
            (0.357, 0.010, 0.10),
        )
        cells = vertical_span_obstacle_cells(chair_leg, **args)
        self.assertEqual(len(cells), 1)
        self.assertGreaterEqual(cells[0][2], 0.24)

    def test_vertical_span_grid_ignores_robot_body_returns(self):
        cells = vertical_span_obstacle_cells(
            ((0.10, 0.0, -0.15), (0.10, 0.0, 0.20)),
            cell_size=0.05,
            min_points=2,
            min_vertical_span=0.08,
            min_z=-0.40,
            max_z=1.50,
            body_front=0.25,
            body_rear=0.25,
            body_half_width=0.20,
        )
        self.assertEqual(cells, ())


if __name__ == "__main__":
    unittest.main()
