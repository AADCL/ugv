#!/usr/bin/env python3

"""Fail-closed velocity gate for WheelTech autonomous motion."""

import math
import struct
import threading
import time
import numpy as np

import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import tf2_ros
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, TriggerResponse
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
    motion_requested,
    snapshot_is_current,
    vertical_span_obstacle_cells,
)


def finite(*values):
    return all(math.isfinite(value) for value in values)


def safety_bound(name, value, default, lower, upper):
    if not math.isfinite(value):
        rospy.logerr("Non-finite safety parameter %s; using %.3f", name, default)
        return default
    bounded = min(upper, max(lower, value))
    if bounded != value:
        rospy.logwarn(
            "Safety parameter %s=%.3f limited to [%.3f, %.3f]",
            name,
            value,
            lower,
            upper,
        )
    return bounded


def bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value == 1
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off", ""):
            return False
    rospy.logwarn("Invalid boolean parameter %s=%r; using %s", name, value, default)
    return default


def copy_twist(message):
    output = Twist()
    output.linear.x = message.linear.x
    output.linear.y = message.linear.y
    output.linear.z = message.linear.z
    output.angular.x = message.angular.x
    output.angular.y = message.angular.y
    output.angular.z = message.angular.z
    return output


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class CmdVelSafetyGate:
    @property
    def armed(self):
        return self.interlock.armed

    @armed.setter
    def armed(self, value):
        self.interlock.armed = value

    @property
    def latched_stop(self):
        return self.interlock.latched

    @latched_stop.setter
    def latched_stop(self, value):
        self.interlock.latched = value

    @property
    def latched_reason(self):
        return self.interlock.reason if self.interlock.latched else ""

    @latched_reason.setter
    def latched_reason(self, value):
        self.interlock.reason = value

    def __init__(self):
        if rospy.get_name() != EXPECTED_NODE_NAME:
            raise RuntimeError(
                "safety gate must run exactly as " + EXPECTED_NODE_NAME
            )
        self.lock = threading.RLock()
        self.output_lock = threading.RLock()
        self.interlock = SafetyInterlock()

        # These absolute names are a security boundary, not launch options.
        self.input_topic = NAV_INPUT_TOPIC
        self.output_topic = DRIVER_OUTPUT_TOPIC
        for topic in (self.input_topic, self.output_topic):
            if rospy.remap_name(topic) != topic:
                raise RuntimeError("safety velocity topic remap forbidden: " + topic)
        self.obstacle_topic = rospy.get_param(
            "~obstacle_topic", "/terrain/obstacle_points"
        )
        self.raw_cloud_topic = rospy.get_param(
            "~raw_cloud_topic", "/cloud_registered_terrain"
        )
        self.costmap_topic = rospy.get_param(
            "~costmap_topic", "/move_base/local_costmap/costmap"
        )
        self.costmap_updates_topic = rospy.get_param(
            "~costmap_updates_topic", "/move_base/local_costmap/costmap_updates"
        )
        self.diagnostics_topic = rospy.get_param(
            "~diagnostics_topic", "/wheeltec_safety/status"
        )
        self.local_plan_topic = rospy.get_param(
            "~local_plan_topic", "/move_base/TebLocalPlannerROS/local_plan"
        )
        self.move_base_status_topic = rospy.get_param(
            "~move_base_status_topic", "/move_base/status"
        )
        self.move_base_goal_topic = "/move_base/goal"
        self.move_base_cancel_topic = rospy.get_param(
            "~move_base_cancel_topic", "/move_base/cancel"
        )
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        self.output_rate = max(10.0, float(rospy.get_param("~output_rate", 20.0)))
        self.command_timeout = max(
            0.05, float(rospy.get_param("~command_timeout", 0.20))
        )
        self.cloud_timeout = max(
            0.10, float(rospy.get_param("~cloud_timeout", 0.60))
        )
        self.raw_cloud_timeout = max(
            0.10, float(rospy.get_param("~raw_cloud_timeout", 0.60))
        )
        self.raw_cloud_min_points = max(
            1, int(rospy.get_param("~raw_cloud_min_points", 10))
        )
        self.raw_xy_cell_size = float(
            rospy.get_param("~raw_xy_cell_size", 0.05)
        )
        self.raw_cell_min_points = int(
            rospy.get_param("~raw_cell_min_points", 2)
        )
        self.raw_min_vertical_span = float(
            rospy.get_param("~raw_min_vertical_span", 0.08)
        )
        self.costmap_timeout = max(
            0.10, float(rospy.get_param("~costmap_timeout", 0.75))
        )
        self.tf_timeout = max(0.05, float(rospy.get_param("~tf_timeout", 0.50)))
        self.tf_lookup_timeout = max(
            0.0, float(rospy.get_param("~tf_lookup_timeout", 0.03))
        )
        self.require_local_plan = bool_param("~require_local_plan", True)
        self.allow_stationary_command_wait = bool_param(
            "~allow_stationary_command_wait", False)
        self.local_plan_timeout = max(
            0.05, float(rospy.get_param("~local_plan_timeout", 0.35))
        )
        self.goal_handshake_timeout = max(
            0.10, float(rospy.get_param("~goal_handshake_timeout", 12.0))
        )
        self.move_base_status_timeout = max(
            0.10, float(rospy.get_param("~move_base_status_timeout", 1.0))
        )
        self.locked_cancel_period = max(
            0.20, float(rospy.get_param("~locked_cancel_period", 1.0))
        )

        self.front = max(0.05, float(rospy.get_param("~footprint_front", 0.25)))
        self.rear = max(0.05, float(rospy.get_param("~footprint_rear", 0.25)))
        self.half_width = max(
            0.05, float(rospy.get_param("~footprint_half_width", 0.20))
        )
        self.footprint_margin = max(
            0.0, float(rospy.get_param("~footprint_margin", 0.02))
        )
        self.min_forward_clearance = max(
            0.0, float(rospy.get_param("~min_forward_clearance", 0.18))
        )
        self.min_reverse_clearance = max(
            0.0, float(rospy.get_param("~min_reverse_clearance", 0.18))
        )
        self.reaction_time = max(
            0.0, float(rospy.get_param("~reaction_time", 0.20))
        )
        self.linear_deceleration = max(
            0.05, float(rospy.get_param("~linear_deceleration", 0.20))
        )
        self.angular_deceleration = max(
            0.05, float(rospy.get_param("~angular_deceleration", 0.40))
        )
        self.max_prediction_horizon = max(
            0.20, float(rospy.get_param("~max_prediction_horizon", 1.50))
        )
        self.prediction_samples = max(
            3, int(rospy.get_param("~prediction_samples", 12))
        )

        self.allow_reverse = bool_param("~allow_reverse", False)
        self.linear_deadband = max(
            0.0, float(rospy.get_param("~linear_deadband", 0.005))
        )
        self.angular_deadband = max(
            0.0, float(rospy.get_param("~angular_deadband", 0.01))
        )
        self.max_linear_speed = max(
            0.01, float(rospy.get_param("~max_linear_speed", 0.20))
        )
        self.max_angular_speed = max(
            0.01, float(rospy.get_param("~max_angular_speed", 0.40))
        )
        self.costmap_lethal_threshold = max(
            1, min(100, int(rospy.get_param("~costmap_lethal_threshold", 100)))
        )
        self.unknown_is_unsafe = bool_param(
            "~treat_costmap_unknown_as_unsafe", True
        )
        self.near_field_crop_radius = float(
            rospy.get_param("~near_field_crop_radius", 1.00)
        )
        self.raw_vertical_min_z = float(
            rospy.get_param("~raw_vertical_min_z", -0.40)
        )
        self.raw_vertical_max_z = float(
            rospy.get_param("~raw_vertical_max_z", 1.50)
        )
        self.output_rate = safety_bound(
            "~output_rate", self.output_rate, 20.0, 20.0, 50.0
        )
        self.command_timeout = safety_bound(
            "~command_timeout", self.command_timeout, 0.20, 0.05, 0.25
        )
        self.cloud_timeout = safety_bound(
            "~cloud_timeout", self.cloud_timeout, 0.60, 0.10, 1.00
        )
        self.raw_cloud_timeout = safety_bound(
            "~raw_cloud_timeout", self.raw_cloud_timeout, 0.60, 0.10, 1.00
        )
        self.costmap_timeout = safety_bound(
            "~costmap_timeout", self.costmap_timeout, 0.75, 0.10, 1.00
        )
        self.tf_timeout = safety_bound(
            "~tf_timeout", self.tf_timeout, 0.50, 0.05, 1.00
        )
        self.tf_lookup_timeout = safety_bound(
            "~tf_lookup_timeout", self.tf_lookup_timeout, 0.03, 0.00, 0.10
        )
        self.local_plan_timeout = safety_bound(
            "~local_plan_timeout", self.local_plan_timeout, 0.35, 0.05, 0.50
        )
        self.goal_handshake_timeout = safety_bound(
            "~goal_handshake_timeout",
            self.goal_handshake_timeout,
            12.0,
            0.10,
            12.0,
        )
        self.move_base_status_timeout = safety_bound(
            "~move_base_status_timeout",
            self.move_base_status_timeout,
            1.00,
            0.10,
            2.00,
        )
        self.locked_cancel_period = safety_bound(
            "~locked_cancel_period", self.locked_cancel_period, 1.00, 0.20, 2.00
        )
        self.front = safety_bound("~footprint_front", self.front, 0.25, 0.25, 0.50)
        self.rear = safety_bound("~footprint_rear", self.rear, 0.25, 0.25, 0.50)
        self.half_width = safety_bound(
            "~footprint_half_width", self.half_width, 0.20, 0.20, 0.40
        )
        self.footprint_margin = safety_bound(
            "~footprint_margin", self.footprint_margin, 0.02, 0.00, 0.10
        )
        self.min_forward_clearance = safety_bound(
            "~min_forward_clearance",
            self.min_forward_clearance,
            0.18,
            0.15,
            1.00,
        )
        self.min_reverse_clearance = safety_bound(
            "~min_reverse_clearance",
            self.min_reverse_clearance,
            0.18,
            0.15,
            1.00,
        )
        self.reaction_time = safety_bound(
            "~reaction_time", self.reaction_time, 0.20, 0.10, 0.50
        )
        self.linear_deceleration = safety_bound(
            "~linear_deceleration", self.linear_deceleration, 0.20, 0.05, 0.20
        )
        self.angular_deceleration = safety_bound(
            "~angular_deceleration", self.angular_deceleration, 0.40, 0.05, 0.40
        )
        self.max_prediction_horizon = safety_bound(
            "~max_prediction_horizon",
            self.max_prediction_horizon,
            1.50,
            0.50,
            2.00,
        )
        self.prediction_samples = min(64, max(6, self.prediction_samples))
        self.linear_deadband = safety_bound(
            "~linear_deadband", self.linear_deadband, 0.005, 0.0, 0.02
        )
        self.angular_deadband = safety_bound(
            "~angular_deadband", self.angular_deadband, 0.01, 0.0, 0.05
        )
        self.max_linear_speed = safety_bound(
            "~max_linear_speed", self.max_linear_speed, 0.20, 0.0, 0.20
        )
        self.max_angular_speed = safety_bound(
            "~max_angular_speed", self.max_angular_speed, 0.40, 0.0, 0.40
        )
        self.raw_xy_cell_size = safety_bound(
            "~raw_xy_cell_size", self.raw_xy_cell_size, 0.05, 0.03, 0.05
        )
        self.raw_cell_min_points = int(
            safety_bound(
                "~raw_cell_min_points",
                float(self.raw_cell_min_points),
                2.0,
                2.0,
                2.0,
            )
        )
        self.raw_min_vertical_span = safety_bound(
            "~raw_min_vertical_span",
            self.raw_min_vertical_span,
            0.08,
            0.04,
            0.08,
        )
        self.raw_vertical_min_z = safety_bound(
            "~raw_vertical_min_z",
            self.raw_vertical_min_z,
            -0.40,
            -0.75,
            -0.15,
        )
        self.raw_vertical_max_z = safety_bound(
            "~raw_vertical_max_z",
            self.raw_vertical_max_z,
            1.50,
            0.30,
            2.00,
        )
        self.stop_region = DirectionalStopRegion(
            front=self.front,
            rear=self.rear,
            half_width=self.half_width,
            footprint_margin=self.footprint_margin,
            min_forward_clearance=self.min_forward_clearance,
            min_reverse_clearance=self.min_reverse_clearance,
            reaction_time=self.reaction_time,
            linear_deceleration=self.linear_deceleration,
            angular_deceleration=self.angular_deceleration,
            max_prediction_horizon=self.max_prediction_horizon,
            prediction_samples=self.prediction_samples,
            linear_deadband=self.linear_deadband,
            angular_deadband=self.angular_deadband,
        )
        footprint_radius = math.hypot(
            max(self.front, self.rear) + self.footprint_margin,
            self.half_width + self.footprint_margin,
        )
        required_crop_radius = (
            footprint_radius
            + self.max_linear_speed * self.max_prediction_horizon
            + max(self.min_forward_clearance, self.min_reverse_clearance)
        )
        self.near_field_crop_radius = safety_bound(
            "~near_field_crop_radius",
            self.near_field_crop_radius,
            1.00,
            required_crop_radius,
            4.00,
        )

        self.command = Twist()
        self.last_command_receive = None
        self.command_valid = True
        self.command_generation = 0
        self.last_command_caller_id = ""
        self.command_goal_generation = None
        self.cloud_points = ()
        self.cloud_frame = ""
        self.cloud_stamp = rospy.Time(0)
        self.last_cloud_receive = None
        self.raw_obstacle_cells = ()
        self.raw_cloud_point_count = 0
        self.raw_cloud_frame = ""
        self.raw_cloud_stamp = rospy.Time(0)
        self.last_raw_cloud_receive = None
        self.costmap = None
        self.last_costmap_receive = None
        self.local_plan_poses = 0
        self.last_local_plan_receive = None
        self.move_base_has_live_goal = False
        self.status_live_goal_ids = frozenset()
        self.last_move_base_status_receive = None
        self.live_goal_since = None
        self.goal_tracker = MoveBaseGoalTracker()
        self.goal_generation = self.goal_tracker.generation
        self.goal_handshake_generation = None
        self.local_plan_goal_generation = None
        self.state_generation = 0
        self.safety_input_generation = 0

        # Startup is deliberately locked. No parameter may bypass this state.
        self.trip_command = Twist()
        self.last_health_ok = False
        self.last_health_reason = "waiting_for_inputs"
        self.last_health_check = 0.0
        self.last_collision_clear = True
        self.last_state = "LOCKED"
        self.last_reason = "startup_lock"
        self.last_cloud_age = math.inf
        self.last_raw_cloud_age = math.inf
        self.last_costmap_age = math.inf
        self.last_command_age = math.inf
        self.last_tf_age = math.inf
        self.last_local_plan_age = math.inf
        self.last_move_base_status_age = math.inf
        self.last_cloud_clearance = math.inf
        self.last_raw_cloud_clearance = math.inf
        self.last_costmap_clearance = math.inf
        self.last_output = Twist()
        self.last_cancel_time = 0.0
        self.shutdown_started = False
        self.paused = False
        self.timer_stop = threading.Event()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.output_publisher = rospy.Publisher(
            self.output_topic, Twist, queue_size=1
        )
        self.diagnostic_publisher = rospy.Publisher(
            self.diagnostics_topic, DiagnosticArray, queue_size=1
        )
        self.cancel_publisher = rospy.Publisher(
            self.move_base_cancel_topic, GoalID, queue_size=1
        )
        self.command_subscriber = rospy.Subscriber(
            self.input_topic,
            Twist,
            self.command_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.cloud_subscriber = rospy.Subscriber(
            self.obstacle_topic, PointCloud2, self.cloud_callback, queue_size=1
        )
        self.raw_cloud_subscriber = rospy.Subscriber(
            self.raw_cloud_topic, PointCloud2, self.raw_cloud_callback, queue_size=1
        )
        self.costmap_subscriber = rospy.Subscriber(
            self.costmap_topic, OccupancyGrid, self.costmap_callback, queue_size=1
        )
        self.costmap_update_subscriber = rospy.Subscriber(
            self.costmap_updates_topic,
            OccupancyGridUpdate,
            self.costmap_update_callback,
            queue_size=10,
        )
        self.local_plan_subscriber = rospy.Subscriber(
            self.local_plan_topic, Path, self.local_plan_callback, queue_size=1
        )
        self.move_base_status_subscriber = rospy.Subscriber(
            self.move_base_status_topic,
            GoalStatusArray,
            self.move_base_status_callback,
            queue_size=1,
        )
        self.move_base_goal_subscriber = rospy.Subscriber(
            self.move_base_goal_topic,
            MoveBaseActionGoal,
            self.move_base_goal_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

        self.arm_service = rospy.Service("~arm", Trigger, self.arm_callback)
        self.stop_service = rospy.Service("~stop", Trigger, self.stop_callback)
        self.reset_service = rospy.Service("~reset", Trigger, self.reset_callback)
        self.pause_service = rospy.Service("~pause", Trigger, self.pause_callback)
        self.resume_service = rospy.Service("~resume", Trigger, self.resume_callback)
        rospy.on_shutdown(self.shutdown)
        self.timer_thread = threading.Thread(
            target=self.timer_loop, name="wheeltec_safety_wall_timer"
        )
        self.timer_thread.daemon = True
        self.timer_thread.start()

        rospy.logwarn(
            "WheelTech safety gate started LOCKED: %s -> %s; call ~arm only "
            "after diagnostics report healthy inputs",
            self.input_topic,
            self.output_topic,
        )

    def command_callback(self, message):
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        values_valid = finite(*values)
        connection_header = getattr(message, "_connection_header", None)
        caller_valid = command_caller_is_expected(connection_header)
        caller_id = (
            connection_header.get("callerid", "")
            if isinstance(connection_header, dict)
            else ""
        )
        valid = values_valid and caller_valid
        cancel_needed = False
        latch_reason = None
        receive_time = time.monotonic()
        zero_requested = valid and not motion_requested(
            message.linear.x,
            message.angular.z,
            self.linear_deadband,
            self.angular_deadband,
        )
        with self.output_lock:
            with self.lock:
                handshake_expired = self.goal_handshake_expired_locked(
                    receive_time
                )
                new_command = copy_twist(message) if valid else Twist()
                if new_command != self.command or valid != self.command_valid:
                    self.command_generation += 1
                self.command = new_command
                self.command_valid = valid
                self.last_command_caller_id = caller_id
                self.last_command_receive = receive_time
                # Identical command heartbeats refresh age without invalidating
                # the collision calculation. A different twist requires a recheck.
                self.command_goal_generation = (
                    self.goal_generation
                    if self.armed and self.goal_tracker.has_current_goal
                    else None
                )
                self.update_goal_handshake_locked(receive_time)
                if not valid or zero_requested:
                    self.state_generation += 1
                if not caller_valid:
                    latch_reason = "unexpected_command_publisher"
                    cancel_needed = self.latch_locked(latch_reason, Twist())
                elif not values_valid and self.armed:
                    latch_reason = "invalid_command"
                    cancel_needed = self.latch_locked(latch_reason, Twist())
                elif handshake_expired and self.armed:
                    latch_reason = "goal_handshake_timeout"
                    cancel_needed = self.latch_locked(
                        latch_reason, self.command
                    )
            if not valid or zero_requested or cancel_needed:
                self.output_publisher.publish(Twist())
            if cancel_needed:
                self.cancel_all_goals()
                rospy.logerr("WheelTech safety stop latched: %s", latch_reason)

    def cloud_callback(self, message):
        decoded = self.decode_cropped_cloud(message, "classified obstacle")
        if decoded is None:
            return
        points, frame, stamp = decoded
        with self.output_lock:
            with self.lock:
                if points != self.cloud_points or frame != self.cloud_frame:
                    self.safety_input_generation += 1
                self.cloud_points = points
                self.cloud_frame = frame
                self.cloud_stamp = stamp
                self.last_cloud_receive = time.monotonic()

    def raw_cloud_callback(self, message):
        decoded = self.decode_cropped_cloud(
            message,
            "raw near-field",
            min_z=self.raw_vertical_min_z,
            max_z=self.raw_vertical_max_z,
        )
        if decoded is None:
            return
        points, frame, stamp = decoded
        obstacle_cells = vertical_span_obstacle_cells(
            points=points,
            cell_size=self.raw_xy_cell_size,
            min_points=self.raw_cell_min_points,
            min_vertical_span=self.raw_min_vertical_span,
            min_z=self.raw_vertical_min_z,
            max_z=self.raw_vertical_max_z,
            body_front=self.front,
            body_rear=self.rear,
            body_half_width=self.half_width,
        )
        with self.output_lock:
            with self.lock:
                if (obstacle_cells != self.raw_obstacle_cells
                        or frame != self.raw_cloud_frame
                        or (len(points) < self.raw_cloud_min_points)
                        != (self.raw_cloud_point_count < self.raw_cloud_min_points)):
                    self.safety_input_generation += 1
                self.raw_obstacle_cells = obstacle_cells
                self.raw_cloud_point_count = len(points)
                self.raw_cloud_frame = frame
                self.raw_cloud_stamp = stamp
                self.last_raw_cloud_receive = time.monotonic()

    def decode_cropped_cloud(
        self, message, label, min_z=-math.inf, max_z=math.inf
    ):
        if not message.header.frame_id:
            rospy.logerr_throttle(2.0, "%s cloud has no frame", label)
            return None
        try:
            transform = None
            if message.header.frame_id != self.base_frame:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    message.header.frame_id,
                    message.header.stamp
                    if message.header.stamp.to_sec() > 0.0
                    else rospy.Time(0),
                    rospy.Duration(self.tf_lookup_timeout),
                )
            points = tuple(
                self.cropped_transformed_points(
                    point_cloud2.read_points(
                        message,
                        field_names=("x", "y", "z"),
                        skip_nans=True,
                    ),
                    transform,
                    min_z,
                    max_z,
                )
            )
        except tf2_ros.TransformException as error:
            rospy.logwarn_throttle(2.0, "%s cloud TF unavailable: %s", label, error)
            return None
        except (KeyError, struct.error, ValueError) as error:
            rospy.logerr_throttle(2.0, "Invalid %s cloud: %s", label, error)
            return None
        return points, self.base_frame, message.header.stamp

    def cropped_transformed_points(self, points, transform, min_z, max_z):
        radius = self.near_field_crop_radius
        if transform is None:
            for source_x, source_y, source_z in points:
                x = float(source_x)
                y = float(source_y)
                z = float(source_z)
                if not finite(x, y, z):
                    continue
                if abs(x) <= radius and abs(y) <= radius and min_z <= z <= max_z:
                    yield x, y, z
            return

        translation = transform.transform.translation
        q = transform.transform.rotation
        tx, ty, tz = translation.x, translation.y, translation.z
        r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        r01 = 2.0 * (q.x * q.y - q.z * q.w)
        r02 = 2.0 * (q.x * q.z + q.y * q.w)
        r10 = 2.0 * (q.x * q.y + q.z * q.w)
        r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z)
        r12 = 2.0 * (q.y * q.z - q.x * q.w)
        r20 = 2.0 * (q.x * q.z - q.y * q.w)
        r21 = 2.0 * (q.y * q.z + q.x * q.w)
        r22 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        for source_x, source_y, source_z in points:
            source_x = float(source_x)
            source_y = float(source_y)
            source_z = float(source_z)
            if not finite(source_x, source_y, source_z):
                continue
            x = tx + r00 * source_x + r01 * source_y + r02 * source_z
            y = ty + r10 * source_x + r11 * source_y + r12 * source_z
            z = tz + r20 * source_x + r21 * source_y + r22 * source_z
            # Transform and discard immediately; never retain the full cloud.
            if abs(x) <= radius and abs(y) <= radius and min_z <= z <= max_z:
                yield x, y, z

    def local_plan_callback(self, message):
        empty_plan = len(message.poses) == 0
        now = time.monotonic()
        cancel_needed = False
        with self.output_lock:
            with self.lock:
                handshake_expired = self.goal_handshake_expired_locked(now)
                if self.armed and self.goal_tracker.has_current_goal:
                    self.local_plan_poses = len(message.poses)
                    self.last_local_plan_receive = now
                    self.local_plan_goal_generation = self.goal_generation
                    self.update_goal_handshake_locked(now)
                    if empty_plan:
                        self.state_generation += 1
                    if handshake_expired:
                        cancel_needed = self.latch_locked(
                            "goal_handshake_timeout", self.command
                        )
            if empty_plan or cancel_needed:
                self.output_publisher.publish(Twist())
            if cancel_needed:
                self.cancel_all_goals()
                rospy.logerr(
                    "WheelTech safety stop latched: goal_handshake_timeout"
                )

    def move_base_goal_callback(self, message):
        goal_key = self.goal_id_key(message.goal_id)
        changed = False
        cancel_needed = False
        now = time.monotonic()
        with self.output_lock:
            with self.lock:
                if goal_key is None:
                    cancel_needed = self.latch_locked(
                        "invalid_move_base_goal_id", Twist()
                    )
                elif self.goal_tracker.accept_goal(goal_key, now):
                    changed = True
                    self.goal_generation = self.goal_tracker.generation
                    self.live_goal_since = self.goal_tracker.live_since
                    self.move_base_has_live_goal = True
                    self.local_plan_poses = 0
                    self.last_local_plan_receive = None
                    self.local_plan_goal_generation = None
                    self.goal_handshake_generation = None
                    self.state_generation += 1
            if changed or cancel_needed:
                self.output_publisher.publish(Twist())
            if cancel_needed:
                self.cancel_all_goals()
                rospy.logerr(
                    "WheelTech safety stop latched: invalid_move_base_goal_id"
                )

    def move_base_status_callback(self, message):
        live_states = (
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
            GoalStatus.PREEMPTING,
            GoalStatus.RECALLING,
        )
        now = time.monotonic()
        live_goal_ids = frozenset(
            self.goal_status_key(status)
            for status in message.status_list
            if status.status in live_states
        )
        terminal_goal_ids = frozenset(
            self.goal_status_key(status)
            for status in message.status_list
            if status.status not in live_states
        )
        transitioned = False
        cancel_needed = False
        with self.output_lock:
            with self.lock:
                status_live_changed = (
                    live_goal_ids != self.status_live_goal_ids
                )
                self.status_live_goal_ids = live_goal_ids
                tracker_result = self.goal_tracker.observe_status(
                    live_goal_ids, terminal_goal_ids
                )
                if tracker_result == "terminal":
                    transitioned = True
                    self.goal_generation = self.goal_tracker.generation
                    self.live_goal_since = self.goal_tracker.live_since
                    self.local_plan_poses = 0
                    self.last_local_plan_receive = None
                    self.local_plan_goal_generation = None
                    self.goal_handshake_generation = None
                elif tracker_result == "confirmed":
                    transitioned = True
                    self.update_goal_handshake_locked(now)
                self.move_base_has_live_goal = (
                    self.goal_tracker.has_current_goal
                    or bool(self.status_live_goal_ids)
                )
                if status_live_changed or transitioned:
                    self.state_generation += 1
                self.last_move_base_status_receive = now
                if self.goal_handshake_expired_locked(now) and self.armed:
                    cancel_needed = self.latch_locked(
                        "goal_handshake_timeout", self.command
                    )
            if transitioned or cancel_needed:
                self.output_publisher.publish(Twist())
            if cancel_needed:
                self.cancel_all_goals()
                rospy.logerr(
                    "WheelTech safety stop latched: goal_handshake_timeout"
                )

    @staticmethod
    def goal_status_key(status):
        return CmdVelSafetyGate.goal_id_key(status.goal_id)

    @staticmethod
    def goal_id_key(goal_id):
        stamp = goal_id.stamp
        if goal_id.id:
            return (goal_id.id, stamp.secs, stamp.nsecs)
        if stamp.secs or stamp.nsecs:
            return ("<anonymous>", stamp.secs, stamp.nsecs)
        return None

    def costmap_callback(self, message):
        expected = message.info.width * message.info.height
        if len(message.data) != expected or expected == 0:
            rospy.logerr_throttle(2.0, "Invalid local costmap snapshot")
            return
        snapshot = OccupancyGrid()
        snapshot.header = message.header
        snapshot.info = message.info
        snapshot.data = list(message.data)
        with self.output_lock:
            with self.lock:
                if not self.same_costmap(self.costmap, snapshot):
                    self.safety_input_generation += 1
                self.costmap = snapshot
                self.last_costmap_receive = time.monotonic()

    def costmap_update_callback(self, update):
        expected = update.width * update.height
        if len(update.data) != expected:
            rospy.logerr_throttle(2.0, "Invalid local costmap update length")
            return
        with self.output_lock:
            with self.lock:
                if self.costmap is None:
                    return
                width = self.costmap.info.width
                height = self.costmap.info.height
                if (
                    update.x + update.width > width
                    or update.y + update.height > height
                    or update.width == 0
                    or update.height == 0
                ):
                    rospy.logerr_throttle(
                        2.0, "Out-of-bounds local costmap update"
                    )
                    return
                if (
                    update.header.frame_id
                    and self.costmap.header.frame_id
                    and update.header.frame_id != self.costmap.header.frame_id
                ):
                    rospy.logerr_throttle(
                        2.0, "Local costmap update frame mismatch"
                    )
                    return
                snapshot = OccupancyGrid()
                snapshot.header = update.header
                if not snapshot.header.frame_id:
                    snapshot.header.frame_id = self.costmap.header.frame_id
                snapshot.info = self.costmap.info
                snapshot.data = apply_grid_update(
                    self.costmap.data,
                    width,
                    height,
                    update.x,
                    update.y,
                    update.width,
                    update.height,
                    update.data,
                )
                if not self.same_costmap(self.costmap, snapshot):
                    self.safety_input_generation += 1
                self.costmap = snapshot
                self.last_costmap_receive = time.monotonic()

    @staticmethod
    def same_costmap(first, second):
        # Publication time/map_load_time do not change collision geometry.
        return (first is not None and second is not None
                and first.header.frame_id == second.header.frame_id
                and first.info.width == second.info.width
                and first.info.height == second.info.height
                and first.info.resolution == second.info.resolution
                and first.info.origin == second.info.origin
                and first.data == second.data)

    def arm_callback(self, _request):
        with self.output_lock:
            with self.lock:
                recent_check = (
                    time.monotonic() - self.last_health_check
                    <= max(0.20, 3.0 / self.output_rate)
                )
                if self.latched_stop:
                    return TriggerResponse(
                        success=False,
                        message="reset required after: " + self.latched_reason,
                    )
                if not recent_check or not self.last_health_ok:
                    return TriggerResponse(
                        success=False,
                        message="safety inputs not healthy: " + self.last_health_reason,
                    )
                status_age = self.age(
                    time.monotonic(), self.last_move_base_status_receive
                )
                if status_age > self.move_base_status_timeout:
                    return TriggerResponse(
                        success=False, message="move_base status is missing or stale"
                    )
                if self.move_base_has_live_goal:
                    return TriggerResponse(
                        success=False,
                        message="cancel the current move_base goal before arming",
                    )
                self.clear_motion_inputs_locked()
                self.paused = False
                self.armed = True
                self.state_generation += 1
                self.last_reason = "armed_waiting_for_new_goal"
            self.output_publisher.publish(Twist())
        return TriggerResponse(success=True, message="velocity gate armed")

    def stop_callback(self, _request):
        with self.output_lock:
            with self.lock:
                self.latch_locked("operator_stop", self.command)
            self.output_publisher.publish(Twist())
            self.cancel_all_goals()
        return TriggerResponse(success=True, message="stop latched; reset required")

    def pause_callback(self, _request):
        with self.output_lock:
            with self.lock:
                self.paused = True
                self.clear_motion_inputs_locked()
                self.state_generation += 1
                self.last_output = Twist()
            self.output_publisher.publish(Twist())
            self.cancel_all_goals()
        return TriggerResponse(success=True, message="paused; no motion until resume and a new goal")

    def resume_callback(self, _request):
        with self.output_lock:
            with self.lock:
                if (not self.armed or self.latched_stop or not self.last_health_ok
                        or time.monotonic() - self.last_health_check > 0.20
                        or self.move_base_has_live_goal):
                    return TriggerResponse(success=False, message="resume requires an armed healthy gate and no live goal")
                self.clear_motion_inputs_locked()
                self.paused = False
                self.state_generation += 1
            self.output_publisher.publish(Twist())
        return TriggerResponse(success=True, message="resumed; waiting for a new goal")

    def reset_callback(self, _request):
        with self.output_lock:
            with self.lock:
                recent_check = (
                    time.monotonic() - self.last_health_check
                    <= max(0.20, 3.0 / self.output_rate)
                )
                if not recent_check or not self.last_health_ok:
                    return TriggerResponse(
                        success=False,
                        message="cannot reset unhealthy gate: " + self.last_health_reason,
                    )
                if not self.last_collision_clear:
                    return TriggerResponse(
                        success=False, message="stop region is still occupied"
                    )
                self.armed = False
                self.latched_stop = False
                self.latched_reason = ""
                self.trip_command = Twist()
                self.clear_motion_inputs_locked()
                self.state_generation += 1
                self.last_reason = "reset_locked"
            self.output_publisher.publish(Twist())
        return TriggerResponse(
            success=True, message="fault reset; gate remains locked until arm"
        )

    def timer_loop(self):
        period = 1.0 / self.output_rate
        next_deadline = time.monotonic()
        while not rospy.is_shutdown() and not self.timer_stop.is_set():
            self.timer_callback()
            next_deadline += period
            now = time.monotonic()
            if next_deadline < now:
                next_deadline = now
            self.timer_stop.wait(max(0.0, next_deadline - now))

    def timer_callback(self):
        try:
            self.run_cycle()
        except Exception as error:  # The output path must fail closed.
            cancel_needed = False
            with self.output_lock:
                now = time.monotonic()
                with self.lock:
                    if self.armed:
                        cancel_needed = self.latch_locked(
                            "gate_exception", self.command
                        )
                    if (
                        not self.armed
                        and now - self.last_cancel_time >= self.locked_cancel_period
                    ):
                        cancel_needed = True
                    self.last_state = "EMERGENCY_STOP"
                    self.last_reason = "gate_exception"
                    self.last_output = Twist()
                self.output_publisher.publish(Twist())
                if cancel_needed:
                    self.cancel_all_goals()
                rospy.logerr_throttle(1.0, "Safety gate exception: %s", error)
                self.publish_diagnostics(DiagnosticStatus.ERROR)

    def run_cycle(self):
        started = time.monotonic()
        deadline = started + 0.8 / self.output_rate
        for attempt in range(3):
            self.last_recheck_count = attempt
            if self._run_cycle_attempt(deadline):
                self.last_cycle_ms = (time.monotonic() - started) * 1000.0
                return
            if time.monotonic() >= deadline:
                break
        # Only sustained contention/overrun reaches this stop. Never reuse an
        # unchecked command or smooth a protective zero across a real hazard.
        with self.output_lock:
            with self.lock:
                self.last_state = 'EMERGENCY_STOP' if self.latched_stop else 'WAITING_FOR_FRESH_CYCLE'
                self.last_reason = self.latched_reason if self.latched_stop else 'collision_check_budget_exhausted'
                self.last_output = Twist()
                self.last_health_ok = False
                self.last_health_reason = 'collision_check_budget_exhausted'
                self.last_collision_clear = False
                self.last_cycle_ms = (time.monotonic() - started) * 1000.0
            self.output_publisher.publish(Twist())
            self.publish_diagnostics(DiagnosticStatus.ERROR if self.latched_stop else DiagnosticStatus.WARN)

    def _run_cycle_attempt(self, deadline):
        now = time.monotonic()
        cancel_needed = False
        newly_latched_reason = None
        with self.lock:
            generation = self.state_generation
            command_generation = self.command_generation
            safety_input_generation = self.safety_input_generation
            command = copy_twist(self.command)
            command_valid = self.command_valid
            command_receive = self.last_command_receive
            command_goal_generation = self.command_goal_generation
            cloud_points = self.cloud_points
            cloud_frame = self.cloud_frame
            cloud_receive = self.last_cloud_receive
            raw_obstacle_cells = self.raw_obstacle_cells
            raw_cloud_point_count = self.raw_cloud_point_count
            raw_cloud_frame = self.raw_cloud_frame
            raw_cloud_receive = self.last_raw_cloud_receive
            costmap = self.copy_costmap_locked()
            costmap_receive = self.last_costmap_receive
            local_plan_poses = self.local_plan_poses
            local_plan_receive = self.last_local_plan_receive
            move_base_status_receive = self.last_move_base_status_receive
            move_base_has_live_goal = self.move_base_has_live_goal
            current_goal_active = self.goal_tracker.has_current_goal
            goal_status_confirmed = self.goal_tracker.status_confirmed
            live_goal_since = self.live_goal_since
            goal_generation = self.goal_generation
            goal_handshake_generation = self.goal_handshake_generation
            local_plan_goal_generation = self.local_plan_goal_generation
            latched = self.latched_stop
            trip_command = copy_twist(self.trip_command)

        command_age = self.age(now, command_receive)
        command_belongs_to_goal = (
            current_goal_active
            and command_goal_generation == goal_generation
        )
        if move_base_has_live_goal and not command_belongs_to_goal:
            command_age = math.inf
        cloud_age = self.age(now, cloud_receive)
        raw_cloud_age = self.age(now, raw_cloud_receive)
        costmap_age = self.age(now, costmap_receive)
        local_plan_age = self.age(now, local_plan_receive)
        move_base_status_age = self.age(now, move_base_status_receive)
        goal_age = self.age(now, live_goal_since)
        health_reason = "healthy"
        costmap_transform = None
        tf_age = math.inf

        if cloud_receive is None or cloud_age > self.cloud_timeout:
            health_reason = "obstacle_cloud_stale"
        elif not cloud_frame:
            health_reason = "obstacle_cloud_frame_missing"
        elif raw_cloud_receive is None or raw_cloud_age > self.raw_cloud_timeout:
            health_reason = "raw_cloud_stale"
        elif not raw_cloud_frame:
            health_reason = "raw_cloud_frame_missing"
        elif raw_cloud_point_count < self.raw_cloud_min_points:
            health_reason = "raw_cloud_too_sparse"
        elif costmap is None or costmap_receive is None:
            health_reason = "local_costmap_missing"
        elif costmap_age > self.costmap_timeout:
            health_reason = "local_costmap_stale"
        elif not costmap.header.frame_id:
            health_reason = "local_costmap_frame_missing"
        else:
            try:
                costmap_transform = self.tf_buffer.lookup_transform(
                    costmap.header.frame_id,
                    self.base_frame,
                    rospy.Time(0),
                    rospy.Duration(self.tf_lookup_timeout),
                )
                tf_age = self.transform_age(costmap_transform)
                if tf_age > self.tf_timeout:
                    health_reason = "robot_tf_stale"
            except tf2_ros.TransformException as error:
                health_reason = "tf_unavailable"
                rospy.logwarn_throttle(2.0, "Safety gate TF unavailable: %s", error)

        health_ok = health_reason == "healthy"
        probe = trip_command if latched else command
        cloud_blocked = False
        raw_cloud_blocked = False
        costmap_blocked = False
        cloud_clearance = math.inf
        raw_cloud_clearance = math.inf
        costmap_clearance = math.inf
        if health_ok:
            cloud_blocked, cloud_clearance = self.cloud_stop_check(
                cloud_points, probe
            )
            raw_cloud_blocked, raw_cloud_clearance = self.raw_cloud_stop_check(
                raw_obstacle_cells, probe
            )
            costmap_blocked, costmap_clearance = self.costmap_stop_check(
                costmap, costmap_transform, probe
            )

        collision_clear = (
            health_ok
            and not cloud_blocked
            and not raw_cloud_blocked
            and not costmap_blocked
        )
        with self.output_lock:
            with self.lock:
                current = snapshot_is_current(
                    generation, command_generation, safety_input_generation,
                    self.state_generation, self.command_generation,
                    self.safety_input_generation)
                # Real detected hazards veto immediately, including when another
                # input arrived while checking. The first latch reason persists.
                if self.armed and health_ok and not collision_clear:
                    fault = ('obstacle_in_cloud_stop_region'
                             if cloud_blocked else 'obstacle_in_raw_cloud_stop_region'
                             if raw_cloud_blocked else 'obstacle_in_costmap_stop_region')
                    if self.latch_locked(fault, probe):
                        cancel_needed = True
                        newly_latched_reason = fault
                if not current and not self.latched_stop:
                    return False
                checked_at = time.monotonic()
                # Freshness is evaluated at publication, not just check start.
                if not health_ok and (
                    self.last_cloud_receive != cloud_receive
                    or self.last_raw_cloud_receive != raw_cloud_receive
                    or self.last_costmap_receive != costmap_receive):
                    return False
                command_age = self.age(checked_at, self.last_command_receive)
                if move_base_has_live_goal and not command_belongs_to_goal:
                    command_age = math.inf
                local_plan_age = self.age(checked_at, self.last_local_plan_receive)
                move_base_status_age = self.age(checked_at, self.last_move_base_status_receive)
                goal_age = self.age(checked_at, live_goal_since)
                if health_ok:
                    for age, limit, stale_reason in (
                        (self.age(checked_at, self.last_cloud_receive), self.cloud_timeout, 'obstacle_cloud_stale'),
                        (self.age(checked_at, self.last_raw_cloud_receive), self.raw_cloud_timeout, 'raw_cloud_stale'),
                        (self.age(checked_at, self.last_costmap_receive), self.costmap_timeout, 'local_costmap_stale'),
                        (self.transform_age(costmap_transform), self.tf_timeout, 'robot_tf_stale'),
                    ):
                        if age > limit:
                            health_ok, health_reason = False, stale_reason
                            collision_clear = False
                            break
                if checked_at > deadline and not self.latched_stop:
                    return False
                self.last_health_ok = health_ok and current
                self.last_cycle_ms = (time.monotonic() - now) * 1000.0
                self.last_health_reason = health_reason
                self.last_health_check = now
                self.last_collision_clear = collision_clear and current
                self.last_command_age = command_age
                self.last_cloud_age = cloud_age
                self.last_raw_cloud_age = raw_cloud_age
                self.last_costmap_age = costmap_age
                self.last_tf_age = tf_age
                self.last_local_plan_age = local_plan_age
                self.last_move_base_status_age = move_base_status_age
                self.last_cloud_clearance = cloud_clearance
                self.last_raw_cloud_clearance = raw_cloud_clearance
                self.last_costmap_clearance = costmap_clearance

                transient_reason = None
                if self.latched_stop:
                    output = Twist()
                    state = "EMERGENCY_STOP"
                    reason = self.latched_reason
                    level = DiagnosticStatus.ERROR
                else:
                    if local_plan_goal_generation != goal_generation:
                        local_plan_age = math.inf
                        local_plan_poses = 0

                    handshake_complete = (
                        current_goal_active
                        and goal_status_confirmed
                        and goal_handshake_generation == goal_generation
                    )
                    handshake_pending = (
                        current_goal_active
                        and not handshake_complete
                    )
                    handshake_fault = goal_handshake_fault(
                        has_live_goal=current_goal_active,
                        goal_age=goal_age,
                        handshake_timeout=self.goal_handshake_timeout,
                        command_matches_goal=handshake_complete,
                        local_plan_matches_goal=handshake_complete,
                        local_plan_poses=1 if handshake_complete else 0,
                    )

                    if not health_ok:
                        fault_reason = health_reason
                    else:
                        fault_reason = motion_fault_reason(
                            linear_x=command.linear.x,
                            angular_z=command.angular.z,
                            command_valid=command_valid,
                            command_age=command_age,
                            command_timeout=self.command_timeout,
                            has_live_goal=move_base_has_live_goal,
                            status_age=move_base_status_age,
                            status_timeout=self.move_base_status_timeout,
                            allow_reverse=self.allow_reverse,
                            linear_deadband=self.linear_deadband,
                            angular_deadband=self.angular_deadband,
                            require_local_plan=self.require_local_plan,
                            local_plan_age=local_plan_age,
                            local_plan_timeout=self.local_plan_timeout,
                            local_plan_poses=local_plan_poses,
                            cloud_blocked=cloud_blocked or raw_cloud_blocked,
                            costmap_blocked=costmap_blocked,
                            allow_stationary_command_wait=self.allow_stationary_command_wait,
                        )

                    handshake_wait_faults = {
                        "command_stream_stale",
                        "local_plan_stale",
                        "local_plan_empty",
                    }
                    if (
                        self.armed
                        and not self.latched_stop
                        and health_ok
                        and handshake_pending
                    ):
                        if handshake_fault is not None:
                            fault_reason = handshake_fault
                        elif (
                            fault_reason is None
                            or fault_reason in handshake_wait_faults
                        ):
                            transient_reason = "goal_handshake_pending"
                            fault_reason = None

                    if (
                        self.armed
                        and not self.latched_stop
                        and fault_reason is not None
                    ):
                        if self.latch_locked(fault_reason, command):
                            cancel_needed = True
                            newly_latched_reason = fault_reason

                    if self.latched_stop:
                        output = Twist()
                        state = "EMERGENCY_STOP"
                        reason = self.latched_reason
                        level = DiagnosticStatus.ERROR
                    elif not self.armed:
                        output = Twist()
                        state = "LOCKED"
                        reason = (
                            "startup_or_operator_lock" if health_ok else health_reason
                        )
                        level = DiagnosticStatus.WARN
                    elif self.paused:
                        output = Twist()
                        state = "PAUSED"
                        reason = "exploration_waiting_for_data"
                        level = DiagnosticStatus.WARN
                    elif transient_reason is not None:
                        output = Twist()
                        state = "WAITING_FOR_PLANNER"
                        reason = transient_reason
                        level = DiagnosticStatus.WARN
                    elif command_age > self.command_timeout:
                        output = Twist()
                        state = "WAITING_FOR_COMMAND"
                        reason = "command_stale"
                        level = DiagnosticStatus.WARN
                    else:
                        output = self.sanitized_command(command)
                        state = "PASS"
                        reason = "healthy"
                        level = DiagnosticStatus.OK

                # A locked gate must not allow a queued goal to become live
                # immediately after a later arm.
                if (
                    not self.armed
                    and now - self.last_cancel_time >= self.locked_cancel_period
                ):
                    cancel_needed = True

                self.last_state = state
                self.last_reason = reason
                self.last_output = copy_twist(output)

            # output_lock linearizes this final generation check and publish
            # against command, goal, stop, reset, and shutdown callbacks.
            self.output_publisher.publish(output)
            if cancel_needed:
                self.cancel_all_goals()
            if newly_latched_reason is not None:
                rospy.logerr(
                    "WheelTech safety stop latched: %s", newly_latched_reason
                )
            self.publish_diagnostics(level)
        return True

    def copy_costmap_locked(self):
        if self.costmap is None:
            return None
        snapshot = OccupancyGrid()
        snapshot.header = self.costmap.header
        snapshot.info = self.costmap.info
        snapshot.data = list(self.costmap.data)
        return snapshot

    @staticmethod
    def age(now, received):
        return math.inf if received is None else max(0.0, now - received)

    @staticmethod
    def transform_age(transform):
        if transform.header.stamp.to_sec() <= 0.0:
            return 0.0
        age = (rospy.Time.now() - transform.header.stamp).to_sec()
        return math.inf if age < -0.10 else max(0.0, age)

    def sanitized_command(self, command):
        output = Twist()
        output.linear.x = max(
            -self.max_linear_speed, min(self.max_linear_speed, command.linear.x)
        )
        output.angular.z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, command.angular.z),
        )
        if abs(output.linear.x) < self.linear_deadband:
            output.linear.x = 0.0
        if not self.allow_reverse and -1e-4 <= output.linear.x < 0:
            output.linear.x = 0.0
        if abs(output.angular.z) < self.angular_deadband:
            output.angular.z = 0.0
        return output

    def prediction_poses(self, command):
        return self.stop_region.prediction_poses(
            command.linear.x, command.angular.z
        )

    def point_in_stop_region(self, x, y, command, poses=None):
        return self.stop_region.contains(
            x,
            y,
            command.linear.x,
            command.angular.z,
            poses=poses,
        )

    def cloud_stop_check(self, points, command):
        return self.stop_region.clearance(points, command.linear.x, command.angular.z)

    def raw_cloud_stop_check(self, obstacle_cells, command):
        return self.stop_region.clearance(obstacle_cells, command.linear.x, command.angular.z)

    def costmap_stop_check(self, costmap, transform, command):
        poses = self.prediction_poses(command)
        if not poses:
            return False, math.inf
        transform_yaw = yaw_from_quaternion(transform.transform.rotation)
        transform_cosine = math.cos(transform_yaw)
        transform_sine = math.sin(transform_yaw)
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        origin = costmap.info.origin
        origin_yaw = yaw_from_quaternion(origin.orientation)
        origin_cosine = math.cos(origin_yaw)
        origin_sine = math.sin(origin_yaw)
        resolution = costmap.info.resolution
        width = costmap.info.width
        values = np.asarray(costmap.data)
        unsafe = values >= self.costmap_lethal_threshold
        if self.unknown_is_unsafe:
            unsafe |= values < 0
        indices = np.flatnonzero(unsafe)
        map_x = (indices % width + 0.5) * resolution
        map_y = (indices // width + 0.5) * resolution
        dx = origin.position.x + origin_cosine*map_x - origin_sine*map_y - tx
        dy = origin.position.y + origin_sine*map_x + origin_cosine*map_y - ty
        points = np.column_stack((transform_cosine*dx + transform_sine*dy,
                                  -transform_sine*dx + transform_cosine*dy))
        return self.stop_region.clearance(points, command.linear.x, command.angular.z)

    def latch_locked(self, reason, command):
        if self.latched_stop:
            return False
        self.armed = False
        self.latched_stop = True
        self.latched_reason = reason
        self.trip_command = copy_twist(command)
        self.last_reason = reason
        self.state_generation += 1
        return True

    def clear_motion_inputs_locked(self):
        self.command = Twist()
        self.command_valid = True
        self.last_command_receive = None
        self.command_generation += 1
        self.command_goal_generation = None
        self.local_plan_poses = 0
        self.last_local_plan_receive = None
        self.local_plan_goal_generation = None
        self.goal_handshake_generation = None

    def update_goal_handshake_locked(self, now):
        if not self.goal_tracker.has_current_goal:
            return
        if not self.goal_tracker.status_confirmed:
            return
        if self.goal_handshake_generation == self.goal_generation:
            return
        if self.age(now, self.live_goal_since) > self.goal_handshake_timeout:
            return
        command_matches = (
            self.command_valid
            and self.command_goal_generation == self.goal_generation
            and self.age(now, self.last_command_receive) <= self.command_timeout
        )
        plan_matches = (
            self.local_plan_goal_generation == self.goal_generation
            and self.local_plan_poses > 0
            and self.age(now, self.last_local_plan_receive)
            <= self.local_plan_timeout
        )
        if command_matches and plan_matches:
            self.goal_handshake_generation = self.goal_generation

    def goal_handshake_expired_locked(self, now):
        complete = (
            self.goal_tracker.has_current_goal
            and self.goal_tracker.status_confirmed
            and self.goal_handshake_generation == self.goal_generation
        )
        goal_age = self.age(now, self.live_goal_since)
        return goal_handshake_fault(
            has_live_goal=self.goal_tracker.has_current_goal,
            goal_age=goal_age,
            handshake_timeout=self.goal_handshake_timeout,
            command_matches_goal=complete,
            local_plan_matches_goal=complete,
            local_plan_poses=1 if complete else 0,
        ) is not None

    def cancel_all_goals(self):
        # Empty id and zero stamp are the actionlib protocol for cancel-all.
        self.cancel_publisher.publish(GoalID())
        with self.lock:
            self.last_cancel_time = time.monotonic()

    def publish_diagnostics(self, level):
        with self.lock:
            values = {
                "state": self.last_state,
                "reason": self.last_reason,
                "armed": self.armed,
                "latched_stop": self.latched_stop,
                "latched_reason": self.latched_reason,
                "paused": self.paused,
                "command_age_s": self.last_command_age,
                "cloud_age_s": self.last_cloud_age,
                "raw_cloud_age_s": self.last_raw_cloud_age,
                "costmap_age_s": self.last_costmap_age,
                "tf_age_s": self.last_tf_age,
                "local_plan_age_s": self.last_local_plan_age,
                "local_plan_poses": self.local_plan_poses,
                "move_base_status_age_s": self.last_move_base_status_age,
                "move_base_has_live_goal": self.move_base_has_live_goal,
                "move_base_status_live_goal_count": len(
                    self.status_live_goal_ids
                ),
                "move_base_current_goal": self.goal_tracker.current_id,
                "move_base_goal_status_confirmed": (
                    self.goal_tracker.status_confirmed
                ),
                "goal_generation": self.goal_generation,
                "goal_handshake_generation": self.goal_handshake_generation,
                "command_generation": self.command_generation,
                "safety_input_generation": self.safety_input_generation,
                "check_duration_ms": getattr(self, "last_cycle_ms", 0.0),
                "same_cycle_rechecks": getattr(self, "last_recheck_count", 0),
                "raw_cloud_near_field_points": self.raw_cloud_point_count,
                "raw_vertical_obstacle_cells": len(self.raw_obstacle_cells),
                "cloud_clearance_m": self.last_cloud_clearance,
                "raw_cloud_clearance_m": self.last_raw_cloud_clearance,
                "costmap_clearance_m": self.last_costmap_clearance,
                "input_linear_x": self.command.linear.x,
                "input_angular_z": self.command.angular.z,
                "input_caller_id": self.last_command_caller_id,
                "output_linear_x": self.last_output.linear.x,
                "output_angular_z": self.last_output.angular.z,
            }
            state = self.last_state
            reason = self.last_reason
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        status = DiagnosticStatus()
        status.name = "wheeltec_safety"
        status.hardware_id = "wheeltec_four_wheel_diff"
        status.level = level
        status.message = state + ": " + reason
        status.values = [
            KeyValue(key=str(key), value=str(value)) for key, value in values.items()
        ]
        array.status.append(status)
        self.diagnostic_publisher.publish(array)

    def shutdown(self):
        with self.output_lock:
            with self.lock:
                if self.shutdown_started:
                    return
                self.shutdown_started = True
                self.timer_stop.set()
                self.armed = False
                self.latched_stop = True
                self.latched_reason = "shutdown"
                self.state_generation += 1
                self.last_state = "EMERGENCY_STOP"
                self.last_reason = "shutdown"
                self.last_output = Twist()
            stop = Twist()
            self.cancel_all_goals()
            for _ in range(10):
                try:
                    self.output_publisher.publish(stop)
                    time.sleep(0.02)
                except Exception:
                    break


def main():
    rospy.init_node("wheeltec_safety")
    CmdVelSafetyGate()
    rospy.spin()


if __name__ == "__main__":
    main()
