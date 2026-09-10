#!/usr/bin/env python3
"""Guard a shadow robot_localization EKF; never publish TF or motion commands."""
import copy
import math
import threading
import time


def diagonal(values):
    result = [0.0] * 36
    for i, value in enumerate(values):
        result[7 * i] = value
    return result


def valid_quaternion(q):
    values = (q.x, q.y, q.z, q.w)
    return (all(math.isfinite(v) for v in values)
            and abs(sum(v*v for v in values) - 1.0) < 0.01)


def rotation_distance(a, b):
    av, bv = (a.x, a.y, a.z, a.w), (b.x, b.y, b.z, b.w)
    dot = sum(x*y for x, y in zip(av, bv))
    norm = math.sqrt(sum(x*x for x in av) * sum(x*x for x in bv))
    return 2.0 * math.acos(min(1.0, abs(dot) / norm))


class StreamGate:
    def __init__(self, max_age, future):
        self.max_age, self.future = max_age, future
        self.stamp = None
        self.wall = None
        self.rejected = 0

    def accept(self, stamp, now, wall):
        valid = (math.isfinite(stamp) and stamp > 0
                 and -self.future <= now - stamp <= self.max_age
                 and (self.stamp is None or stamp > self.stamp))
        if not valid:
            self.rejected += 1
            return False
        self.stamp, self.wall = stamp, wall
        return True

    def fresh(self, now, wall, timeout):
        return (self.stamp is not None and wall - self.wall <= timeout
                and -self.future <= now - self.stamp <= timeout)


class FusionGuard:
    def __init__(self):
        import rospy
        from nav_msgs.msg import Odometry
        from diagnostic_msgs.msg import DiagnosticArray
        self.ros, self.Odometry = rospy, Odometry
        self.lock = threading.RLock()
        self.profile = rospy.get_param('~profile', 'baseline')
        self.world = rospy.get_param('~odom_frame', 'odom')
        self.body = rospy.get_param('~base_frame', 'base_link')
        self.age = float(rospy.get_param('~max_input_age', 0.30))
        future = float(rospy.get_param('~future_tolerance', 0.05))
        self.timeout = float(rospy.get_param('~dropout_timeout', 0.50))
        self.vmax = float(rospy.get_param('~max_linear_speed', 2.0))
        self.wmax = float(rospy.get_param('~max_angular_speed', 3.0))
        self.jump = float(rospy.get_param('~lio_jump_margin_m', 0.30))
        self.angle_jump = float(rospy.get_param('~lio_jump_margin_rad', 0.20))
        self.pose_var = rospy.get_param('~lio_pose_variances', [0.01]*6)
        self.initial_pose_var = rospy.get_param('~initial_lio_pose_variances', self.pose_var)
        self.speed_var = float(rospy.get_param('~wheel_speed_variance', 0.0025))
        self.turn_scale = float(rospy.get_param('~turn_scale_rad_s', 0.30))
        self.max_scale = float(rospy.get_param('~max_turn_variance_scale', 25.0))
        positive = [self.age, self.timeout, self.vmax, self.wmax, self.jump,
                    self.angle_jump, self.speed_var, self.turn_scale, self.max_scale]
        if (len(self.pose_var) != 6 or len(self.initial_pose_var) != 6 or not self.world or not self.body
                or not math.isfinite(future) or future < 0
                or not all(math.isfinite(v) and v > 0 for v in positive + self.pose_var + self.initial_pose_var)):
            raise ValueError('Invalid fusion guard configuration')
        self.lio = StreamGate(self.age, future)
        self.wheel = StreamGate(self.age, future)
        self.output = StreamGate(self.age, future)
        self.previous_pose = None
        self.first_lio = None
        self.started = False
        self.ekf_seeded = False
        self.fault = ''
        self.last_clock = None
        self.last_status = 0.0
        lio_topic = rospy.get_param('~lio_topic', '/fastlio_odom')
        wheel_topic = rospy.get_param('~wheel_topic', '/scout/odom')
        output_topic = rospy.get_param('~output_topic', '/scout/fused_odom')
        topics = [lio_topic, wheel_topic, output_topic, '/scout/fusion/lio_input',
                  '/scout/fusion/wheel_input', '/scout/fusion/ekf_raw']
        if len(set(rospy.resolve_name(t) for t in topics)) != len(topics):
            raise ValueError('Input/output topics must be distinct (feedback loop)')
        self.lio_pub = rospy.Publisher(topics[3], Odometry, queue_size=10)
        self.wheel_pub = rospy.Publisher(topics[4], Odometry, queue_size=20)
        self.pub = rospy.Publisher(output_topic, Odometry, queue_size=10)
        self.diag = rospy.Publisher('/scout/fusion/status', DiagnosticArray, queue_size=2)
        self.subs = [rospy.Subscriber(lio_topic, Odometry, self.on_lio, queue_size=20),
                     rospy.Subscriber(wheel_topic, Odometry, self.on_wheel, queue_size=50),
                     rospy.Subscriber(topics[5], Odometry, self.on_output, queue_size=20)]
        # Wall-clock watchdog still fires if /clock or all input streams stop.
        self.worker = threading.Thread(target=self.watchdog, daemon=True)
        self.worker.start()

    def clock(self):
        now, wall = self.ros.Time.now().to_sec(), time.monotonic()
        if self.last_clock is not None and now < self.last_clock - 0.001:
            self.fail('ROS clock moved backwards; restart fusion.launch')
        self.last_clock = now
        return now, wall

    def fail(self, reason):
        if not self.fault:
            self.fault = reason
            self.ros.logerr('Fusion output stopped: %s', reason)

    def on_lio(self, msg):
        with self.lock:
            now, wall = self.clock()
            if self.fault:
                return
            if msg.header.frame_id != self.world or msg.child_frame_id != self.body:
                self.fail('LIO must already be odom/base_link; raw camera_init/body is not accepted')
                return
            pose = msg.pose.pose
            if (not valid_quaternion(pose.orientation)
                    or not all(math.isfinite(v) for v in (pose.position.x, pose.position.y, pose.position.z))):
                self.fail('Invalid LIO pose')
                return
            stamp = msg.header.stamp.to_sec()
            previous_stamp = self.lio.stamp
            if not self.lio.accept(stamp, now, wall):
                return
            if self.previous_pose is not None:
                dt = stamp - previous_stamp
                p = self.previous_pose.position
                distance = math.sqrt((pose.position.x-p.x)**2 + (pose.position.y-p.y)**2 + (pose.position.z-p.z)**2)
                if (dt > self.timeout or distance > self.jump + self.vmax*dt
                        or rotation_distance(pose.orientation, self.previous_pose.orientation) > self.angle_jump + self.wmax*dt):
                    self.fail('LIO gap/reset/jump; restart fusion.launch after checking source')
                    return
            self.previous_pose = copy.deepcopy(pose)
            first = self.first_lio is None
            if first:
                self.first_lio = stamp
            out = copy.deepcopy(msg)
            # ROS publisher/subscriber startup is asynchronous: the first
            # published pose may never reach the EKF. Keep the origin prior
            # until an actual initialized EKF output has been observed.
            out.pose.covariance = diagonal(self.pose_var if self.ekf_seeded else self.initial_pose_var)
            # Config excludes ALL LIO twist fields. Preserve input message on original topic.
            self.lio_pub.publish(out)

    def on_wheel(self, msg):
        with self.lock:
            now, wall = self.clock()
            if self.fault or self.first_lio is None or not self.ekf_seeded:
                return
            if msg.child_frame_id != self.body:
                self.fail('Wheel twist must be at base_link origin with FLU axes')
                return
            v, omega = msg.twist.twist.linear.x, msg.twist.twist.angular.z
            if not (math.isfinite(v) and math.isfinite(omega)) or abs(v) > self.vmax or abs(omega) > self.wmax:
                self.wheel.rejected += 1
                return
            stamp = msg.header.stamp.to_sec()
            if stamp < self.first_lio or not self.wheel.accept(stamp, now, wall):
                return
            # Deliberately never read wheel pose or its world origin: driver resets are harmless.
            out = self.Odometry()
            out.header.stamp = msg.header.stamp
            out.header.frame_id, out.child_frame_id = self.world, self.body
            out.pose.pose.orientation.w = 1.0
            out.pose.covariance = diagonal([1e6]*6)
            out.twist.twist.linear.x = v
            scale = min(self.max_scale, 1.0 + (omega/self.turn_scale)**2)
            out.twist.covariance = diagonal([self.speed_var*scale] + [1e6]*5)
            self.wheel_pub.publish(out)

    def on_output(self, msg):
        with self.lock:
            now, wall = self.clock()
            if self.fault or not self.lio.fresh(now, wall, self.timeout):
                return
            if msg.header.frame_id != self.world or msg.child_frame_id != self.body:
                self.fail('Unexpected EKF output frame')
                return
            pose, twist = msg.pose.pose, msg.twist.twist
            values = [pose.position.x, pose.position.y, pose.position.z,
                      twist.linear.x, twist.linear.y, twist.linear.z,
                      twist.angular.x, twist.angular.y, twist.angular.z]
            values += list(msg.pose.covariance) + list(msg.twist.covariance)
            if not valid_quaternion(pose.orientation) or not all(math.isfinite(v) for v in values):
                self.fail('Non-finite EKF output')
                return
            if msg.header.stamp.to_sec() < self.first_lio:
                return
            # Wait for a pose-seeded EKF before admitting wheel velocity. Otherwise
            # a wheel-only first measurement can initialize the filter at (0,0,0).
            if not self.ekf_seeded:
                p = self.previous_pose.position
                if math.sqrt((pose.position.x-p.x)**2 + (pose.position.y-p.y)**2 + (pose.position.z-p.z)**2) > 0.25:
                    return
                if rotation_distance(pose.orientation, self.previous_pose.orientation) > 0.20:
                    return
                self.ekf_seeded = True
            if not self.wheel.fresh(now, wall, self.timeout):
                return
            if self.output.accept(msg.header.stamp.to_sec(), now, wall):
                self.started = True
                self.pub.publish(msg)

    def watchdog(self):
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        while not self.ros.is_shutdown():
            with self.lock:
                now, wall = self.clock()
                if self.started and not all(g.fresh(now, wall, self.timeout) for g in (self.lio, self.wheel, self.output)):
                    self.fail('Source or EKF dropout; restart entire fusion.launch, not guard alone')
                if wall - self.last_status >= 1.0:
                    status = DiagnosticStatus()
                    status.name = 'scout_shadow_fusion'
                    status.hardware_id = 'scout_mini'
                    status.level = 2 if self.fault else (0 if self.started else 1)
                    status.message = self.fault or ('SHADOW_OK_NOT_NAVIGATION' if self.started else 'WAITING_FOR_FRESH_INPUTS')
                    status.values = [KeyValue(key=k, value=str(v)) for k,v in (
                        ('profile', self.profile),
                        ('lio_rejected', self.lio.rejected), ('wheel_rejected', self.wheel.rejected),
                        ('lio_stamp', self.lio.stamp), ('wheel_stamp', self.wheel.stamp),
                        ('output_stamp', self.output.stamp))]
                    message = DiagnosticArray()
                    message.header.stamp = self.ros.Time.now()
                    message.status = [status]
                    self.diag.publish(message)
                    self.last_status = wall
            time.sleep(0.05)


if __name__ == '__main__':
    import rospy
    rospy.init_node('scout_fusion_guard')
    FusionGuard()
    rospy.spin()
