#!/usr/bin/env python3
"""Launch-owned exploration session: ready -> start -> finish/stop -> save."""
import os
import signal
import subprocess
import threading
import time

import rospy
from actionlib_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Supervisor:
    def __init__(self):
        self.lock = threading.RLock()
        self.auto_start = rospy.get_param('~auto_start', True)
        self.completion_hold = max(1.0, float(rospy.get_param('~completion_hold_time', 30.0)))
        self.completion_maps = max(2, int(rospy.get_param('~completion_map_updates', 2)))
        self.map_name = rospy.get_param('~map_name', 'current_exploration')
        self.map_timeout = float(rospy.get_param('~map_timeout', 2.0))
        self.map_status_topic = rospy.get_param('~map_status_topic', '/exploration/map_status')
        self.armed = False
        self.latched = True
        self.safety_reason = 'waiting_for_safety'
        self.available = self.frontier_valid = self.active = False
        self.frontier_since = self.empty_since = None
        self.empty_generation = 0
        self.idle_since = time.monotonic()
        self.safety_stamp = self.frontier_stamp = self.status_stamp = self.map_stamp = 0.0
        self.map_generation = 0
        self.last_map_message = None
        self.refresh_ok = False
        self.arm_requested = self.ever_armed = False
        self.arm_request_time = self.next_arm_try = 0.0
        self.restarts = 0
        self.child = None
        self.last_state = None
        self.result = 'STOPPED'
        self.paused = False
        self.resume_ready_since = None
        self.state_publisher = rospy.Publisher('/exploration/state', String, queue_size=1, latch=True)
        self.arm_service = rospy.ServiceProxy('/wheeltec_safety/arm', Trigger)
        self.stop_service = rospy.ServiceProxy('/wheeltec_safety/stop', Trigger)
        self.pause_service = rospy.ServiceProxy('/wheeltec_safety/pause', Trigger)
        self.resume_service = rospy.ServiceProxy('/wheeltec_safety/resume', Trigger)
        self.save_service = rospy.ServiceProxy('/wheeltec_pointcloud_mapper/save_map', Trigger)
        self.subs = [
            rospy.Subscriber('/wheeltec_safety/status', DiagnosticArray, self.safety),
            rospy.Subscriber('/exploration/frontier_status', String, self.frontier),
            rospy.Subscriber('/move_base/status', GoalStatusArray, self.status),
            rospy.Subscriber(self.map_status_topic, String, self.map_status),
        ]

    def map_status(self, message):
        with self.lock:
            if message.data.startswith('ready:'):
                self.refresh_ok = True
                if message.data != self.last_map_message:
                    self.map_generation += 1
                    self.map_stamp = time.monotonic()
                    self.last_map_message = message.data
            elif message.data.startswith('refresh failed:'):
                self.refresh_ok = False

    def publish_state(self, state):
        if state != self.last_state:
            self.state_publisher.publish(String(data=state))
            rospy.loginfo('Exploration state: %s', state)
            self.last_state = state

    def safety(self, message):
        with self.lock:
            for status in message.status:
                if status.name == 'wheeltec_safety':
                    values = {v.key: v.value for v in status.values}
                    self.latched = values.get('latched_stop') != 'False'
                    self.armed = values.get('armed') == 'True' and not self.latched
                    self.ever_armed |= self.armed
                    self.safety_reason = (values.get('latched_reason') if self.latched else None) or values.get('reason', status.message)
                    self.safety_stamp = time.monotonic()

    def frontier(self, message):
        # One atomic status distinguishes a valid empty map from missing TF/data.
        with self.lock:
            now = time.monotonic()
            self.frontier_valid = message.data in ('AVAILABLE', 'EMPTY')
            available = message.data == 'AVAILABLE'
            if available and (not self.available or now-self.frontier_stamp >= 2.5):
                self.frontier_since = now
            if not available:
                self.frontier_since = None
            self.available = available
            self.frontier_stamp = now

    def status(self, message):
        with self.lock:
            now = time.monotonic()
            active = any(s.status in (0, 1, 6, 7) for s in message.status_list)
            if self.active or active:
                self.idle_since = now
            self.active = active
            self.status_stamp = now

    def data_ready(self, now):
        return (now-self.safety_stamp < 1.0 and now-self.status_stamp < 1.0
                and self.map_stamp > 0.0 and now-self.map_stamp < self.map_timeout
                and self.refresh_ok and self.frontier_valid and now-self.frontier_stamp < 2.5)

    def can_run(self, now):
        return self.armed and not self.latched and self.data_ready(now)

    def stable_frontier(self, now):
        return (self.available and self.frontier_since is not None
                and now-self.frontier_since >= 2.0)

    def can_start(self, now):
        return self.can_run(now) and self.stable_frontier(now)

    def should_arm(self, now):
        return (self.auto_start and not self.arm_requested and not self.ever_armed
                and not self.latched and not self.armed and not self.active
                and now >= self.next_arm_try and self.data_ready(now)
                and self.refresh_ok and self.stable_frontier(now))

    def completed(self, now):
        enabled = self.auto_start or self.ever_armed
        if (not enabled or self.paused or not self.data_ready(now) or not self.refresh_ok
                or self.latched or self.active or self.available):
            self.empty_since = None
            return False
        if self.empty_since is None:
            self.empty_since = now
            self.empty_generation = self.map_generation
        # Wait for a frontier observation after the newest successful map load.
        if self.frontier_stamp < self.map_stamp:
            return False
        return (now-self.empty_since >= self.completion_hold
                and self.map_generation-self.empty_generation >= self.completion_maps)

    def can_restart(self, now):
        return (self.restarts < 2 and self.can_start(now) and not self.active
                and now-self.frontier_since >= 8.0 and now-self.idle_since >= 15.0)

    def start(self):
        self.child = subprocess.Popen(
            ['rosrun', 'wheeltec_explore_lite', 'wheeltec_explore',
             '__name:=explore', '_preview_only:=false'], start_new_session=True)

    def stop_child(self):
        child = self.child
        if child is not None and child.poll() is None:
            for sig, timeout in ((signal.SIGINT, 3), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
                try:
                    os.killpg(child.pid, sig)
                    child.wait(timeout=timeout)
                    break
                except ProcessLookupError:
                    break
                except subprocess.TimeoutExpired:
                    continue
        self.child = None

    def request_arm(self, now):
        self.next_arm_try = now + 2.0
        try:
            response = self.arm_service()
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(5.0, 'Waiting for safety readiness: %s', error)
            return
        if response.success:
            # Authorization is consumed once. Never reset or rearm after a stop.
            self.arm_requested = True
            self.arm_request_time = time.monotonic()
            rospy.loginfo('Launch startup accepted by safety gate; beginning exploration')
        else:
            rospy.logwarn_throttle(5.0, 'Waiting for safety readiness: %s', response.message)

    def hold_session(self, fault):
        # A safety latch is never reset/rearmed here. Preserve its first cause.
        if not self.paused:
            if not self.latched:
                try:
                    response = self.pause_service()
                    if not response.success:
                        self.stop_service()
                except rospy.ServiceException:
                    try:
                        self.stop_service()
                    except rospy.ServiceException:
                        rospy.logerr('Safety pause unavailable; cancelling goal producer')
            self.stop_child()
            self.paused = True
        self.empty_since = None
        self.resume_ready_since = None
        self.publish_state('FAULT_STOPPED' if fault else 'PAUSED_WAITING_FOR_DATA')

    def resume_if_ready(self, now):
        if not self.paused:
            return True
        if not self.can_run(now) or self.active:
            self.resume_ready_since = None
            return False
        if self.resume_ready_since is None:
            self.resume_ready_since = now
        if now - self.resume_ready_since < 2.0:
            return False
        try:
            response = self.resume_service()
        except rospy.ServiceException:
            return False
        if response.success:
            self.paused = False
            self.resume_ready_since = None
            self.idle_since = now
            return True
        return False

    def finish(self):
        # Stop/cancel first, before stopping the goal producer or saving a snapshot.
        try:
            self.stop_service()
        except rospy.ServiceException as error:
            rospy.logwarn('Stop service unavailable during shutdown: %s', error)
        self.stop_child()
        if not rospy.is_shutdown():
            try:
                saved = self.save_service()
                if not saved.success:
                    self.result = 'SAVE_FAILED'
                    rospy.logerr('Static point-cloud snapshot failed: %s', saved.message)
            except rospy.ServiceException as error:
                self.result = 'SAVE_FAILED'
                rospy.logerr('Static point-cloud snapshot unavailable: %s', error)
        # Ctrl+C also shuts the original mapper down; its normal final save remains.
        rospy.loginfo('Session %s. After launch exits, convert manually: '
                      'rosrun wheeltec_map_tools finalize_map.py %s', self.result, self.map_name)
        if not rospy.is_shutdown():
            self.publish_state(self.result)

    def run(self):
        try:
            while not rospy.is_shutdown():
                with self.lock:
                    now = time.monotonic()
                    lost_gate = self.ever_armed and not self.armed
                    failed = ((self.arm_requested or self.ever_armed)
                              and (self.latched or lost_gate
                                   or (not self.armed and now-self.arm_request_time > 2.0)))
                    stopped = self.latched and self.safety_stamp > 0
                    waiting = ((self.arm_requested or self.ever_armed)
                               and not self.data_ready(now))
                    done = self.completed(now)
                    arm = self.should_arm(now)
                    start = self.child is None and self.can_start(now)
                    restart = self.child is not None and self.can_restart(now)
                    active, available = self.active, self.available
                if failed or stopped:
                    self.result = 'STOPPED' if self.safety_reason == 'operator_stop' else 'FAULT'
                    rospy.logwarn_throttle(5.0, 'Exploration parked; mapping and diagnostics retained: %s', self.safety_reason)
                    self.hold_session(fault=True)
                    time.sleep(0.2)
                    continue
                if waiting:
                    self.hold_session(fault=False)
                    time.sleep(0.2)
                    continue
                if not self.resume_if_ready(now):
                    time.sleep(0.2)
                    continue
                if done:
                    self.result = 'COMPLETED'
                    rospy.loginfo('No qualifying reachable frontiers for %.1f s across %d new maps',
                                  self.completion_hold, self.completion_maps)
                    break
                if arm:
                    self.request_arm(now)
                if start:
                    self.idle_since = time.monotonic()
                    self.start()
                elif restart:
                    self.stop_child()
                    with self.lock:
                        if self.can_restart(time.monotonic()):
                            self.restarts += 1
                            self.idle_since = time.monotonic()
                            self.start()
                if self.child is not None:
                    self.publish_state('EXPLORING' if active else
                                       'WAITING_FOR_GOAL' if available else 'CONFIRMING_COMPLETE')
                elif not self.auto_start and not self.ever_armed:
                    self.publish_state('WAITING_FOR_ARM')
                elif self.empty_since is not None:
                    self.publish_state('CONFIRMING_COMPLETE')
                else:
                    self.publish_state('WAITING_FOR_READY')
                time.sleep(0.2)
        finally:
            self.finish()


if __name__ == '__main__':
    rospy.init_node('wheeltec_explore_supervisor')
    Supervisor().run()
