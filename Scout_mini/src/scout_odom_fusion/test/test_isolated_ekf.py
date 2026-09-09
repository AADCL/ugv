#!/usr/bin/env python3
"""Synthetic FLU/zero-reset/dropout test on a private ROS master, never the robot master."""
import math
import os
import signal
import socket
import subprocess
import tempfile
import time


def main():
    port = 11431
    probe = socket.socket()
    try:
        probe.bind(('127.0.0.1', port))
    finally:
        probe.close()
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ['ROS_HOSTNAME'] = '127.0.0.1'
    os.environ.pop('ROS_IP', None)
    log_dir = tempfile.mkdtemp(prefix='scout_fusion_test_')
    children = []
    with open(log_dir + '/processes.log', 'w') as log:
        try:
            children.append(subprocess.Popen(['roscore', '-p', str(port)], stdout=log, stderr=log))
            import xmlrpc.client
            master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
            deadline = time.monotonic() + 15
            while True:
                try:
                    if master.getPid('/fusion_test')[0] == 1:
                        break
                except OSError:
                    pass
                if time.monotonic() > deadline:
                    raise RuntimeError('Private master did not start')
                time.sleep(.1)
            children.append(subprocess.Popen(['roslaunch', 'scout_odom_fusion', 'fusion.launch'], stdout=log, stderr=log))
            import rospy
            from nav_msgs.msg import Odometry
            from diagnostic_msgs.msg import DiagnosticArray
            from tf2_msgs.msg import TFMessage
            rospy.init_node('isolated_fusion_test', anonymous=True, disable_signals=True)
            outputs, statuses, tf_messages = [], [], []
            subs = [rospy.Subscriber('/scout/fused_odom', Odometry, outputs.append),
                    rospy.Subscriber('/scout/fusion/status', DiagnosticArray, statuses.append),
                    rospy.Subscriber('/tf', TFMessage, tf_messages.append),
                    rospy.Subscriber('/tf_static', TFMessage, tf_messages.append)]
            lio_pub = rospy.Publisher('/fastlio_odom', Odometry, queue_size=10)
            wheel_pub = rospy.Publisher('/scout/odom', Odometry, queue_size=10)
            deadline = time.monotonic() + 15
            while not (lio_pub.get_num_connections() and wheel_pub.get_num_connections()):
                if time.monotonic() > deadline:
                    raise RuntimeError('Fusion subscribers did not start')
                time.sleep(.1)
            start = time.monotonic()
            step = 0
            while time.monotonic() - start < 10:
                elapsed = time.monotonic() - start
                stamp = rospy.Time.now()
                wheel = Odometry()
                wheel.header.stamp, wheel.header.frame_id, wheel.child_frame_id = stamp, 'scout_odom', 'base_link'
                wheel.pose.pose.orientation.w = 1
                wheel.pose.pose.position.x = 100 + .2*elapsed if elapsed < 5 else 0
                wheel.twist.twist.linear.x = .2
                wheel_pub.publish(wheel)
                if step % 5 == 0:
                    lio = Odometry()
                    # LIO arrives 80 ms late; timestamp and trajectory agree.
                    lio.header.stamp = stamp - rospy.Duration(.08)
                    lio.header.frame_id, lio.child_frame_id = 'odom', 'base_link'
                    lio.pose.pose.position.x = 10
                    lio.pose.pose.position.y = 20 + .2*max(0, elapsed-.08)
                    lio.pose.pose.orientation.z = math.sin(math.pi/4)
                    lio.pose.pose.orientation.w = math.cos(math.pi/4)
                    lio_pub.publish(lio)
                step += 1
                time.sleep(.02)
            assert len(outputs) > 100, ('Too few outputs', len(outputs), log_dir)
            last = outputs[-1]
            assert abs(last.pose.pose.position.x - 10) < .15, 'Body vx incorrectly treated as world vx'
            assert abs(last.pose.pose.position.y - 22) < .25, 'Incorrect origin, timing or wheel reset'
            assert abs(last.twist.twist.linear.x - .2) < .1
            assert all(m.header.frame_id == 'odom' and m.child_frame_id == 'base_link' for m in outputs)
            assert all(b.header.stamp > a.header.stamp for a,b in zip(outputs, outputs[1:]))
            assert not tf_messages, 'Shadow nodes published TF'
            time.sleep(.8)
            stopped_count = len(outputs)
            time.sleep(1.3)
            assert len(outputs) == stopped_count, 'Output continued after input dropout'
            assert statuses[-1].status[0].level == 2, 'Dropout was not diagnosed'
            print('PASS: nonzero origin, FLU 90-degree yaw, delayed LIO, wheel pose reset, no TF, monotonic stamps, dropout stop')
            print('outputs=%d last_xy=(%.4f, %.4f) logs=%s' % (len(outputs), last.pose.pose.position.x, last.pose.pose.position.y, log_dir))
        finally:
            for child in reversed(children):
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=12)
                    except subprocess.TimeoutExpired:
                        child.terminate()
                        child.wait(timeout=5)


if __name__ == '__main__':
    main()
