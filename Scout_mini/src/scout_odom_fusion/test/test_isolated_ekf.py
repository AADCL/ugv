#!/usr/bin/env python3
"""Synthetic FLU/zero-reset/dropout test on a private ROS master, never the robot master."""
import math
import argparse
import os
import signal
import socket
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--profile', choices=('baseline', 'wheel_priority'), default='baseline')
    parser.add_argument('--yaw-deg', type=float, default=90.)
    parser.add_argument('--turn-rate', type=float, default=0.)
    parser.add_argument('--speed', type=float, default=.2)
    parser.add_argument('--lio-scale', type=float, default=1.)
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--lio-delay', type=float, default=.08)
    args = parser.parse_args()
    yaw0 = math.radians(args.yaw_deg)

    def trajectory(t):
        angle = yaw0 + args.turn_rate*t
        if abs(args.turn_rate) < 1e-9:
            return args.speed*t*math.cos(yaw0), args.speed*t*math.sin(yaw0), angle
        return (args.speed/args.turn_rate*(math.sin(angle)-math.sin(yaw0)),
                args.speed/args.turn_rate*(math.cos(yaw0)-math.cos(angle)), angle)

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
            children.append(subprocess.Popen(['roslaunch', 'scout_odom_fusion', 'fusion.launch',
                                             'profile:='+args.profile], stdout=log, stderr=log))
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
            while time.monotonic() - start < args.duration:
                elapsed = time.monotonic() - start
                stamp = rospy.Time.now()
                wheel = Odometry()
                wheel.header.stamp, wheel.header.frame_id, wheel.child_frame_id = stamp, 'scout_odom', 'base_link'
                wheel.pose.pose.orientation.w = 1
                wheel.pose.pose.position.x = 100 + .2*elapsed if elapsed < 5 else 0
                wheel.twist.twist.linear.x = args.speed
                wheel.twist.twist.angular.z = args.turn_rate
                wheel_pub.publish(wheel)
                if step % 5 == 0:
                    lio = Odometry()
                    # LIO arrives 80 ms late; timestamp and trajectory agree.
                    lio.header.stamp = stamp - rospy.Duration(args.lio_delay)
                    lio.header.frame_id, lio.child_frame_id = 'odom', 'base_link'
                    dx, dy, angle = trajectory(max(0, elapsed-args.lio_delay))
                    lio.pose.pose.position.x = 10 + args.lio_scale*dx
                    lio.pose.pose.position.y = 20 + args.lio_scale*dy
                    lio.pose.pose.orientation.z = math.sin(angle/2)
                    lio.pose.pose.orientation.w = math.cos(angle/2)
                    lio_pub.publish(lio)
                step += 1
                time.sleep(.02)
            assert len(outputs) > 100, ('Too few outputs', len(outputs), log_dir)
            last = outputs[-1]
            print('first=%s last=%s dt=%.3f first_xy=(%.4f, %.4f)' %
                  (outputs[0].header.stamp,last.header.stamp,
                   (last.header.stamp-outputs[0].header.stamp).to_sec(),
                   outputs[0].pose.pose.position.x,outputs[0].pose.pose.position.y),flush=True)
            dx, dy, _ = trajectory(elapsed)
            error = math.hypot(last.pose.pose.position.x-10-dx, last.pose.pose.position.y-20-dy)
            print('RESULT xy=(%.4f, %.4f) expected=(%.4f, %.4f) body_v=(%.4f, %.4f) error=%.4f' %
                  (last.pose.pose.position.x,last.pose.pose.position.y,10+dx,20+dy,
                   last.twist.twist.linear.x,last.twist.twist.linear.y,error), flush=True)
            if args.lio_scale == 1. or args.profile == 'wheel_priority':
                assert error < .30, ('Wrong body/world transformation or excessive LIO pull', error, log_dir)
            assert abs(last.twist.twist.linear.x - args.speed) < .1
            assert all(m.header.frame_id == 'odom' and m.child_frame_id == 'base_link' for m in outputs)
            assert all(b.header.stamp > a.header.stamp for a,b in zip(outputs, outputs[1:]))
            assert not tf_messages, 'Shadow nodes published TF'
            time.sleep(.8)
            stopped_count = len(outputs)
            time.sleep(1.3)
            assert len(outputs) == stopped_count, 'Output continued after input dropout'
            assert statuses[-1].status[0].level == 2, 'Dropout was not diagnosed'
            print('PASS: profile=%s yaw=%g turn=%g speed=%g lio_scale=%g endpoint_error=%.4fm; nonzero origin, delayed LIO, wheel reset, no TF, monotonic stamps, dropout stop' %
                  (args.profile,args.yaw_deg,args.turn_rate,args.speed,args.lio_scale,error))
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
