#!/usr/bin/env python3
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler


def deg2rad(v):
    return v * math.pi / 180.0


def main():
    rospy.init_node("scout_geometry_tf_publisher")

    base = "/scout_geometry/odom_to_camera_init"

    x = rospy.get_param(base + "/x")
    y = rospy.get_param(base + "/y")
    z = rospy.get_param(base + "/z")

    roll = deg2rad(rospy.get_param(base + "/roll_deg"))
    pitch = deg2rad(rospy.get_param(base + "/pitch_deg"))
    yaw = deg2rad(rospy.get_param(base + "/yaw_deg"))

    q = quaternion_from_euler(roll, pitch, yaw)

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "camera_init"

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(t)

    rospy.loginfo(
        "odom -> camera_init: xyz=(%.4f, %.4f, %.4f), "
        "rpy_deg=(%.3f, %.3f, %.3f)",
        x, y, z,
        rospy.get_param(base + "/roll_deg"),
        rospy.get_param(base + "/pitch_deg"),
        rospy.get_param(base + "/yaw_deg"),
    )

    rospy.spin()


if __name__ == "__main__":
    main()

