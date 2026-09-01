#!/usr/bin/env python3

import rospy
import tf2_ros

from nav_msgs.msg import Odometry


def main():
    rospy.init_node("scout_pose_adapter")

    # --------------------------------------------------------
    # 参数
    # --------------------------------------------------------

    parent_frame = rospy.get_param(
        "~parent_frame",
        "odom"
    )

    child_frame = rospy.get_param(
        "~child_frame",
        "base_link"
    )

    output_topic = rospy.get_param(
        "~output_topic",
        "/fastlio_odom"
    )

    publish_rate = rospy.get_param(
        "~publish_rate",
        20.0
    )

    # --------------------------------------------------------
    # TF Listener
    # --------------------------------------------------------

    tf_buffer = tf2_ros.Buffer(
        cache_time=rospy.Duration(10.0)
    )

    tf_listener = tf2_ros.TransformListener(
        tf_buffer
    )

    # --------------------------------------------------------
    # Odometry Publisher
    # --------------------------------------------------------

    odom_pub = rospy.Publisher(
        output_topic,
        Odometry,
        queue_size=20
    )

    rate = rospy.Rate(
        publish_rate
    )

    rospy.loginfo(
        "Scout Pose Adapter started."
    )

    rospy.loginfo(
        "TF: %s -> %s",
        parent_frame,
        child_frame
    )

    rospy.loginfo(
        "Output: %s",
        output_topic
    )

    # 给 TF Listener 一点时间建立缓存
    rospy.sleep(1.0)

    # --------------------------------------------------------
    # 主循环
    # --------------------------------------------------------

    while not rospy.is_shutdown():

        try:

            # 获取最新的：
            #
            # odom -> base_link
            #
            # 这里返回的是：
            # base_link 在 odom 坐标系中的位姿。
            transform = tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rospy.Time(0),
                rospy.Duration(0.1)
            )

            odom = Odometry()

            # 使用 TF 自己的时间戳
            if transform.header.stamp != rospy.Time(0):
                odom.header.stamp = transform.header.stamp
            else:
                odom.header.stamp = rospy.Time.now()

            odom.header.frame_id = parent_frame
            odom.child_frame_id = child_frame

            # ------------------------------------------------
            # Position
            # ------------------------------------------------

            odom.pose.pose.position.x = (
                transform.transform.translation.x
            )

            odom.pose.pose.position.y = (
                transform.transform.translation.y
            )

            odom.pose.pose.position.z = (
                transform.transform.translation.z
            )

            # ------------------------------------------------
            # Orientation
            # ------------------------------------------------

            odom.pose.pose.orientation.x = (
                transform.transform.rotation.x
            )

            odom.pose.pose.orientation.y = (
                transform.transform.rotation.y
            )

            odom.pose.pose.orientation.z = (
                transform.transform.rotation.z
            )

            odom.pose.pose.orientation.w = (
                transform.transform.rotation.w
            )

            # ------------------------------------------------
            # 当前节点只用于：
            #
            #   FAST-LIO 位姿转换
            #   PlotJuggler 数据比较
            #
            # 暂时不计算 twist。
            # ------------------------------------------------

            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0

            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = 0.0

            # 发布
            odom_pub.publish(
                odom
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ) as error:

            rospy.logwarn_throttle(
                2.0,
                "Waiting for TF %s -> %s: %s",
                parent_frame,
                child_frame,
                str(error)
            )

        rate.sleep()


if __name__ == "__main__":
    main()
