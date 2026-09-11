#!/usr/bin/env python3

import collections
import sys

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid


def main():
    rospy.init_node("wheeltec_exploration_check", anonymous=True)
    timeout = rospy.get_param("~timeout", 15.0)

    try:
        occupancy = rospy.wait_for_message(
            "/nav_static_map", OccupancyGrid, timeout=timeout
        )
        local = rospy.wait_for_message(
            "/exploration/observed_local_map", OccupancyGrid, timeout=timeout
        )
    except rospy.ROSException as error:
        rospy.logerr("Exploration topics unavailable: %s", error)
        return 2

    counts = collections.Counter(occupancy.data)
    invalid = sorted(set(occupancy.data) - {-1, 0, 100})
    local_counts = collections.Counter(local.data)
    print(
        "map frame={} size={}x{} resolution={:.3f} unknown={} free={} occupied={}".format(
            occupancy.header.frame_id,
            occupancy.info.width,
            occupancy.info.height,
            occupancy.info.resolution,
            counts[-1],
            counts[0],
            counts[100],
        )
    )
    print(
        "observed local unknown={} free={} occupied={}".format(
            local_counts[-1], local_counts[0], local_counts[100],
        )
    )

    if invalid:
        rospy.logerr("Map is not tri-state; invalid values: %s", invalid[:20])
        return 3
    if (rospy.Time.now()-local.header.stamp).to_sec() > 2.0:
        rospy.logerr("Local observation is stale")
        return 7
    if occupancy.header.frame_id != "map":
        rospy.logerr("Unexpected map frame: %s", occupancy.header.frame_id)
        return 4

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    try:
        transform = tf_buffer.lookup_transform(
            "odom", "base_link", rospy.Time(0), rospy.Duration(timeout)
        )
        print(
            "tf odom->base_link x={:.3f} y={:.3f} z={:.3f}".format(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            )
        )
    except tf2_ros.TransformException as error:
        rospy.logerr("TF unavailable: %s", error)
        return 5

    if counts[0] == 0:
        rospy.logwarn("No free cells yet; check live terrain topics and TF")
        return 6
    print("EXPLORATION_CHECK_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
