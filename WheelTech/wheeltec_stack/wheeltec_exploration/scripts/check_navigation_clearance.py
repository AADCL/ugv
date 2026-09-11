#!/usr/bin/env python3

import collections
import math
import sys

import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2


HALF_LENGTH = 0.25
HALF_WIDTH = 0.20
TEB_CLEARANCE = 0.04


def yaw_from_quaternion(rotation):
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def footprint_clearance(x, y):
    dx = max(abs(x) - HALF_LENGTH, 0.0)
    dy = max(abs(y) - HALF_WIDTH, 0.0)
    return math.hypot(dx, dy)


def rotate_point(point, rotation):
    qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
    tx = 2.0 * (qy * point[2] - qz * point[1])
    ty = 2.0 * (qz * point[0] - qx * point[2])
    tz = 2.0 * (qx * point[1] - qy * point[0])
    return (
        point[0] + qw * tx + qy * tz - qz * ty,
        point[1] + qw * ty + qz * tx - qx * tz,
        point[2] + qw * tz + qx * ty - qy * tx,
    )


def main():
    rospy.init_node("wheeltec_navigation_clearance_check", anonymous=True)
    timeout = rospy.get_param("~timeout", 10.0)
    try:
        costmap = rospy.wait_for_message(
            "/move_base/local_costmap/costmap", OccupancyGrid, timeout=timeout
        )
        obstacles = rospy.wait_for_message(
            "/terrain/obstacle_points", PointCloud2, timeout=timeout
        )
    except rospy.ROSException as error:
        rospy.logerr("Navigation diagnostic topics unavailable: %s", error)
        return 2

    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    try:
        map_from_robot = buffer.lookup_transform(
            costmap.header.frame_id, "base_link", rospy.Time(0), rospy.Duration(timeout)
        )
        robot_from_cloud = buffer.lookup_transform(
            "base_link", obstacles.header.frame_id, rospy.Time(0), rospy.Duration(timeout)
        )
    except tf2_ros.TransformException as error:
        rospy.logerr("Navigation diagnostic TF unavailable: %s", error)
        return 3

    robot_x = map_from_robot.transform.translation.x
    robot_y = map_from_robot.transform.translation.y
    robot_yaw = yaw_from_quaternion(map_from_robot.transform.rotation)
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    resolution = costmap.info.resolution
    origin_x = costmap.info.origin.position.x
    origin_y = costmap.info.origin.position.y

    counts = collections.Counter(costmap.data)
    lethal_inside = 0
    lethal_clearance = 0
    nearest_lethal = float("inf")
    for index, value in enumerate(costmap.data):
        if value < 99:
            continue
        mx = index % costmap.info.width
        my = index // costmap.info.width
        world_x = origin_x + (mx + 0.5) * resolution
        world_y = origin_y + (my + 0.5) * resolution
        dx = world_x - robot_x
        dy = world_y - robot_y
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        clearance = footprint_clearance(local_x, local_y)
        nearest_lethal = min(nearest_lethal, clearance)
        if clearance == 0.0:
            lethal_inside += 1
        if clearance <= TEB_CLEARANCE:
            lethal_clearance += 1

    transform = robot_from_cloud.transform
    cloud_inside = 0
    cloud_clearance = 0
    nearest_cloud = float("inf")
    cloud_count = 0
    for point in point_cloud2.read_points(
        obstacles, field_names=("x", "y", "z"), skip_nans=True
    ):
        rotated = rotate_point(point, transform.rotation)
        local_x = rotated[0] + transform.translation.x
        local_y = rotated[1] + transform.translation.y
        clearance = footprint_clearance(local_x, local_y)
        nearest_cloud = min(nearest_cloud, clearance)
        cloud_count += 1
        if clearance == 0.0:
            cloud_inside += 1
        if clearance <= TEB_CLEARANCE:
            cloud_clearance += 1

    def distance_text(value):
        return "n/a" if not math.isfinite(value) else "{:.3f}".format(value)

    print(
        "local_costmap free={} soft={} lethal={} unknown={}".format(
            counts[0],
            sum(count for value, count in counts.items() if 0 < value < 99),
            sum(count for value, count in counts.items() if value >= 99),
            counts[-1],
        )
    )
    print(
        "costmap footprint_inside={} within_4cm={} nearest_m={}".format(
            lethal_inside, lethal_clearance, distance_text(nearest_lethal)
        )
    )
    print(
        "obstacle_cloud points={} footprint_inside={} within_4cm={} nearest_m={}".format(
            cloud_count, cloud_inside, cloud_clearance, distance_text(nearest_cloud)
        )
    )
    if lethal_inside or cloud_inside:
        print("SELF_OBSTACLE_DETECTED")
        return 4
    print("NAVIGATION_CLEARANCE_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
