#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest


class GlobalPlanTester:
    def __init__(self):
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.tolerance = rospy.get_param("~tolerance", 0.15)
        self.service_name = rospy.get_param(
            "~make_plan_service",
            "/move_base/make_plan"
        )

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.plan_pub = rospy.Publisher(
            "/scout_global_plan_test",
            Path,
            queue_size=1,
            latch=True
        )

        rospy.loginfo("Waiting for %s ...", self.service_name)
        rospy.wait_for_service(self.service_name)
        self.make_plan = rospy.ServiceProxy(self.service_name, GetPlan)

        self.sub = rospy.Subscriber(
            "/clicked_point",
            PointStamped,
            self.point_callback,
            queue_size=1
        )

        rospy.loginfo(
            "Global plan tester ready. RViz -> Publish Point to request a plan."
        )

    def current_start_pose(self):
        tf_msg = self.tf_buffer.lookup_transform(
            self.global_frame,
            self.base_frame,
            rospy.Time(0),
            rospy.Duration(0.5)
        )

        start = PoseStamped()
        start.header.stamp = rospy.Time.now()
        start.header.frame_id = self.global_frame

        start.pose.position.x = tf_msg.transform.translation.x
        start.pose.position.y = tf_msg.transform.translation.y
        start.pose.position.z = tf_msg.transform.translation.z
        start.pose.orientation = tf_msg.transform.rotation

        return start

    def point_callback(self, msg):
        try:
            point_map = self.tf_buffer.transform(
                msg,
                self.global_frame,
                rospy.Duration(0.5)
            )

            start = self.current_start_pose()

            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = self.global_frame
            goal.pose.position.x = point_map.point.x
            goal.pose.position.y = point_map.point.y
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0

            req = GetPlanRequest()
            req.start = start
            req.goal = goal
            req.tolerance = self.tolerance

            result = self.make_plan(req)

            if not result.plan.poses:
                rospy.logwarn(
                    "No global plan: start=(%.2f, %.2f), goal=(%.2f, %.2f)",
                    start.pose.position.x,
                    start.pose.position.y,
                    goal.pose.position.x,
                    goal.pose.position.y
                )
                return

            # move_base/make_plan 在 ROS Noetic 中可能返回
            # plan.poses 有效，但 nav_msgs/Path 顶层 header 为空。
            # RViz 的 Path Display 需要合法的 frame_id。
            plan = result.plan
            plan.header.stamp = rospy.Time.now()
            plan.header.frame_id = self.global_frame

            # 保险处理：确保每个 PoseStamped 也有合法 frame_id。
            for pose in plan.poses:
                if not pose.header.frame_id:
                    pose.header.frame_id = self.global_frame

            self.plan_pub.publish(plan)

            rospy.loginfo(
                "Global plan OK: %d poses, start=(%.2f, %.2f), goal=(%.2f, %.2f)",
                len(plan.poses),
                start.pose.position.x,
                start.pose.position.y,
                goal.pose.position.x,
                goal.pose.position.y
            )

        except Exception as e:
            rospy.logerr("Global plan test failed: %s", str(e))


def main():
    rospy.init_node("scout_global_plan_tester")
    GlobalPlanTester()
    rospy.spin()


if __name__ == "__main__":
    main()

