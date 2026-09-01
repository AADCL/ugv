#!/usr/bin/env python3

import math
import rospy
import tf2_ros
import tf.transformations as tft

from geometry_msgs.msg import TransformStamped


def create_matrix(x, y, z, roll, pitch, yaw):
    """
    xyz + RPY -> 4x4 齐次变换矩阵
    """

    translation = tft.translation_matrix(
        [x, y, z]
    )

    quaternion = tft.quaternion_from_euler(
        roll,
        pitch,
        yaw
    )

    rotation = tft.quaternion_matrix(
        quaternion
    )

    return tft.concatenate_matrices(
        translation,
        rotation
    )


def matrix_to_tf(matrix, parent, child):
    """
    4x4 齐次变换矩阵 -> TransformStamped
    """

    translation = tft.translation_from_matrix(
        matrix
    )

    quaternion = tft.quaternion_from_matrix(
        matrix
    )

    msg = TransformStamped()

    msg.header.stamp = rospy.Time.now()

    msg.header.frame_id = parent
    msg.child_frame_id = child

    msg.transform.translation.x = float(
        translation[0]
    )

    msg.transform.translation.y = float(
        translation[1]
    )

    msg.transform.translation.z = float(
        translation[2]
    )

    msg.transform.rotation.x = float(
        quaternion[0]
    )

    msg.transform.rotation.y = float(
        quaternion[1]
    )

    msg.transform.rotation.z = float(
        quaternion[2]
    )

    msg.transform.rotation.w = float(
        quaternion[3]
    )

    return msg


def main():

    rospy.init_node(
        "wheeltec_tf_manager"
    )

    transforms = rospy.get_param(
        "~transforms",
        []
    )

    if not transforms:
        rospy.logfatal(
            "No static transforms configured."
        )
        return

    broadcaster = (
        tf2_ros.StaticTransformBroadcaster()
    )

    messages = []

    for item in transforms:

        name = item.get(
            "name",
            "unnamed"
        )

        parent = item["parent"]
        child = item["child"]

        x = float(
            item.get("x", 0.0)
        )

        y = float(
            item.get("y", 0.0)
        )

        z = float(
            item.get("z", 0.0)
        )

        roll = math.radians(
            float(
                item.get(
                    "roll_deg",
                    0.0
                )
            )
        )

        pitch = math.radians(
            float(
                item.get(
                    "pitch_deg",
                    0.0
                )
            )
        )

        yaw = math.radians(
            float(
                item.get(
                    "yaw_deg",
                    0.0
                )
            )
        )

        matrix = create_matrix(
            x,
            y,
            z,
            roll,
            pitch,
            yaw
        )

        if item.get(
            "publish_inverse",
            False
        ):

            # 对完整刚体变换求逆。
            matrix = tft.inverse_matrix(
                matrix
            )

            tf_parent = child
            tf_child = parent

        else:

            tf_parent = parent
            tf_child = child

        messages.append(
            matrix_to_tf(
                matrix,
                tf_parent,
                tf_child
            )
        )

        rospy.loginfo(
            "[%s] static TF: %s -> %s",
            name,
            tf_parent,
            tf_child
        )

    broadcaster.sendTransform(
        messages
    )

    rospy.loginfo(
        "Published %d static transform(s).",
        len(messages)
    )

    rospy.spin()


if __name__ == "__main__":
    main()
