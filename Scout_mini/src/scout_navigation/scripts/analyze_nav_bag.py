#!/usr/bin/env python3
import argparse
import bisect
import csv
import glob
import math
import os
import statistics
import sys

import rosbag


DWA_LOCAL = "/move_base/DWAPlannerROS/local_plan"
DWA_TRAJ = "/move_base/DWAPlannerROS/trajectory_cloud"
TEB_LOCAL = "/move_base/TebLocalPlannerROS/local_plan"
TEB_POSES = "/move_base/TebLocalPlannerROS/teb_poses"


def bag_paths(path):
    path = os.path.abspath(os.path.expanduser(path))
    if os.path.isdir(path):
        files = sorted(glob.glob(os.path.join(path, "*.bag")))
    else:
        files = [path]
    files = [p for p in files if os.path.isfile(p)]
    if not files:
        raise FileNotFoundError("No .bag file found: " + path)
    return files


def write_csv(path, header, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def nearest_value(samples, ts, max_dt=0.25):
    if not samples:
        return None
    times = [x[0] for x in samples]
    i = bisect.bisect_left(times, ts)
    candidates = []
    if i < len(samples):
        candidates.append(samples[i])
    if i > 0:
        candidates.append(samples[i - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda x: abs(x[0] - ts))
    if abs(best[0] - ts) > max_dt:
        return None
    return best


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def path_length(poses):
    total = 0.0
    for i in range(1, len(poses)):
        p0 = poses[i - 1].pose.position
        p1 = poses[i].pose.position
        total += math.hypot(p1.x - p0.x, p1.y - p0.y)
    return total


def max_abs(rows, idx):
    return max((abs(x[idx]) for x in rows), default=0.0)


def max_gap(rows):
    if len(rows) < 2:
        return 0.0
    return max(rows[i][0] - rows[i - 1][0] for i in range(1, len(rows)))


def percentile(values, p):
    if not values:
        return 0.0
    values = sorted(values)
    if len(values) == 1:
        return values[0]
    k = (len(values) - 1) * p
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return values[int(k)]
    return values[f] * (c - k) + values[c] * (k - f)


def main():
    parser = argparse.ArgumentParser(description="Summarize Scout navigation rosbag (DWA/TEB compatible)")
    parser.add_argument("path", help="navigation run directory or .bag file")
    args = parser.parse_args()

    bags = bag_paths(args.path)
    if os.path.isdir(os.path.abspath(os.path.expanduser(args.path))):
        out_dir = os.path.abspath(os.path.expanduser(args.path))
    else:
        out_dir = os.path.dirname(bags[0])
    os.makedirs(out_dir, exist_ok=True)

    cmd = []
    odom = []
    local_plan = []
    dwa_traj_cloud = []
    teb_poses = []
    status_rows = []
    goals = []
    planner_fail_logs = []
    topic_counts = {}
    t_min = None
    t_max = None

    topics = [
        "/cmd_vel",
        "/scout/odom",
        DWA_LOCAL,
        DWA_TRAJ,
        TEB_LOCAL,
        TEB_POSES,
        "/move_base/status",
        "/move_base/goal",
        "/rosout_agg",
    ]

    fail_patterns = (
        "failed to find a valid plan",
        "cost functions discarded all candidates",
        "DWA planner failed to produce path",
        "failed to produce path",
        "no valid trajectories",
        "trajectory is not feasible",
        "infeasible trajectory",
        "timed-elastic-band detected an infeasible pose",
        "teblocalplannerros",
        "optimization failed",
        "diverged",
        "failed to obtain a local plan",
    )

    for path in bags:
        with rosbag.Bag(path, "r") as bag:
            for topic, msg, t in bag.read_messages(topics=topics):
                ts = t.to_sec()
                t_min = ts if t_min is None else min(t_min, ts)
                t_max = ts if t_max is None else max(t_max, ts)
                topic_counts[topic] = topic_counts.get(topic, 0) + 1

                if topic == "/cmd_vel":
                    cmd.append((ts, float(msg.linear.x), float(msg.angular.z)))
                elif topic == "/scout/odom":
                    odom.append((ts, float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z)))
                elif topic in (DWA_LOCAL, TEB_LOCAL):
                    local_plan.append((ts, topic, len(msg.poses), path_length(msg.poses)))
                elif topic == DWA_TRAJ:
                    dwa_traj_cloud.append((ts, int(msg.width) * int(msg.height)))
                elif topic == TEB_POSES:
                    teb_poses.append((ts, len(getattr(msg, "poses", []))))
                elif topic == "/move_base/status":
                    active = any(s.status == 1 for s in msg.status_list)
                    pending = any(s.status == 0 for s in msg.status_list)
                    status_rows.append((ts, int(active), int(pending), len(msg.status_list)))
                elif topic == "/move_base/goal":
                    pose = msg.goal.target_pose.pose
                    goals.append((
                        ts,
                        getattr(msg.goal_id, "id", ""),
                        float(pose.position.x),
                        float(pose.position.y),
                        float(yaw_from_quaternion(pose.orientation)),
                    ))
                elif topic == "/rosout_agg":
                    text = str(getattr(msg, "msg", ""))
                    lower = text.lower()
                    if any(pattern.lower() in lower for pattern in fail_patterns):
                        planner_fail_logs.append((ts, text))

    cmd.sort()
    odom.sort()
    local_plan.sort()
    dwa_traj_cloud.sort()
    teb_poses.sort()
    status_rows.sort()
    goals.sort()
    planner_fail_logs.sort()

    write_csv(os.path.join(out_dir, "cmd_vel.csv"), ["t", "linear_x", "angular_z"], cmd)
    write_csv(os.path.join(out_dir, "scout_odom_twist.csv"), ["t", "linear_x", "angular_z"], odom)
    write_csv(os.path.join(out_dir, "local_plan.csv"), ["t", "topic", "pose_count", "path_length_m"], local_plan)
    write_csv(os.path.join(out_dir, "trajectory_cloud.csv"), ["t", "point_count"], dwa_traj_cloud)
    write_csv(os.path.join(out_dir, "teb_poses.csv"), ["t", "pose_count"], teb_poses)
    write_csv(os.path.join(out_dir, "move_base_status.csv"), ["t", "active", "pending", "status_count"], status_rows)
    write_csv(os.path.join(out_dir, "goals.csv"), ["t", "goal_id", "x", "y", "yaw_rad"], goals)
    write_csv(os.path.join(out_dir, "planner_fail_logs.csv"), ["t", "message"], planner_fail_logs)
    # 兼容旧分析流程/文件名。
    write_csv(os.path.join(out_dir, "dwa_fail_logs.csv"), ["t", "message"], planner_fail_logs)

    lin_mismatch = 0
    lin_test = 0
    ang_mismatch = 0
    ang_test = 0
    for ts, vx, wz in cmd:
        nearest = nearest_value(odom, ts)
        if nearest is None:
            continue
        _, ovx, owz = nearest
        if abs(vx) >= 0.20:
            lin_test += 1
            if abs(ovx) < 0.05:
                lin_mismatch += 1
        if abs(wz) >= 0.20:
            ang_test += 1
            if abs(owz) < 0.05:
                ang_mismatch += 1

    empty_local = sum(1 for _, _, n, _ in local_plan if n == 0)
    local_lengths = [length for _, _, _, length in local_plan]
    local_pose_counts = [n for _, _, n, _ in local_plan]

    zero_dwa_traj = sum(1 for _, n in dwa_traj_cloud if n == 0)
    dwa_traj_counts = [n for _, n in dwa_traj_cloud]
    zero_teb_poses = sum(1 for _, n in teb_poses if n == 0)
    teb_pose_counts = [n for _, n in teb_poses]

    status_times = [x[0] for x in status_rows]

    def active_at(ts):
        if not status_rows:
            return False
        i = bisect.bisect_right(status_times, ts) - 1
        return i >= 0 and bool(status_rows[i][1])

    max_active_zero_cmd_sec = 0.0
    zero_start = None
    prev_t = None
    for ts, vx, wz in cmd:
        zero = abs(vx) < 0.01 and abs(wz) < 0.01 and active_at(ts)
        if zero:
            if zero_start is None or (prev_t is not None and ts - prev_t > 0.30):
                zero_start = ts
            max_active_zero_cmd_sec = max(max_active_zero_cmd_sec, ts - zero_start)
        else:
            zero_start = None
        prev_t = ts

    max_empty_local_plan_sec = 0.0
    empty_start = None
    prev_t = None
    for ts, _, n, _ in local_plan:
        if n == 0:
            if empty_start is None or (prev_t is not None and ts - prev_t > 0.30):
                empty_start = ts
            max_empty_local_plan_sec = max(max_empty_local_plan_sec, ts - empty_start)
        else:
            empty_start = None
        prev_t = ts

    duration = 0.0 if t_min is None or t_max is None else t_max - t_min

    nonzero_vx = [abs(vx) for _, vx, _ in cmd if abs(vx) >= 0.01]
    share_ge_030 = 0.0
    if nonzero_vx:
        share_ge_030 = sum(1 for x in nonzero_vx if x >= 0.30) / float(len(nonzero_vx))

    has_dwa = topic_counts.get(DWA_LOCAL, 0) > 0 or topic_counts.get(DWA_TRAJ, 0) > 0
    has_teb = topic_counts.get(TEB_LOCAL, 0) > 0 or topic_counts.get(TEB_POSES, 0) > 0
    if has_teb and not has_dwa:
        planner = "TEB"
    elif has_dwa and not has_teb:
        planner = "DWA"
    elif has_teb and has_dwa:
        planner = "MIXED/UNKNOWN"
    else:
        planner = "UNKNOWN"

    lines = []
    lines.append("Scout navigation log summary (DWA/TEB compatible)")
    lines.append("planner_detected: {}".format(planner))
    lines.append("bags: {}".format(len(bags)))
    lines.append("duration_sec: {:.3f}".format(duration))
    lines.append("goal_count: {}".format(len(goals)))
    for ts, goal_id, x, y, yaw in goals:
        rel = 0.0 if t_min is None else ts - t_min
        lines.append("  goal +{:.3f}s x={:.3f} y={:.3f} yaw_rad={:.3f} id={}".format(rel, x, y, yaw, goal_id))

    lines.append("cmd_vel_samples: {}".format(len(cmd)))
    lines.append("cmd_vel_rate_hz_overall: {:.3f}".format(len(cmd) / duration if duration > 0 else 0.0))
    lines.append("odom_samples: {}".format(len(odom)))
    lines.append("odom_rate_hz_overall: {:.3f}".format(len(odom) / duration if duration > 0 else 0.0))
    lines.append("max_abs_cmd_linear_x: {:.3f}".format(max_abs(cmd, 1)))
    lines.append("max_abs_cmd_angular_z: {:.3f}".format(max_abs(cmd, 2)))
    lines.append("max_abs_odom_linear_x: {:.3f}".format(max_abs(odom, 1)))
    lines.append("max_abs_odom_angular_z: {:.3f}".format(max_abs(odom, 2)))

    if nonzero_vx:
        lines.append("nonzero_cmd_linear_median: {:.3f}".format(statistics.median(nonzero_vx)))
        lines.append("nonzero_cmd_linear_p90: {:.3f}".format(percentile(nonzero_vx, 0.90)))
        lines.append("nonzero_cmd_linear_p95: {:.3f}".format(percentile(nonzero_vx, 0.95)))
        lines.append("nonzero_cmd_linear_share_ge_0.30: {:.3f}".format(share_ge_030))

    lines.append("local_plan_messages: {}".format(len(local_plan)))
    lines.append("empty_local_plan_messages: {}".format(empty_local))
    lines.append("max_local_plan_gap_sec: {:.3f}".format(max_gap([(r[0],) for r in local_plan])))
    lines.append("max_active_zero_cmd_sec: {:.3f}".format(max_active_zero_cmd_sec))
    lines.append("max_empty_local_plan_sec: {:.3f}".format(max_empty_local_plan_sec))
    if local_pose_counts:
        lines.append("local_plan_pose_count_median: {:.1f}".format(statistics.median(local_pose_counts)))
        lines.append("local_plan_pose_count_max: {}".format(max(local_pose_counts)))
    if local_lengths:
        lines.append("local_plan_length_m_median: {:.3f}".format(statistics.median(local_lengths)))
        lines.append("local_plan_length_m_p90: {:.3f}".format(percentile(local_lengths, 0.90)))
        lines.append("local_plan_length_m_max: {:.3f}".format(max(local_lengths)))

    lines.append("dwa_trajectory_cloud_messages: {}".format(len(dwa_traj_cloud)))
    lines.append("dwa_zero_point_trajectory_cloud_messages: {}".format(zero_dwa_traj))
    if dwa_traj_counts:
        lines.append("dwa_trajectory_cloud_points_median: {:.1f}".format(statistics.median(dwa_traj_counts)))
        lines.append("dwa_trajectory_cloud_points_max: {}".format(max(dwa_traj_counts)))

    lines.append("teb_poses_messages: {}".format(len(teb_poses)))
    lines.append("teb_zero_pose_messages: {}".format(zero_teb_poses))
    if teb_pose_counts:
        lines.append("teb_pose_count_median: {:.1f}".format(statistics.median(teb_pose_counts)))
        lines.append("teb_pose_count_max: {}".format(max(teb_pose_counts)))

    lines.append("planner_failed_logs: {}".format(len(planner_fail_logs)))
    lines.append("linear_cmd_without_odom_response: {}/{}".format(lin_mismatch, lin_test))
    lines.append("angular_cmd_without_odom_response: {}/{}".format(ang_mismatch, ang_test))

    lines.append("topic_counts:")
    for topic in sorted(topic_counts):
        lines.append("  {}: {}".format(topic, topic_counts[topic]))

    lines.append("")
    lines.append("Interpretation hints:")
    lines.append("- TEB: local_plan_length_m and teb_poses are the primary trajectory-shape indicators.")
    lines.append("- DWA: trajectory_cloud zero counts remain useful for legacy comparisons.")
    lines.append("- long max_active_zero_cmd_sec + planner failure logs/local-plan gaps: local planner/costmap feasibility issue is likely.")
    lines.append("- nonzero cmd_vel but high *_cmd_without_odom_response: chassis execution/dead-zone issue is likely.")
    lines.append("- low open-space linear command with TEB: inspect weight_optimaltime and velocity/acceleration limits before raising max_vel_x.")
    lines.append("- use goals.csv timestamps to split one session into individual goal tests.")

    summary_path = os.path.join(out_dir, "summary.txt")
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    print("\n".join(lines))
    print("[OK] summary: " + summary_path)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("[ERROR] {}".format(e), file=sys.stderr)
        sys.exit(1)

