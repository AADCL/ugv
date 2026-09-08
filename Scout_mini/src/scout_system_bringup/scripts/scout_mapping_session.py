#!/usr/bin/env python3
"""Run one Scout mapping session and finalize it in a safe order."""

import argparse
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rospy
import yaml
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


MAP_NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


def stop_process_group(process):
    """Stop roslaunch cleanly, escalating only after bounded waits."""
    if process.poll() is not None:
        return process.returncode
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        return process.wait(timeout=30.0)
    except subprocess.TimeoutExpired:
        print("[WARN] roslaunch did not stop after 30 s; sending SIGTERM")
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    try:
        return process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        print("[ERROR] roslaunch still did not stop; sending SIGKILL")
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        return process.wait(timeout=5.0)


def finalize(map_name):
    command = [
        "rosrun",
        "scout_map_tools",
        "finalize_map.py",
        map_name,
        "--replace-raw",
    ]
    print("[STEP 3/3] generating PCD, PGM and 2.5D assets")
    result = subprocess.run(command, check=False)
    if result.returncode != 0:
        raise RuntimeError(
            "map finalization failed with exit code {}".format(
                result.returncode
            )
        )

    map_dir = Path.home() / "livox_fastlio" / "maps" / map_name
    required = (
        "raw_camera_init.pcd",
        "traversed_path_map.pcd",
        "public_map.pcd",
        "map_raw.pgm",
        "map_raw.yaml",
        "map.pgm",
        "map.yaml",
        "terrain_2p5d.yaml",
        "map_metadata.yaml",
    )
    missing = [
        name
        for name in required
        if not (map_dir / name).is_file()
        or (map_dir / name).stat().st_size == 0
    ]
    if missing:
        raise RuntimeError(
            "finalizer returned success but required outputs are missing: "
            + ", ".join(missing)
        )
    terrain_yaml = map_dir / "terrain_2p5d.yaml"
    with terrain_yaml.open("r", encoding="utf-8") as stream:
        terrain_index = yaml.safe_load(stream) or {}
    if terrain_index.get("format") != "scout_terrain_2p5d":
        raise RuntimeError("terrain_2p5d.yaml has an unsupported format")
    layers = terrain_index.get("layers")
    if not isinstance(layers, dict) or set(layers) != {
            "elevation", "slope_deg", "roughness", "step_height", "cost",
            "confidence"}:
        raise RuntimeError("terrain_2p5d.yaml does not index all six layers")
    map_root = map_dir.resolve()
    for layer_name, relative_path in layers.items():
        layer_path = (map_dir / str(relative_path)).resolve()
        if layer_path.parent != map_root:
            raise RuntimeError(
                "terrain layer {} escapes the map directory".format(layer_name)
            )
        if not layer_path.is_file() or layer_path.stat().st_size == 0:
            raise RuntimeError(
                "terrain layer {} is missing or empty: {}".format(
                    layer_name, layer_path
                )
            )
    print("[DONE] finalized map: {}".format(map_dir))


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Start the complete Scout mapping launch; on Ctrl+C, explicitly "
            "save the Bayesian map, stop all launch nodes, then finalize all "
            "map products."
        )
    )
    parser.add_argument("map_name")
    parser.add_argument(
        "--online-terrain-diagnostics",
        action="store_true",
        help="enable the optional online Patchwork++ diagnostic branch",
    )
    args = parser.parse_args()

    if not MAP_NAME_PATTERN.fullmatch(args.map_name) or args.map_name in (
            ".", ".."):
        parser.error(
            "map_name must contain only letters, digits, dot, underscore or "
            "hyphen, start with a letter/digit, and not be '.' or '..'"
        )
    if os.name != "posix":
        raise RuntimeError("the mapping session supervisor requires Linux")

    stop_requested = False

    def request_stop(signum, _frame):
        nonlocal stop_requested
        if not stop_requested:
            print(
                "\n[STEP 1/3] stop requested; holding /cmd_vel at zero and "
                "saving the mapper before shutting down ROS nodes"
            )
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, request_stop)

    launch_command = [
        "roslaunch",
        "scout_system_bringup",
        "scout_mapping.launch",
        "map_name:={}".format(args.map_name),
        "enable_online_terrain_diagnostics:={}".format(
            "true" if args.online_terrain_diagnostics else "false"
        ),
    ]
    print("[START] " + " ".join(launch_command))
    launch = subprocess.Popen(launch_command, preexec_fn=os.setsid)
    saved = False
    launch_exit = None
    try:
        print(
            "[RUNNING] Drive the complete mapping route. Park the vehicle, "
            "release teleop/remote motion, then press Ctrl+C once. Do not "
            "start a second mapping/localization launch."
        )
        while launch.poll() is None and not stop_requested:
            time.sleep(0.2)
        if not stop_requested:
            raise RuntimeError(
                "scout_mapping.launch exited unexpectedly with code {}".format(
                    launch.returncode
                )
            )

        rospy.init_node(
            "scout_mapping_session_supervisor",
            anonymous=True,
            disable_signals=True,
        )
        stop_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        zero_stop = threading.Event()

        def hold_zero_velocity():
            zero = Twist()
            while not zero_stop.is_set() and not rospy.is_shutdown():
                stop_publisher.publish(zero)
                time.sleep(0.05)

        zero_thread = threading.Thread(
            target=hold_zero_velocity, name="scout_zero_velocity", daemon=True
        )
        zero_thread.start()
        time.sleep(0.25)
        rospy.wait_for_service(
            "/scout_pointcloud_mapper/save_map", timeout=15.0
        )
        response = rospy.ServiceProxy(
            "/scout_pointcloud_mapper/save_map", Trigger
        )()
        if not response.success:
            raise RuntimeError("mapper refused save: " + response.message)
        print("[OK] " + response.message)
        map_dir = Path.home() / "livox_fastlio" / "maps" / args.map_name
        for required_source in (
                "filtered_camera_init.pcd", "traversed_path_map.pcd"):
            source = map_dir / required_source
            if not source.is_file() or source.stat().st_size == 0:
                raise RuntimeError(
                    "mapper save did not produce non-empty {}; check "
                    "/fastlio_odom and repeat the mapping session".format(source)
                )
        saved = True
    finally:
        print("[STEP 2/3] stopping mapping launch")
        launch_exit = stop_process_group(launch)
        if "zero_stop" in locals():
            zero_stop.set()
            zero_thread.join(timeout=1.0)

    if not saved:
        raise RuntimeError("map was not saved; finalization was not started")
    if launch_exit not in (0, 130, -signal.SIGINT):
        print(
            "[WARN] roslaunch stopped with code {}; saved map will still be "
            "finalized".format(launch_exit)
        )
    finalize(args.map_name)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("[ERROR] {}".format(error), file=sys.stderr)
        sys.exit(1)
