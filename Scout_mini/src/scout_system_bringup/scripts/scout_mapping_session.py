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


def pcd_header(path, allow_empty=False):
    """Validate a generated PCD header without loading its point payload."""
    fields = {}
    payload_offset = None
    with path.open("rb") as stream:
        for _ in range(128):
            raw_line = stream.readline(65536)
            if not raw_line:
                break
            try:
                line = raw_line.decode("ascii").strip()
            except UnicodeDecodeError as error:
                raise RuntimeError("invalid PCD header encoding: {}".format(path)) from error
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            fields[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                payload_offset = stream.tell()
                break
    required = {
        "VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT",
        "POINTS", "DATA",
    }
    missing = sorted(required.difference(fields))
    if missing:
        raise RuntimeError(
            "invalid PCD header {}: missing {}".format(path, ", ".join(missing))
        )
    try:
        points = int(fields["POINTS"][0])
        width = int(fields["WIDTH"][0])
        height = int(fields["HEIGHT"][0])
    except (IndexError, ValueError) as error:
        raise RuntimeError("invalid PCD dimensions: {}".format(path)) from error
    if points < 0 or width < 0 or height <= 0 or points != width * height:
        raise RuntimeError("inconsistent PCD dimensions: {}".format(path))
    if not allow_empty and points == 0:
        raise RuntimeError("PCD contains no points: {}".format(path))
    if not fields["DATA"] or fields["DATA"][0].lower() not in (
            "ascii", "binary", "binary_compressed"):
        raise RuntimeError("unsupported PCD DATA encoding: {}".format(path))
    encoding = fields["DATA"][0].lower()
    try:
        sizes = [int(value) for value in fields["SIZE"]]
        counts = [int(value) for value in fields["COUNT"]]
    except ValueError as error:
        raise RuntimeError("invalid PCD SIZE/COUNT values: {}".format(path)) from error
    field_count = len(fields["FIELDS"])
    if (field_count == 0 or len(sizes) != field_count or
            len(fields["TYPE"]) != field_count or len(counts) != field_count or
            any(size <= 0 for size in sizes) or any(count <= 0 for count in counts)):
        raise RuntimeError("inconsistent PCD field metadata: {}".format(path))
    if encoding == "binary":
        expected_size = payload_offset + points * sum(
            size * count for size, count in zip(sizes, counts)
        )
        if path.stat().st_size != expected_size:
            raise RuntimeError(
                "PCD binary payload size mismatch: expected {} bytes: {}".format(
                    expected_size, path
                )
            )
    elif points > 0 and path.stat().st_size <= payload_offset:
        raise RuntimeError("PCD payload is missing: {}".format(path))
    return points


def validate_pgm(path):
    with path.open("rb") as stream:
        tokens = []
        while len(tokens) < 4:
            line = stream.readline(65536)
            if not line:
                break
            tokens.extend(line.split(b"#", 1)[0].split())
        payload_offset = stream.tell()
    try:
        magic = tokens[0]
        width = int(tokens[1])
        height = int(tokens[2])
        maximum = int(tokens[3])
    except (IndexError, ValueError) as error:
        raise RuntimeError("invalid PGM header: {}".format(path)) from error
    if magic != b"P5" or width <= 0 or height <= 0 or maximum != 255:
        raise RuntimeError("invalid PGM dimensions or encoding: {}".format(path))
    expected_size = payload_offset + width * height
    if path.stat().st_size != expected_size:
        raise RuntimeError(
            "PGM payload size mismatch: expected {} bytes: {}".format(
                expected_size, path
            )
        )


def validate_map_yaml(map_dir, yaml_name):
    yaml_path = map_dir / yaml_name
    with yaml_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    image = config.get("image")
    if not isinstance(image, str) or not image:
        raise RuntimeError("map YAML does not name an image: {}".format(yaml_path))
    image_path = (map_dir / image).resolve()
    if image_path.parent != map_dir.resolve():
        raise RuntimeError("map YAML image escapes map directory: {}".format(yaml_path))
    if not image_path.is_file() or image_path.stat().st_size == 0:
        raise RuntimeError("map image is missing or empty: {}".format(image_path))
    validate_pgm(image_path)


def validate_terrain_layers(map_dir):
    terrain_yaml = map_dir / "terrain_2p5d.yaml"
    with terrain_yaml.open("r", encoding="utf-8") as stream:
        terrain_index = yaml.safe_load(stream) or {}
    if terrain_index.get("format") != "scout_terrain_2p5d":
        raise RuntimeError("terrain_2p5d.yaml has an unsupported format")
    try:
        width = int(terrain_index["width"])
        height = int(terrain_index["height"])
        resolution = float(terrain_index["resolution"])
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError("terrain_2p5d.yaml has invalid dimensions") from error
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise RuntimeError("terrain_2p5d.yaml has invalid dimensions")
    layers = terrain_index.get("layers")
    expected = {
        "elevation": 4,
        "slope_deg": 4,
        "roughness": 4,
        "step_height": 4,
        "cost": 1,
        "confidence": 1,
    }
    if not isinstance(layers, dict) or set(layers) != set(expected):
        raise RuntimeError("terrain_2p5d.yaml does not index all six layers")
    map_root = map_dir.resolve()
    for layer_name, bytes_per_cell in expected.items():
        layer_path = (map_dir / str(layers[layer_name])).resolve()
        if layer_path.parent != map_root:
            raise RuntimeError(
                "terrain layer {} escapes the map directory".format(layer_name)
            )
        expected_size = width * height * bytes_per_cell
        if not layer_path.is_file() or layer_path.stat().st_size != expected_size:
            raise RuntimeError(
                "terrain layer {} size mismatch: expected {} bytes, got {}".format(
                    layer_name,
                    expected_size,
                    layer_path.stat().st_size if layer_path.is_file() else "missing",
                )
            )


def validate_finalized_bundle(map_dir, map_name):
    for marker_name in (
            ".finalization_incomplete",
            "filtered_camera_init.pcd.capacity_limited"):
        if (map_dir / marker_name).exists():
            raise RuntimeError("map bundle is not deliverable: {}".format(
                map_dir / marker_name
            ))

    for name in (
            "filtered_camera_init.pcd",
            "raw_camera_init.pcd",
            "traversed_path_map.pcd",
            "public_map.pcd"):
        pcd_header(map_dir / name)
    # A valid scene can have no classified obstacle points. Require complete
    # PCD headers but do not reject a zero-point classified output.
    for name in (
            "terrain_ground_candidates_map.pcd",
            "terrain_ground_map.pcd",
            "terrain_obstacles_map.pcd"):
        pcd_header(map_dir / name, allow_empty=True)
    for yaml_name in ("map_raw.yaml", "map.yaml", "terrain_cost.yaml"):
        validate_map_yaml(map_dir, yaml_name)
    validate_terrain_layers(map_dir)

    metadata_path = map_dir / "map_metadata.yaml"
    with metadata_path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream) or {}
    if metadata.get("map_name") != map_name:
        raise RuntimeError(
            "map_metadata.yaml belongs to {!r}, expected {!r}".format(
                metadata.get("map_name"), map_name
            )
        )


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
    validate_finalized_bundle(map_dir, map_name)
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
