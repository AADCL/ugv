#!/usr/bin/env python3
import argparse
import copy
import datetime
import os
import shutil
import subprocess
import sys
import time

import yaml


def run(cmd):
    print("[RUN] " + " ".join(cmd))
    subprocess.run(cmd, check=True)


def rospack_find(pkg):
    return subprocess.check_output(
        ["rospack", "find", pkg],
        text=True
    ).strip()


def master_online():
    try:
        import rosgraph
        return rosgraph.is_master_online()
    except Exception:
        return False


def wait_master(timeout_sec=8.0):
    start = time.time()
    while time.time() - start < timeout_sec:
        if master_online():
            return True
        time.sleep(0.2)
    return False


def private_args(params):
    out = []
    for key, value in params.items():
        if isinstance(value, bool):
            value = "true" if value else "false"
        out.append("_{}:={}".format(key, value))
    return out


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if data is not None else {}


def main():
    parser = argparse.ArgumentParser(
        description="Archive the filtered mapper PCD and build 3D/2D maps"
    )
    parser.add_argument("map_name")
    parser.add_argument(
        "--source",
        default=None,
        help=(
            "filtered PCD path; defaults to "
            "~/livox_fastlio/maps/<map_name>/filtered_camera_init.pcd"
        )
    )
    parser.add_argument(
        "--replace-raw",
        action="store_true",
        help="replace existing raw_camera_init.pcd from --source"
    )
    args = parser.parse_args()

    home = os.path.expanduser("~")
    workspace = os.path.join(home, "livox_fastlio")
    map_dir = os.path.join(workspace, "maps", args.map_name)
    os.makedirs(map_dir, exist_ok=True)

    bringup_dir = rospack_find("scout_system_bringup")
    tools_dir = rospack_find("scout_map_tools")

    source_pcd = args.source or os.path.join(
        map_dir, "filtered_camera_init.pcd"
    )

    geometry_yaml = os.path.join(
        bringup_dir, "config", "scout_geometry.yaml"
    )

    raw_profile_path = os.path.join(
        tools_dir, "config", "scout_raw.yaml"
    )

    nav_profile_path = os.path.join(
        tools_dir, "config", "scout_nav.yaml"
    )

    raw_pcd = os.path.join(map_dir, "raw_camera_init.pcd")
    public_pcd = os.path.join(map_dir, "public_map.pcd")

    if not os.path.isfile(raw_pcd) or args.replace_raw:
        if not os.path.isfile(source_pcd):
            raise FileNotFoundError(
                "Filtered source PCD not found: " + source_pcd
            )
        shutil.copy2(source_pcd, raw_pcd)
        print("[OK] archived raw PCD -> " + raw_pcd)
    else:
        print("[KEEP] existing raw PCD -> " + raw_pcd)
        print("       use --replace-raw only when intentionally replacing it")

    geometry = load_yaml(geometry_yaml)
    tf_cfg = geometry["odom_to_camera_init"]

    started_master = None
    if not master_online():
        print("[INFO] ROS master is offline; starting temporary roscore")
        started_master = subprocess.Popen(
            ["roscore"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        if not wait_master():
            started_master.terminate()
            raise RuntimeError("temporary roscore failed to start")

    try:
        transform_params = {
            "input_pcd": raw_pcd,
            "output_pcd": public_pcd,
            "x": tf_cfg["x"],
            "y": tf_cfg["y"],
            "z": tf_cfg["z"],
            "roll_deg": tf_cfg["roll_deg"],
            "pitch_deg": tf_cfg["pitch_deg"],
            "yaw_deg": tf_cfg["yaw_deg"],
        }

        run(
            ["rosrun", "scout_map_tools", "pcd_transform_node"]
            + private_args(transform_params)
        )

        profiles = [
            (
                "raw",
                raw_profile_path,
                os.path.join(map_dir, "map_raw.pgm"),
                os.path.join(map_dir, "map_raw.yaml"),
            ),
            (
                "nav",
                nav_profile_path,
                os.path.join(map_dir, "map.pgm"),
                os.path.join(map_dir, "map.yaml"),
            ),
        ]

        profile_snapshot = {}

        for name, profile_path, output_pgm, output_yaml in profiles:
            cfg = load_yaml(profile_path)
            profile_snapshot[name] = copy.deepcopy(cfg)

            params = copy.deepcopy(cfg)
            params["input_pcd"] = public_pcd
            params["output_pgm"] = output_pgm
            params["output_yaml"] = output_yaml

            run(
                ["rosrun", "scout_map_tools", "pcd_to_pgm_node"]
                + private_args(params)
            )

        metadata = {
            "map_name": args.map_name,
            "created_at": datetime.datetime.now().isoformat(timespec="seconds"),
            "frames": {
                "raw_pcd": "camera_init",
                "public_map": "map",
            },
            "files": {
                "raw_pcd": "raw_camera_init.pcd",
                "public_pcd": "public_map.pcd",
                "raw_map_yaml": "map_raw.yaml",
                "nav_map_yaml": "map.yaml",
            },
            "geometry_snapshot": geometry,
            "map_generation": profile_snapshot,
        }

        metadata_path = os.path.join(map_dir, "map_metadata.yaml")
        with open(metadata_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(
                metadata,
                f,
                allow_unicode=True,
                sort_keys=False
            )

        print("\n[DONE] map finalized")
        print("  map dir    : " + map_dir)
        print("  raw PCD    : " + raw_pcd)
        print("  public PCD : " + public_pcd)
        print("  nav map    : " + os.path.join(map_dir, "map.yaml"))
        print("  metadata   : " + metadata_path)

    finally:
        if started_master is not None:
            started_master.terminate()
            started_master.wait(timeout=5)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("[ERROR] {}".format(e), file=sys.stderr)
        sys.exit(1)
