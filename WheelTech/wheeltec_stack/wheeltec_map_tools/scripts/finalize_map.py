#!/usr/bin/env python3
import argparse
import copy
import datetime
import filecmp
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
    terrain_group = parser.add_mutually_exclusive_group()
    terrain_group.add_argument(
        "--terrain",
        dest="terrain",
        action="store_true",
        help="compatibility option; complete 2.5D output is already the default"
    )
    terrain_group.add_argument(
        "--legacy-2d-only",
        dest="terrain",
        action="store_false",
        help="developer-only: skip classified terrain and 2.5D map generation"
    )
    parser.set_defaults(terrain=True)
    args = parser.parse_args()

    home = os.path.expanduser("~")
    workspace = os.path.join(home, "livox_fastlio")
    map_dir = os.path.join(workspace, "maps", args.map_name)
    os.makedirs(map_dir, exist_ok=True)

    bringup_dir = rospack_find("wheeltec_system_bringup")
    tools_dir = rospack_find("wheeltec_map_tools")
    mapper_dir = rospack_find("wheeltec_pointcloud_mapper")

    source_pcd = args.source or os.path.join(
        map_dir, "filtered_camera_init.pcd"
    )

    geometry_yaml = os.path.join(
        bringup_dir, "config", "wheeltec_geometry.yaml"
    )

    raw_profile_path = os.path.join(
        tools_dir, "config", "wheeltec_raw.yaml"
    )

    nav_profile_path = os.path.join(
        tools_dir, "config", "wheeltec_nav.yaml"
    )

    terrain_cost_profile_path = os.path.join(
        tools_dir, "config", "wheeltec_terrain_cost.yaml"
    )

    raw_pcd = os.path.join(map_dir, "raw_camera_init.pcd")
    public_pcd = os.path.join(map_dir, "public_map.pcd")
    terrain_ground_raw = os.path.join(map_dir, "terrain_ground_camera_init.pcd")
    terrain_obstacle_raw = os.path.join(map_dir, "terrain_obstacles_camera_init.pcd")
    terrain_ground_static_raw = os.path.join(
        map_dir, "terrain_ground_static_camera_init.pcd"
    )
    terrain_obstacle_static_raw = os.path.join(
        map_dir, "terrain_obstacles_static_camera_init.pcd"
    )
    terrain_ground_map = os.path.join(map_dir, "terrain_ground_map.pcd")
    terrain_obstacle_map = os.path.join(map_dir, "terrain_obstacles_map.pcd")
    terrain_2p5d_yaml = os.path.join(map_dir, "terrain_2p5d.yaml")

    if not os.path.isfile(source_pcd):
        raise FileNotFoundError(
            "Filtered source PCD not found: " + source_pcd
        )

    if not os.path.isfile(raw_pcd) or args.replace_raw:
        shutil.copy2(source_pcd, raw_pcd)
        print("[OK] archived raw PCD -> " + raw_pcd)
    elif not filecmp.cmp(source_pcd, raw_pcd, shallow=False):
        raise RuntimeError(
            "Filtered PCD differs from the archived raw PCD; refusing to mix "
            "mapping sessions. Re-run with --replace-raw to use the current map."
        )
    else:
        print("[KEEP] existing raw PCD is identical to filtered source -> " + raw_pcd)

    mapper_profile = load_yaml(os.path.join(
        mapper_dir, "config", "mapper.yaml"
    ))
    try:
        static_gate_voxel_size = float(
            mapper_profile["map"]["voxel_size"]
        )
    except (KeyError, TypeError, ValueError):
        raise RuntimeError(
            "map/voxel_size is missing from pointcloud mapper config"
        )

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
            ["rosrun", "wheeltec_map_tools", "pcd_transform_node"]
            + private_args(transform_params)
        )

        if args.terrain:
            for source, gated in (
                (terrain_ground_raw, terrain_ground_static_raw),
                (terrain_obstacle_raw, terrain_obstacle_static_raw),
            ):
                if not os.path.isfile(source):
                    raise FileNotFoundError("Terrain classified PCD not found: " + source)
                run(
                    ["rosrun", "wheeltec_map_tools", "pcd_static_gate_node"]
                    + private_args({
                        "authority_pcd": raw_pcd,
                        "input_pcd": source,
                        "output_pcd": gated,
                        "voxel_size": static_gate_voxel_size,
                    })
                )

            for source, output in (
                (terrain_ground_static_raw, terrain_ground_map),
                (terrain_obstacle_static_raw, terrain_obstacle_map),
            ):
                classified_transform = copy.deepcopy(transform_params)
                classified_transform["input_pcd"] = source
                classified_transform["output_pcd"] = output
                run(
                    ["rosrun", "wheeltec_map_tools", "pcd_transform_node"]
                    + private_args(classified_transform)
                )

            terrain_2p5d_dir = rospack_find("wheeltec_2p5d_navigation")
            terrain_2p5d_profile = load_yaml(os.path.join(
                terrain_2p5d_dir, "config", "terrain_builder.yaml"
            ))
            terrain_2p5d_profile.update({
                "ground_pcd": terrain_ground_map,
                "obstacle_pcd": terrain_obstacle_map,
                "output_yaml": terrain_2p5d_yaml,
                "frame_id": "map",
            })
            run(
                ["rosrun", "wheeltec_2p5d_navigation", "terrain_map_builder_node"]
                + private_args(terrain_2p5d_profile)
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

        if args.terrain:
            profiles.append(
                (
                    "terrain_cost",
                    terrain_cost_profile_path,
                    os.path.join(map_dir, "terrain_cost.pgm"),
                    os.path.join(map_dir, "terrain_cost.yaml"),
                )
            )

        profile_snapshot = {}

        for name, profile_path, output_pgm, output_yaml in profiles:
            cfg = load_yaml(profile_path)
            profile_snapshot[name] = copy.deepcopy(cfg)

            params = copy.deepcopy(cfg)
            if args.terrain:
                params["classification_mode"] = True
                params["ground_pcd"] = terrain_ground_map
                params["obstacle_pcd"] = terrain_obstacle_map
            else:
                params["input_pcd"] = public_pcd
            params["output_pgm"] = output_pgm
            params["output_yaml"] = output_yaml

            run(
                ["rosrun", "wheeltec_map_tools", "pcd_to_pgm_node"]
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
                "terrain_cost_map_yaml": "terrain_cost.yaml" if args.terrain else None,
                "terrain_ground_pcd": "terrain_ground_map.pcd" if args.terrain else None,
                "terrain_obstacle_pcd": "terrain_obstacles_map.pcd" if args.terrain else None,
                "terrain_ground_static_source": (
                    "terrain_ground_static_camera_init.pcd" if args.terrain else None
                ),
                "terrain_obstacle_static_source": (
                    "terrain_obstacles_static_camera_init.pcd" if args.terrain else None
                ),
                "terrain_2p5d_yaml": "terrain_2p5d.yaml" if args.terrain else None,
            },
            "terrain_classification": args.terrain,
            "static_authority": {
                "pcd": "raw_camera_init.pcd",
                "voxel_size": static_gate_voxel_size,
                "policy": "classified terrain voxel must exist in final Bayesian map",
            } if args.terrain else None,
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
        if args.terrain:
            print("  slope costs: " + os.path.join(map_dir, "terrain_cost.yaml"))
            print("  2.5D map   : " + terrain_2p5d_yaml)
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
