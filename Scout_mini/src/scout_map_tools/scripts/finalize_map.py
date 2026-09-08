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
    parser.add_argument(
        "--allow-capacity-limited",
        action="store_true",
        help=(
            "developer-only recovery override: accept a mapper PCD whose "
            "voxel capacity was exceeded"
        ),
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
    parser.add_argument(
        "--legacy-terrain-labels",
        dest="terrain_reclassify",
        action="store_false",
        help=(
            "use the old Patchwork++ label gate instead of rebuilding a "
            "slope-aware floor surface"
        ),
    )
    parser.set_defaults(terrain=True, terrain_reclassify=True)
    args = parser.parse_args()

    home = os.path.expanduser("~")
    workspace = os.path.join(home, "livox_fastlio")
    map_dir = os.path.join(workspace, "maps", args.map_name)
    os.makedirs(map_dir, exist_ok=True)

    bringup_dir = rospack_find("scout_system_bringup")
    tools_dir = rospack_find("scout_map_tools")
    mapper_dir = rospack_find("scout_pointcloud_mapper")

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

    terrain_cost_profile_path = os.path.join(
        tools_dir, "config", "scout_terrain_cost.yaml"
    )

    raw_pcd = os.path.join(map_dir, "raw_camera_init.pcd")
    public_pcd = os.path.join(map_dir, "public_map.pcd")
    source_traversed_path = os.path.join(
        os.path.dirname(os.path.abspath(source_pcd)),
        "traversed_path_map.pcd"
    )
    archived_traversed_path = os.path.join(
        map_dir, "traversed_path_map.pcd"
    )
    terrain_ground_raw = os.path.join(map_dir, "terrain_ground_camera_init.pcd")
    terrain_obstacle_raw = os.path.join(map_dir, "terrain_obstacles_camera_init.pcd")
    terrain_ground_static_raw = os.path.join(
        map_dir, "terrain_ground_static_camera_init.pcd"
    )
    terrain_obstacle_static_raw = os.path.join(
        map_dir, "terrain_obstacles_static_camera_init.pcd"
    )
    terrain_ground_candidates_map = os.path.join(
        map_dir, "terrain_ground_candidates_map.pcd"
    )
    terrain_ground_map = os.path.join(map_dir, "terrain_ground_map.pcd")
    terrain_obstacle_map = os.path.join(map_dir, "terrain_obstacles_map.pcd")
    terrain_2p5d_yaml = os.path.join(map_dir, "terrain_2p5d.yaml")

    if not os.path.isfile(source_pcd):
        raise FileNotFoundError(
            "Filtered source PCD not found: " + source_pcd
        )
    capacity_marker = source_pcd + ".capacity_limited"
    if os.path.isfile(capacity_marker) and not args.allow_capacity_limited:
        raise RuntimeError(
            "Mapper capacity was exceeded; refusing to finalize an incomplete "
            "PCD. Inspect {}, raise dynamic_filter/max_voxels or "
            "map/max_voxels, and remap. The developer-only recovery override "
            "is --allow-capacity-limited.".format(capacity_marker)
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

    traversed_path = None
    if os.path.isfile(source_traversed_path):
        if os.path.abspath(source_traversed_path) == os.path.abspath(
                archived_traversed_path):
            traversed_path = archived_traversed_path
        elif not os.path.isfile(archived_traversed_path) or args.replace_raw:
            shutil.copy2(source_traversed_path, archived_traversed_path)
            traversed_path = archived_traversed_path
            print("[OK] archived traversed path -> " + traversed_path)
        elif not filecmp.cmp(
                source_traversed_path, archived_traversed_path, shallow=False):
            raise RuntimeError(
                "Traversed path differs from the archived evidence; refusing "
                "to mix mapping sessions. Re-run with --replace-raw."
            )
        else:
            traversed_path = archived_traversed_path
            print("[KEEP] existing traversed path matches source -> " + traversed_path)
    else:
        print(
            "[WARN] no traversed_path_map.pcd beside the source PCD; "
            "ground-missing gaps will remain unknown"
        )

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
    try:
        base_link_height = float(
            geometry["base_link_height_above_ground"]
        )
    except (KeyError, TypeError, ValueError):
        raise RuntimeError(
            "base_link_height_above_ground is missing from scout_geometry.yaml"
        )

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

        terrain_reclassify_profile = None
        if args.terrain:
            if args.terrain_reclassify:
                # The final Bayesian point cloud is the single static authority.
                # Conservative PMF seeds are expanded only through locally
                # planar, slope-continuous cells, then all obstacles are rebuilt
                # by height relative to that surface. Patchwork++ history is not
                # accumulated or gated a second time in the default path.
                terrain_reclassify_profile = load_yaml(os.path.join(
                    tools_dir, "config", "terrain_reclassify.yaml"
                ))
                reclassify_params = copy.deepcopy(terrain_reclassify_profile)
                reclassify_params.pop("ground_reference_radius_m", None)
                reclassify_params.update({
                    "input_pcd": public_pcd,
                    "output_candidate_pcd": terrain_ground_candidates_map,
                    "output_ground_pcd": terrain_ground_map,
                    "output_obstacle_pcd": terrain_obstacle_map,
                    # base_link starts at z=0 in odom; the transformed floor is
                    # therefore one measured base height below the map origin.
                    # Do not substitute the rigid sensor Z offset here.
                    "seed_ground_z": -base_link_height,
                })
                run(
                    ["rosrun", "scout_map_tools", "terrain_reclassify_node"]
                    + private_args(reclassify_params)
                )
            else:
                for source in (terrain_ground_raw, terrain_obstacle_raw):
                    if not os.path.isfile(source):
                        raise FileNotFoundError(
                            "Terrain classified PCD not found: " + source
                        )
                for source, gated in (
                    (terrain_ground_raw, terrain_ground_static_raw),
                    (terrain_obstacle_raw, terrain_obstacle_static_raw),
                ):
                    run(
                        ["rosrun", "scout_map_tools", "pcd_static_gate_node"]
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
                        ["rosrun", "scout_map_tools", "pcd_transform_node"]
                        + private_args(classified_transform)
                    )

            terrain_2p5d_dir = rospack_find("scout_2p5d_navigation")
            terrain_2p5d_profile = load_yaml(os.path.join(
                terrain_2p5d_dir, "config", "terrain_builder.yaml"
            ))
            terrain_2p5d_profile.update({
                "ground_pcd": terrain_ground_map,
                "obstacle_pcd": terrain_obstacle_map,
                "output_yaml": terrain_2p5d_yaml,
                "frame_id": "map",
            })
            if args.terrain_reclassify:
                terrain_2p5d_profile.update({
                    "obstacle_ground_search_radius_m": (
                        terrain_reclassify_profile[
                            "ground_reference_radius_m"
                        ]
                    ),
                    "obstacle_min_relative_height_m": (
                        terrain_reclassify_profile[
                            "min_obstacle_relative_height_m"
                        ]
                    ),
                    "obstacle_max_relative_height_m": (
                        terrain_reclassify_profile[
                            "max_obstacle_relative_height_m"
                        ]
                    ),
                })
            run(
                ["rosrun", "scout_2p5d_navigation", "terrain_map_builder_node"]
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
            params = copy.deepcopy(cfg)
            if args.terrain:
                params["classification_mode"] = True
                params["ground_pcd"] = terrain_ground_map
                params["obstacle_pcd"] = terrain_obstacle_map
                if args.terrain_reclassify:
                    params.update({
                        "classified_obstacle/ground_search_radius_m": (
                            terrain_reclassify_profile[
                                "ground_reference_radius_m"
                            ]
                        ),
                        "classified_obstacle/min_relative_height_m": (
                            terrain_reclassify_profile[
                                "min_obstacle_relative_height_m"
                            ]
                        ),
                        "classified_obstacle/max_relative_height_m": (
                            terrain_reclassify_profile[
                                "max_obstacle_relative_height_m"
                            ]
                        ),
                    })
            else:
                params["input_pcd"] = public_pcd

            # Only the normal occupancy maps consume physically traversed free
            # space. The terrain-cost map keeps missing slope cells unknown.
            if traversed_path is not None and name in ("raw", "nav"):
                params["free_evidence_pcd"] = traversed_path

            # Record the effective parameters after terrain reclassification
            # overrides, without embedding machine-specific absolute PCD paths.
            effective_profile = copy.deepcopy(params)
            for path_key in (
                    "ground_pcd", "obstacle_pcd", "input_pcd",
                    "free_evidence_pcd"):
                effective_profile.pop(path_key, None)
            profile_snapshot[name] = effective_profile

            params["output_pgm"] = output_pgm
            params["output_yaml"] = output_yaml

            run(
                [
                    "rosrun", "scout_map_tools", "pcd_to_pgm_node",
                    "__name:=scout_pcd_to_pgm_" + name,
                ]
                + private_args(params)
            )

        metadata = {
            "map_name": args.map_name,
            "created_at": datetime.datetime.now().isoformat(timespec="seconds"),
            "frames": {
                "raw_pcd": "camera_init",
                "public_map": "map",
                "traversed_path": "odom (same coordinates as saved map)",
            },
            "files": {
                "raw_pcd": "raw_camera_init.pcd",
                "public_pcd": "public_map.pcd",
                "traversed_path_pcd": (
                    "traversed_path_map.pcd"
                    if traversed_path is not None else None
                ),
                "nav_map_yaml": "map_raw.yaml",
                "compatibility_map_yaml": "map.yaml",
                "terrain_cost_map_yaml": "terrain_cost.yaml" if args.terrain else None,
                "terrain_ground_pcd": "terrain_ground_map.pcd" if args.terrain else None,
                "terrain_obstacle_pcd": "terrain_obstacles_map.pcd" if args.terrain else None,
                "terrain_ground_static_source": (
                    "terrain_ground_static_camera_init.pcd"
                    if args.terrain and not args.terrain_reclassify else None
                ),
                "terrain_obstacle_static_source": (
                    "terrain_obstacles_static_camera_init.pcd"
                    if args.terrain and not args.terrain_reclassify else None
                ),
                "terrain_2p5d_yaml": "terrain_2p5d.yaml" if args.terrain else None,
                "terrain_ground_candidates": (
                    "terrain_ground_candidates_map.pcd"
                    if args.terrain and args.terrain_reclassify else None
                ),
            },
            "terrain_classification": args.terrain,
            "terrain_reclassification": {
                "enabled": True,
                "profile": terrain_reclassify_profile,
                "policy": (
                    "conservative PMF seeds plus robust local-plane growth; "
                    "Bayesian static points reclassified by relative height"
                ),
            } if args.terrain and args.terrain_reclassify else {
                "enabled": False,
                "policy": "legacy exact static voxel gate",
            } if args.terrain else None,
            "static_authority": {
                "pcd": (
                    "public_map.pcd" if args.terrain_reclassify
                    else "raw_camera_init.pcd"
                ),
                "voxel_size": (
                    static_gate_voxel_size
                    if not args.terrain_reclassify else None
                ),
                "policy": (
                    "final Bayesian map is the direct terrain source; no "
                    "second accumulated-label gate"
                    if args.terrain_reclassify else
                    "classified terrain voxel must exist in final Bayesian map"
                ),
            } if args.terrain else None,
            "free_space_authority": {
                "pcd": (
                    "traversed_path_map.pcd"
                    if traversed_path is not None else None
                ),
                "policy": (
                    "normal occupancy maps union ground evidence with the "
                    "recorded base-link swept corridor; classified obstacles "
                    "always override free evidence"
                    if traversed_path is not None else
                    "no recorded traversal evidence; only observed ground is free"
                ),
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
        if traversed_path is not None:
            print("  free path  : " + traversed_path)
        print("  nav map    : " + os.path.join(map_dir, "map_raw.yaml"))
        print("  compat map : " + os.path.join(map_dir, "map.yaml"))
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
