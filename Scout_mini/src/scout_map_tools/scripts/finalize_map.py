#!/usr/bin/env python3
import argparse
import copy
import datetime
import filecmp
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

import yaml


MAP_NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
INCOMPLETE_MARKER = ".finalization_incomplete"


def atomic_write_yaml(path, payload):
    """Replace a small YAML file without exposing a partially written file."""
    directory = os.path.dirname(path)
    fd, temporary = tempfile.mkstemp(
        prefix=os.path.basename(path) + ".", suffix=".tmp", dir=directory
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump(payload, stream, allow_unicode=True, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        try:
            directory_fd = os.open(directory, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except OSError:
            # Directory fsync is supported on the target Linux host. The
            # atomic os.replace above remains valid elsewhere.
            pass
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def create_incomplete_marker(map_dir):
    marker = os.path.join(map_dir, INCOMPLETE_MARKER)
    atomic_write_yaml(marker, {
        "state": "incomplete",
        "started_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "reason": (
            "Map finalization is in progress or failed. Localization and "
            "navigation must not consume this directory until the marker is "
            "removed by a successful finalization."
        ),
    })
    return marker


def remove_incomplete_marker(marker):
    os.unlink(marker)
    try:
        directory_fd = os.open(os.path.dirname(marker), os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except OSError:
        pass


def pcd_header(path, allow_empty=False):
    """Validate the ASCII header of an ASCII/binary PCD and return POINTS."""
    fields = {}
    payload_offset = None
    with open(path, "rb") as stream:
        for _ in range(128):
            raw_line = stream.readline(65536)
            if not raw_line:
                break
            try:
                line = raw_line.decode("ascii").strip()
            except UnicodeDecodeError as error:
                raise RuntimeError("Invalid PCD header encoding: " + path) from error
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
            "Invalid PCD header {}: missing {}".format(path, ", ".join(missing))
        )
    try:
        points = int(fields["POINTS"][0])
        width = int(fields["WIDTH"][0])
        height = int(fields["HEIGHT"][0])
    except (IndexError, ValueError) as error:
        raise RuntimeError("Invalid PCD dimensions: " + path) from error
    if points < 0 or width < 0 or height <= 0 or points != width * height:
        raise RuntimeError("Inconsistent PCD dimensions: " + path)
    if not allow_empty and points == 0:
        raise RuntimeError("PCD contains no points: " + path)
    if not fields["DATA"] or fields["DATA"][0].lower() not in (
            "ascii", "binary", "binary_compressed"):
        raise RuntimeError("Unsupported PCD DATA encoding: " + path)
    encoding = fields["DATA"][0].lower()
    try:
        sizes = [int(value) for value in fields["SIZE"]]
        counts = [int(value) for value in fields["COUNT"]]
    except ValueError as error:
        raise RuntimeError("Invalid PCD SIZE/COUNT values: " + path) from error
    field_count = len(fields["FIELDS"])
    if (field_count == 0 or len(sizes) != field_count or
            len(fields["TYPE"]) != field_count or len(counts) != field_count or
            any(size <= 0 for size in sizes) or any(count <= 0 for count in counts)):
        raise RuntimeError("Inconsistent PCD field metadata: " + path)
    if encoding == "binary":
        expected_size = payload_offset + points * sum(
            size * count for size, count in zip(sizes, counts)
        )
        if os.path.getsize(path) != expected_size:
            raise RuntimeError(
                "PCD binary payload size mismatch: expected {} bytes: {}".format(
                    expected_size, path
                )
            )
    elif points > 0 and os.path.getsize(path) <= payload_offset:
        raise RuntimeError("PCD payload is missing: " + path)
    return points


def validate_pgm(path):
    with open(path, "rb") as stream:
        tokens = []
        while len(tokens) < 4:
            line = stream.readline(65536)
            if not line:
                break
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())
        payload_offset = stream.tell()
    try:
        magic = tokens[0]
        width = int(tokens[1])
        height = int(tokens[2])
        maximum = int(tokens[3])
    except (IndexError, ValueError) as error:
        raise RuntimeError("Invalid PGM header: " + path) from error
    if magic != b"P5" or width <= 0 or height <= 0 or maximum != 255:
        raise RuntimeError("Invalid PGM dimensions or encoding: " + path)
    expected_size = payload_offset + width * height
    if os.path.getsize(path) != expected_size:
        raise RuntimeError(
            "PGM payload size mismatch: expected {} bytes: {}".format(
                expected_size, path
            )
        )


def validate_map_yaml(map_dir, yaml_name):
    yaml_path = os.path.join(map_dir, yaml_name)
    config = load_yaml(yaml_path)
    image = config.get("image")
    if not isinstance(image, str) or not image:
        raise RuntimeError("Map YAML does not name an image: " + yaml_path)
    image_path = os.path.realpath(os.path.join(map_dir, image))
    if os.path.dirname(image_path) != os.path.realpath(map_dir):
        raise RuntimeError("Map YAML image escapes map directory: " + yaml_path)
    if not os.path.isfile(image_path) or os.path.getsize(image_path) == 0:
        raise RuntimeError("Map image is missing or empty: " + image_path)
    validate_pgm(image_path)


def validate_terrain_layers(map_dir):
    terrain_yaml = os.path.join(map_dir, "terrain_2p5d.yaml")
    terrain = load_yaml(terrain_yaml)
    if terrain.get("format") != "scout_terrain_2p5d":
        raise RuntimeError("terrain_2p5d.yaml has an unsupported format")
    try:
        width = int(terrain["width"])
        height = int(terrain["height"])
        resolution = float(terrain["resolution"])
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError("terrain_2p5d.yaml has invalid dimensions") from error
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise RuntimeError("terrain_2p5d.yaml has invalid dimensions")
    layers = terrain.get("layers")
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
    map_root = os.path.realpath(map_dir)
    for name, bytes_per_cell in expected.items():
        layer_path = os.path.realpath(os.path.join(map_dir, str(layers[name])))
        if os.path.dirname(layer_path) != map_root:
            raise RuntimeError("Terrain layer escapes map directory: " + name)
        expected_size = width * height * bytes_per_cell
        if not os.path.isfile(layer_path) or os.path.getsize(layer_path) != expected_size:
            raise RuntimeError(
                "Terrain layer {} size mismatch: expected {} bytes".format(
                    layer_path, expected_size
                )
            )


def validate_final_bundle(map_dir, terrain, terrain_reclassify, traversed_path):
    for name in ("raw_camera_init.pcd", "public_map.pcd"):
        pcd_header(os.path.join(map_dir, name))
    if traversed_path is not None:
        pcd_header(os.path.join(map_dir, "traversed_path_map.pcd"))
    map_yamls = ["map_raw.yaml", "map.yaml"]
    if terrain:
        # Empty classified clouds are valid outputs. Their PCD headers must
        # still be complete so stale or truncated files cannot pass delivery.
        classified_pcds = [
            "terrain_ground_map.pcd",
            "terrain_obstacles_map.pcd",
        ]
        if terrain_reclassify:
            classified_pcds.append("terrain_ground_candidates_map.pcd")
        else:
            classified_pcds.extend((
                "terrain_ground_camera_init.pcd",
                "terrain_obstacles_camera_init.pcd",
                "terrain_ground_static_camera_init.pcd",
                "terrain_obstacles_static_camera_init.pcd",
            ))
        for name in classified_pcds:
            pcd_header(os.path.join(map_dir, name), allow_empty=True)
        map_yamls.append("terrain_cost.yaml")
        validate_terrain_layers(map_dir)
    for yaml_name in map_yamls:
        validate_map_yaml(map_dir, yaml_name)
    metadata_path = os.path.join(map_dir, "map_metadata.yaml")
    metadata = load_yaml(metadata_path)
    if not isinstance(metadata, dict) or not metadata.get("map_name"):
        raise RuntimeError("Invalid map_metadata.yaml: " + metadata_path)


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

    if not MAP_NAME_PATTERN.fullmatch(args.map_name) or args.map_name in (
            ".", ".."):
        parser.error(
            "map_name must contain only letters, digits, dot, underscore or "
            "hyphen, start with a letter/digit, and not be '.' or '..'"
        )

    home = os.path.expanduser("~")
    workspace = os.path.join(home, "livox_fastlio")
    map_dir = os.path.join(workspace, "maps", args.map_name)
    os.makedirs(map_dir, exist_ok=True)
    incomplete_marker = create_incomplete_marker(map_dir)

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

        validate_final_bundle(
            map_dir, args.terrain, args.terrain_reclassify, traversed_path
        )
        remove_incomplete_marker(incomplete_marker)

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
