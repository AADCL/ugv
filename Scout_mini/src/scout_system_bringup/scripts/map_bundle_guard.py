#!/usr/bin/env python3

"""Reject incomplete Scout map bundles before safety-critical consumers start."""

import math
import os
import sys

import rospy
import yaml


INCOMPLETE_MARKER = ".finalization_incomplete"
CAPACITY_MARKER = "filtered_camera_init.pcd.capacity_limited"


def expanded_path(path):
    return os.path.realpath(os.path.expanduser(os.path.expandvars(path)))


def inside_map_dir(map_dir, path):
    try:
        return os.path.commonpath([map_dir, path]) == map_dir
    except ValueError:
        return False


def required_paths(map_dir, required_files):
    paths = []
    for entry in required_files:
        if not isinstance(entry, str) or not entry.strip():
            raise ValueError("required_files must contain non-empty path strings")
        entry = os.path.expanduser(os.path.expandvars(entry.strip()))
        if not os.path.isabs(entry):
            entry = os.path.join(map_dir, entry)
        path = os.path.realpath(entry)
        if not inside_map_dir(map_dir, path):
            raise ValueError(
                "required file escapes map_dir: {}".format(entry)
            )
        paths.append(path)
    return paths


def inspect_file(path, expected_size=None):
    if not os.path.isfile(path):
        return "required map file is missing: {}".format(path)
    try:
        size = os.path.getsize(path)
    except OSError as error:
        return "cannot inspect required map file {}: {}".format(path, error)
    if size <= 0:
        return "required map file is empty: {}".format(path)
    if expected_size is not None and size != expected_size:
        return (
            "terrain layer size mismatch: {} is {} bytes, expected {}"
            .format(path, size, expected_size)
        )
    return None


def pcd_error(path):
    fields = {}
    payload_offset = None
    try:
        with open(path, "rb") as stream:
            for _ in range(128):
                raw_line = stream.readline(65536)
                if not raw_line:
                    break
                line = raw_line.decode("ascii").strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                fields[parts[0].upper()] = parts[1:]
                if parts[0].upper() == "DATA":
                    payload_offset = stream.tell()
                    break
    except (OSError, UnicodeError) as error:
        return "cannot read PCD header {}: {}".format(path, error)
    required = {
        "VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT",
        "POINTS", "DATA",
    }
    missing = sorted(required.difference(fields))
    if missing:
        return "invalid PCD header {}: missing {}".format(
            path, ", ".join(missing)
        )
    try:
        points = int(fields["POINTS"][0])
        width = int(fields["WIDTH"][0])
        height = int(fields["HEIGHT"][0])
        sizes = [int(value) for value in fields["SIZE"]]
        counts = [int(value) for value in fields["COUNT"]]
    except (IndexError, ValueError) as error:
        return "invalid PCD dimensions/fields {}: {}".format(path, error)
    field_count = len(fields["FIELDS"])
    if (points <= 0 or width <= 0 or height <= 0 or points != width * height or
            field_count == 0 or len(sizes) != field_count or
            len(fields["TYPE"]) != field_count or len(counts) != field_count or
            any(size <= 0 for size in sizes) or any(count <= 0 for count in counts)):
        return "inconsistent or empty PCD metadata: {}".format(path)
    if not fields["DATA"] or fields["DATA"][0].lower() != "binary":
        return "map PCD must use the generated binary encoding: {}".format(path)
    expected_size = payload_offset + points * sum(
        size * count for size, count in zip(sizes, counts)
    )
    try:
        actual_size = os.path.getsize(path)
    except OSError as error:
        return "cannot inspect PCD payload {}: {}".format(path, error)
    if actual_size != expected_size:
        return "PCD payload size mismatch: {} is {} bytes, expected {}".format(
            path, actual_size, expected_size
        )
    return None


def pgm_error(path):
    try:
        with open(path, "rb") as stream:
            tokens = []
            while len(tokens) < 4:
                line = stream.readline(65536)
                if not line:
                    break
                tokens.extend(line.split(b"#", 1)[0].split())
            payload_offset = stream.tell()
        magic = tokens[0]
        width = int(tokens[1])
        height = int(tokens[2])
        maximum = int(tokens[3])
        actual_size = os.path.getsize(path)
    except (OSError, IndexError, ValueError) as error:
        return "cannot parse PGM {}: {}".format(path, error)
    if magic != b"P5" or width <= 0 or height <= 0 or maximum != 255:
        return "invalid PGM dimensions or encoding: {}".format(path)
    expected_size = payload_offset + width * height
    if actual_size != expected_size:
        return "PGM payload size mismatch: {} is {} bytes, expected {}".format(
            path, actual_size, expected_size
        )
    return None


def referenced_path(map_dir, yaml_path, reference):
    if not isinstance(reference, str) or not reference.strip():
        raise ValueError("empty/non-string file reference in {}".format(yaml_path))
    reference = os.path.expanduser(os.path.expandvars(reference.strip()))
    if not os.path.isabs(reference):
        reference = os.path.join(os.path.dirname(yaml_path), reference)
    path = os.path.realpath(reference)
    if not inside_map_dir(map_dir, path):
        raise ValueError(
            "file reference escapes map_dir: {} -> {}".format(yaml_path, path)
        )
    return path


def yaml_reference_errors(map_dir, yaml_paths):
    errors = []
    terrain_layers = {
        "elevation": 4,
        "slope_deg": 4,
        "roughness": 4,
        "step_height": 4,
        "cost": 1,
        "confidence": 1,
    }
    for yaml_path in yaml_paths:
        if not os.path.isfile(yaml_path):
            continue
        try:
            yaml_size = os.path.getsize(yaml_path)
        except OSError as error:
            errors.append("cannot inspect map YAML {}: {}".format(yaml_path, error))
            continue
        if yaml_size <= 0:
            continue
        try:
            with open(yaml_path, "r", encoding="utf-8") as stream:
                document = yaml.safe_load(stream)
        except (OSError, UnicodeError, yaml.YAMLError) as error:
            errors.append("cannot parse map YAML {}: {}".format(yaml_path, error))
            continue
        if not isinstance(document, dict):
            errors.append("map YAML is not a mapping: {}".format(yaml_path))
            continue

        basename = os.path.basename(yaml_path)
        is_terrain_yaml = basename == "terrain_2p5d.yaml"
        is_map_server_yaml = basename != "map_metadata.yaml" and not is_terrain_yaml

        # ROS map_server consumes the image referenced by its YAML, not merely
        # a conventional sibling filename. Every required ordinary YAML in
        # these launches is a map-server YAML and therefore must name one.
        if is_map_server_yaml and "image" not in document:
            errors.append("map YAML has no image reference: {}".format(yaml_path))
        elif is_map_server_yaml:
            try:
                image_path = referenced_path(
                    map_dir, yaml_path, document["image"]
                )
                error = inspect_file(image_path)
                if error:
                    errors.append(error)
                else:
                    error = pgm_error(image_path)
                    if error:
                        errors.append(error)
            except ValueError as error:
                errors.append(str(error))

        if not is_terrain_yaml:
            continue
        if document.get("format") != "scout_terrain_2p5d":
            errors.append("unsupported terrain YAML format: {}".format(yaml_path))
            continue
        width = document.get("width")
        height = document.get("height")
        if (isinstance(width, bool) or not isinstance(width, int) or width <= 0 or
                isinstance(height, bool) or not isinstance(height, int) or
                height <= 0):
            errors.append(
                "terrain YAML has invalid width/height: {}".format(yaml_path)
            )
            continue
        layers = document.get("layers")
        if not isinstance(layers, dict):
            errors.append("terrain YAML has no layers mapping: {}".format(yaml_path))
            continue
        cell_count = width * height
        for layer_name, bytes_per_cell in terrain_layers.items():
            if layer_name not in layers:
                errors.append(
                    "terrain YAML is missing layer '{}': {}".format(
                        layer_name, yaml_path
                    )
                )
                continue
            try:
                layer_path = referenced_path(
                    map_dir, yaml_path, layers[layer_name]
                )
                error = inspect_file(layer_path, cell_count * bytes_per_cell)
                if error:
                    errors.append(error)
            except ValueError as error:
                errors.append(str(error))
    return errors


def startup_errors(map_dir, files, marker_path, capacity_marker_path):
    errors = []
    if not os.path.isdir(map_dir):
        errors.append("map_dir is missing or is not a directory: {}".format(map_dir))
        return errors
    if os.path.lexists(marker_path):
        errors.append(
            "map finalization is incomplete (marker exists): {}".format(
                marker_path
            )
        )
    if os.path.lexists(capacity_marker_path):
        errors.append(
            "mapper capacity limit marker exists; remap before localization or "
            "navigation: {}".format(capacity_marker_path)
        )
    for path in files:
        error = inspect_file(path)
        if error:
            errors.append(error)
        elif path.lower().endswith(".pcd"):
            error = pcd_error(path)
            if error:
                errors.append(error)
    errors.extend(yaml_reference_errors(
        map_dir, [path for path in files if path.lower().endswith(".yaml")]
    ))
    # Close the small race in which finalization starts while files are checked.
    if os.path.lexists(marker_path) and not any(
            marker_path in error for error in errors):
        errors.append(
            "map finalization started during validation: {}".format(marker_path)
        )
    return errors


def main():
    rospy.init_node("scout_map_bundle_guard")

    map_dir_param = rospy.get_param("~map_dir", "")
    required_files = rospy.get_param("~required_files", [])
    check_period_param = rospy.get_param("~check_period", 1.0)

    if not isinstance(map_dir_param, str) or not map_dir_param.strip():
        rospy.logfatal("~map_dir must be a non-empty path")
        return 2
    if isinstance(required_files, str):
        required_files = [required_files]
    if not isinstance(required_files, (list, tuple)) or not required_files:
        rospy.logfatal("~required_files must be a non-empty list")
        return 2
    try:
        check_period = float(check_period_param)
    except (TypeError, ValueError):
        rospy.logfatal("~check_period must be a positive finite number")
        return 2
    if not math.isfinite(check_period) or check_period <= 0.0:
        rospy.logfatal("~check_period must be a positive finite number")
        return 2

    map_dir = expanded_path(map_dir_param.strip())
    marker_path = os.path.join(map_dir, INCOMPLETE_MARKER)
    capacity_marker_path = os.path.join(map_dir, CAPACITY_MARKER)
    try:
        files = required_paths(map_dir, required_files)
    except ValueError as error:
        rospy.logfatal("Invalid map-bundle guard configuration: %s", error)
        return 2

    errors = startup_errors(map_dir, files, marker_path, capacity_marker_path)
    if errors:
        for error in errors:
            rospy.logfatal("Map bundle rejected: %s", error)
        return 1

    rospy.loginfo(
        "Map bundle accepted: %s (%d required files); monitoring %s every %.2f s",
        map_dir,
        len(files),
        marker_path,
        check_period,
    )
    while not rospy.is_shutdown():
        if os.path.lexists(marker_path):
            rospy.logfatal(
                "Map bundle became unsafe: finalization marker appeared: %s",
                marker_path,
            )
            return 1
        if os.path.lexists(capacity_marker_path):
            rospy.logfatal(
                "Map bundle became unsafe: mapper capacity marker appeared: %s",
                capacity_marker_path,
            )
            return 1
        rospy.rostime.wallsleep(check_period)
    return 0


if __name__ == "__main__":
    sys.exit(main())
