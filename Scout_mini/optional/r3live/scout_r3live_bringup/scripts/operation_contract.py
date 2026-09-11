"""Resolve operational launch requirements before starting sensors. No hardware actions."""
import importlib.util
import math
from pathlib import Path
import re

MODES = ('test', 'local', 'mapping', 'localization', 'navigation')


def validate_mode(mode, duration, project_interface):
    if mode not in MODES:
        raise ValueError('Unknown R3LIVE operation mode: '+mode)
    if not math.isfinite(duration) or not (30 <= duration <= 1800 or (duration == 0 and mode != 'test')):
        raise ValueError('Duration must be 30..1800 seconds, or 0 for continuous operational launch')
    if mode != 'test' and not project_interface:
        raise ValueError('Operational launches require project_interface=true')


def prepare_pipeline(package, system_package, mode, map_name, map_dir, start_chassis):
    import roslaunch
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', map_name):
        raise ValueError('Invalid map_name')
    root = Path(map_dir).expanduser().resolve()
    if mode == 'mapping' and root.exists() and (not root.is_dir() or any(root.iterdir())):
        raise ValueError('Mapping refuses an existing nonempty map directory; choose a new map_name: '+str(root))
    arguments = ['operation_mode:='+mode, 'map_name:='+map_name, 'map_dir:='+str(root),
                 'start_chassis:='+str(start_chassis).lower()]
    launch = Path(package)/'launch/operation_pipeline.launch'
    config = roslaunch.config.load_config_default([(str(launch), arguments)], None)
    # Derive map requirements from the actual consumer launches, including all terrain layers.
    guard_file = Path(system_package)/'scripts/map_bundle_guard.py'
    spec = importlib.util.spec_from_file_location('scout_operation_map_guard', str(guard_file))
    guard = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(guard)
    for name, param in config.params.items():
        if name.endswith('/required_files'):
            files = guard.required_paths(str(root), param.value)
            errors = guard.startup_errors(str(root), files, str(root/guard.INCOMPLETE_MARKER),
                                         str(root/guard.CAPACITY_MARKER))
            if errors:
                raise ValueError('Map preflight failed: '+'; '.join(errors))
    return arguments
