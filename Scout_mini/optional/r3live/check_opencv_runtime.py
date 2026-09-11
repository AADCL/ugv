#!/usr/bin/env python3
"""Fail installation when camera/estimator/plugin dependencies mix OpenCV ABIs."""
import os
from pathlib import Path
import re
import subprocess
import sys

workspace = Path(sys.argv[1]).resolve()
env = dict(os.environ)
env['LD_LIBRARY_PATH'] = str(workspace / 'devel/lib') + ':' + env.get('LD_LIBRARY_PATH', '')
for relative in ('librealsense2_camera.so', 'libcv_bridge.so',
                 'libcompressed_image_transport.so',
                 'libcompressed_depth_image_transport.so',
                 'libtheora_image_transport.so', 'r3live/r3live_mapping'):
    library = workspace / 'devel/lib' / relative
    output = subprocess.check_output(['ldd', str(library)], env=env, universal_newlines=True)
    versions = set(re.findall(r'libopencv_\w+\.so\.(\d+\.\d+)', output))
    if versions != {'4.5'} or 'not found' in output:
        raise SystemExit('Invalid dependencies for {}:\n{}'.format(library, output))
    print('OPENCV_ABI_OK {} {}'.format(relative, sorted(versions)))
