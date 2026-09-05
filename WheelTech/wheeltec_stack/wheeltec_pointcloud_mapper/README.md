# wheeltec_pointcloud_mapper

This node subscribes to FAST-LIO's registered world-frame scan and odometry,
filters outliers, and maintains the stack's single authoritative dynamic/static
decision in a lightweight 3D Bayesian occupancy grid. Scan endpoints increase
occupancy confidence; exact 3D voxel traversal applies negative evidence to
every crossed voxel on selected free-space rays. Fine map points are published
only while their coarse occupancy generation is probable and stable. Demotion
invalidates that complete generation so stale geometry cannot reappear after a
later hit. The node does not publish TF and never feeds points back to FAST-LIO.

The normal mapping launch starts this node automatically. It saves the filtered
PCD every 30 seconds and once more during a normal shutdown, so no save service
is needed in the normal workflow. `/wheeltec_pointcloud_mapper/save_map` remains
available for diagnostics only.

Reset all candidate and confirmed voxels:

```bash
rosservice call /wheeltec_pointcloud_mapper/reset_map
```

Start a named mapping session with:

```bash
roslaunch wheeltec_system_bringup wheeltec_mapping.launch map_name:=wheeltec_map_01
```

The default mapping launch writes to:

```text
~/livox_fastlio/maps/current_mapping/filtered_camera_init.pcd
```

The named launch writes `filtered_camera_init.pcd` below
`~/livox_fastlio/maps/wheeltec_map_01/`. Run `finalize_map.py` once when converting
that PCD into localization and navigation assets.

The default dynamic filter requires at least 25 hit scans spanning four seconds
with a 70 percent hit ratio. Free-space clearing traces every second filtered
point out to 20 m to bound Jetson CPU use. Validation keeps
`/wheeltec/dynamic_points` enabled. A departed object is removed only after the
lidar observes free space through its former position; revisit occluded areas
before finalizing a map.

Patchwork++ and the terrain accumulator consume `/wheeltec/static_scan`. They
only label and accumulate geometry; they do not make a second static-persistence
decision. Final map conversion intersects classified terrain with this node's
saved PCD so a voxel demoted late in the session is removed from every map
artifact.
