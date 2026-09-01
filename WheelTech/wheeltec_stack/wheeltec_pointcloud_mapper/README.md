# wheeltec_pointcloud_mapper

This node subscribes to FAST-LIO's registered world-frame scan and odometry,
filters outliers, and maintains a lightweight 3D Bayesian occupancy grid. Scan
endpoints increase occupancy confidence; sampled free-space rays decrease it.
Fine map points are published only while their coarse occupancy voxel is both
probable and stable, so departed people and robots can be removed. It does not
publish TF and never feeds points back to FAST-LIO.

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

The default dynamic filter requires at least eight hit scans spanning two
seconds. Free-space clearing traces one quarter of the filtered points out to
20 m to limit Jetson CPU use. A departed object is removed only after the lidar
observes free space through its former position; revisit occluded areas before
finalizing a map.
