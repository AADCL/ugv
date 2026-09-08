# scout_pointcloud_mapper

This node subscribes to FAST-LIO's registered world-frame scan and odometry,
filters outliers, and maintains the stack's single authoritative dynamic/static
decision in a lightweight 3D Bayesian occupancy grid. Scan endpoints increase
occupancy confidence; exact 3D voxel traversal applies negative evidence to
every crossed voxel on selected free-space rays. Fine map points are published
only while their coarse occupancy generation is probable and stable. Demotion
invalidates that complete generation so stale geometry cannot reappear after a
later hit. The node does not publish TF and never feeds points back to FAST-LIO.

The normal mapping launch starts this node automatically. It atomically saves
a crash-recovery checkpoint every 120 seconds and once more during a normal shutdown. The
recommended session supervisor explicitly calls
`/scout_pointcloud_mapper/save_map` before it stops roslaunch, avoiding a
shutdown/finalizer race.

Reset all candidate and confirmed voxels:

```bash
rosservice call /scout_pointcloud_mapper/reset_map
```

Start a named mapping session with:

```bash
rosrun scout_system_bringup scout_mapping_session.py scout_map_01
```

The default mapping launch writes to:

```text
~/livox_fastlio/maps/current_mapping/filtered_camera_init.pcd
```

The named launch writes `filtered_camera_init.pcd` below
`~/livox_fastlio/maps/scout_map_01/`. After the vehicle is parked, one Ctrl+C
makes the supervisor save the map and trajectory, stop roslaunch, and run
`finalize_map.py` automatically.

The default dynamic filter requires at least 12 hit scans spanning two seconds
with a 60 percent hit ratio. Free-space clearing traces every second filtered
point out to 20 m to bound Jetson CPU use. Validation keeps
`/scout/dynamic_points` disabled by default. A departed object is removed only after the
lidar observes free space through its former position; revisit occluded areas
before finalizing a map.

The temporal and fine grids have hard capacities of two and five million
voxels. If either limit drops points, the mapper writes a
`.capacity_limited` marker and returns a failed save response; the finalizer
then refuses to publish the truncated PCD as a formal map.

The latched full-map debug cloud is rebuilt only while it has subscribers, so
normal headless mapping does not traverse and serialize the entire fine grid
every two seconds. Invalid fine generations are still pruned independently
every ten seconds, so disabling the debug serialization does not leak candidate
voxels toward the map capacity limit.

Default mapping does not run Patchwork++ online. Final map conversion rebuilds
terrain directly from this node's final Bayesian PCD, so a voxel demoted late
in the session is removed from every map artifact. The optional online terrain
branch remains diagnostic only.
