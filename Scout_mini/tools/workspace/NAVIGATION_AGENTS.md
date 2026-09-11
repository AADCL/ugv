# Scout deployed workspace instructions

- ROS Noetic, Ubuntu 20.04; workspace `/home/nvidia/livox_fastlio`.
- Read the three current manuals in `docs/` for architecture and TF invariants.
- Canonical code/manuals: `/home/nvidia/github_upload/ugv/Scout_mini`.
- Directory guide: `README_Scout_mini.md`; three current manuals in `docs/`.
- Preserve navigation parameters, chassis geometry, unique TF owners and maps.
- FAST-LIO input unchanged; native PCD saving disabled.
- R3LIVE, legacy camera and calibration physically live in `optional/` with
  independent src/build/devel. Keep their old home-path compatibility symlinks.
- Build using old workspace paths and `catkin_make -j1` to match CMake caches.
- Selected extrinsic/backups: `optional/r3live_ws/config/accepted_20260911`.
- R3LIVE's independent entry defaults to project_interface=true, publishing the
  existing public odometry/cloud interfaces and camera_init -> body TF. FAST-LIO
  and R3LIVE interface modes are mutually exclusive. Native R3LIVE TF is isolated
  on /r3live/tf_raw; do not relay it back or start a second TF/pose/cloud adapter.
- /fastlio_odom is a legacy pose-only name; under R3LIVE it carries transformed
  R3LIVE data. The test entry starts no chassis/NDT/navigation. The four explicit
  scout_r3live_{local,mapping,localization,navigation}.launch operational entries
  in scout_system_bringup run continuously and add the selected pipeline after
  local warmup. Navigation includes localization; never stack a second launch.
- Operational launches reuse unchanged chassis/NDT/navigation configurations and
  start no wheel fusion. Mapping refuses nonempty directories; stop and wait for
  mapper saving, then finalize separately. Preserve full map bundle guards.
- User authorized deletion of historical bags and derivatives on 2026-09-11;
  do not treat this as authorization to delete future recordings.
- CCS `/home/nvidia/ccs_edge_ws` must not be changed.
- Do not start hardware or send velocity commands during directory maintenance.
- Preserve upstream patches and user changes. Synchronize three detailed manuals
  when implementing changes; WheelTech is outside Scout-only maintenance scope.
