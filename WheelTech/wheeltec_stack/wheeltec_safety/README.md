# WheelTech velocity safety gate

In the exploration entry, this package is the only software path from autonomous
navigation commands to the exploration chassis command topic:

```text
move_base -> /nav_cmd_vel -> wheeltec_safety -> /wheeltec_driver/cmd_vel
```

Those two velocity topic names and the `/wheeltec_safety` node name are fixed
in code and cannot be overridden by launch arguments. Every input velocity
message must carry the ROS caller ID `/move_base`; a missing or different
caller immediately latches the gate.

The gate starts locked and continuously publishes zero velocity until all
required inputs are fresh and the launch-owned exploration supervisor requests
one startup authorization. Manual authorization remains available in diagnostic
mode (auto_start:=false):

```bash
rosservice call /wheeltec_safety/arm
```

Immediate stop and fault reset are separate operations:

```bash
rosservice call /wheeltec_safety/stop
rosservice call /wheeltec_safety/reset
```

`reset` never rearms the vehicle. Inspect `/wheeltec_safety/status`, clear the
cause, reset, and explicitly arm again.

Arming clears every cached command and local plan. Motion is held at zero until
a new goal ID from `/move_base/goal`, a matching `/move_base/status`
confirmation, a plan for that goal generation, and a post-goal velocity
command have all arrived. Replacing active goal A with B starts a new
generation immediately, before the status array catches up. The handshake has
a 12.0 second startup deadline (5 seconds global planning + 5 seconds local
control patience + 2 seconds status margin) that repeated status or zero commands cannot
reset. Output stays zero throughout this startup wait. Once the handshake
completes, the existing 0.20 second command timeout and 0.35 second local-plan
timeout still apply. Swept-region checks use vectorized geometry equivalent
to the scalar predicate; snapshot invalidation and obstacle checks remain.

The stop region checks both classified obstacle points and a raw near-field
cloud. The raw cloud is transformed once, cropped to the bounded swept region,
and reduced to 5 cm XY cells. A cell is blocked when its local vertical span is
at least 8 cm, retaining thin upright objects while rejecting locally planar
floors and ramps. The full cloud is never retained or reprocessed at 20 Hz.

While the gate is locked it repeats a cancel-all request to `move_base` once per
second. A safety latch parks exploration while mapping and diagnostics remain
alive; the supervisor never automatically resets or rearms after a stop.
The pause/resume services support temporary map-data waits without clearing a
fault latch or granting startup authorization. Resume requires healthy inputs,
an already armed gate and no live goal, and clears cached motion inputs.
The internal launch/include/cmd_vel_safety.launch.xml is included by the single
exploration entry; ordinary navigation retains its existing command path.

Identical command and perception refreshes update freshness without discarding
the collision result. Changed content is checked again within the same output
cycle, at most three attempts and 80% of the timer period. Real obstacles and
stale motion commands still stop immediately; sustained contention also stops.
The independent `latched_reason` retains the first cause. Reverse arcs are
rejected as a whole; the deadband no longer hides TEB's 0.01 m/s reverse range.

This ROS gate does not replace an emergency-stop switch or a command timeout in
the lower motor controller. Those remain necessary because no ROS process can
stop the chassis after a command if the entire computer or serial link fails.
