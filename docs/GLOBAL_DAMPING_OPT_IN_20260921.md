# Global/tunnel damping opt-in — September 21

The gain-0.75 lateral-stop result is now available in the global-planner flight
adapter as an **opt-in, default-OFF** mode. Both the flown C++ adapter and its
Python fallback accept the same launch parameters.

## Effective defaults

| Parameter | Default |
| --- | --- |
| `brake_damping` | `false` |
| `damping_gain` | `0.75` |
| `damping_limit` | `0.10` m/s² |
| `damping_time_constant` | `0.10` s |
| `damping_velocity_timeout` | `0.10` s |
| Total horizontal acceleration cap | existing `command_acceleration`, 0.30 m/s² |

The correction is `gain * (reference_velocity - measured_velocity)`, vector
limited, low-pass filtered and added only while the normal validated route is
braking or holding. The combined acceleration is capped again at the existing
route acceleration limit. Cruise and acceleration phases smoothly release the
correction.

It clears immediately for climb, LAND, planner-fault hold, command hold,
degraded follower ride-out, clearance escape, NaN/pure-position control,
invalid or older-than-100-ms PX4 velocity, control callback gaps over 150 ms,
and PX4 local-frame resets. Existing route clearance, geofence, yaw, fault and
landing behavior is unchanged.

The flight bag now records `/planner/flight/damping_debug` on every setpoint.
The C++ adapter's latched `/planner/flight/config` includes all damping values
and `damping_scope=validated_route_braking_and_holds`.

## First trial command

Run the normal planner/VIO stack and confirm the usual health and route gates.
For the first props-off check, keep `auto_arm:=false`:

```bash
cd /home/john/autonomous_drone_px4_vio
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source ros2_ws/install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge offboard_global_planner.launch.py \
  cpp_nodes:=true \
  auto_arm:=false \
  brake_damping:=true
```

After that dry run is reviewed, the controlled armed trial uses the identical
launch with `auto_arm:=true`. Keep the existing tunnel route speed and all
other proven settings unchanged. To roll back, omit the argument or set
`brake_damping:=false`; no rebuild is needed.

## Verification completed

- Package build completed successfully.
- C++ damping and route-limiter suite: 62/62 passed.
- Targeted Python adapter, launch and lateral-regression tests: 117/117 passed.
- The standalone 3D planner integration suite passed 2/2. The wider package
  run still has two unrelated issues: an existing 3D feature-watchdog teardown
  timing failure and a missing `depthai_ros_msgs_v3` test dependency.
- An isolated-domain, non-arming C++ constructor check read back enabled gain
  0.75, correction cap 0.10, filter 0.10 and velocity timeout 0.10.

No armed process, PX4 parameter write or live flight command was issued.
