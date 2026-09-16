# 3D unknown-allowed flight adapter — September 15, 2026

## Current implementation

`offboard_planner_3d_unknown.launch.py` connects the amber unknown-allowed route
(`/planner3d/reference_snapshot`) to PX4. It reuses the existing C++
`OffboardHover` sequencer, Offboard/arming requests, tracking watchdogs, yaw
ramp, landing and emergency controls. The monitor still publishes its separate
verified local path; this dedicated flight adapter deliberately consumes the
full proposal requested by the user.

Unknown voxels are traversable in this mode. Known occupied voxels remain
blocked by a 35 cm swept sphere: 25 cm robot radius plus 10 cm margin. The
source OctoMap is not modified by the adapter. The monitor's requested 50 cm
artificial start clearing remains an explicit test assertion and is recorded
in flight status; it is not a sensor measurement. Historical instructions that
3D outputs have no flight consumer are superseded for this dedicated launch.

## Start the monitor

Use the ROS-native setup, with the backend and monitor running once:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch px4_vio_bridge ros_planner_3d_monitor.launch.py \
  start_backend:=true foxglove:=true record_bag:=true \
  test_start_clear_radius:=0.5 min_z:=-0.8 max_z:=1.6 \
  goal_height_mode:=current exploration_route:=true
```

If this graph is already running, reuse it. Restart it to pick up the updated
CPU process list, which now includes the flight adapter and raw-VIO bridge.
The legacy `rtabmap_slam_px4.launch.py` is not the backend for this launch.

## Start the adapter in a second terminal

For the initial props-off PX4 check, source the same environment and run:

```bash
ros2 launch px4_vio_bridge offboard_planner_3d_unknown.launch.py auto_arm:=false \
  geofence_radius:=4.0
```

The example uses a 4 m geofence to contain the latest wider-room route. The
launch default is 3 m (2.65 m usable center radius), which would reject that
recorded endpoint roughly 3 m from its start. Choose a waypoint within the
configured bounds.

This **publishes real PX4 commands**, including Offboard requests, even with
`auto_arm:=false`; that parameter suppresses automatic ARM requests. Use
props off for this check. Only one flight adapter may run per machine/domain:
C++ and Python adapters now share a process lock. Stop an older adapter first.

The launch starts the raw `/rtabmap/vio_odometry` to PX4 bridge by default.
Use `start_vio_bridge:=false` only when the same raw-VIO bridge is already
running. Corrected map poses must never feed PX4 external vision.

For the later supervised flight, `auto_arm:=true` enables the existing arming
sequence. This implementation has not yet been validated on an armed vehicle.
It climbs to a 0.5 m hover, reports `WAITING_FOR_GOAL`, then requires a **new**
Foxglove waypoint on `/waypoint/clicked`. With `goal_height_mode:=current`,
clicked XY is latched at hover height. A waypoint clicked while sitting on the
floor is intentionally not flown after takeoff. Normal carry tests cannot
exercise PX4's position response or the actual takeoff sequence.

Controls: **L** requests LAND; **K** forces disarm. The existing Foxglove
Twist control is `/planner3d/flight/teleop`: `linear.z=-1` for LAND and `-2`
for KILL. Keep the RC controls available as in the existing flight procedure.

## Motion and rejection behavior

- Full map-to-odometry quaternion conversion, then ENU to PX4 NED; the bridge
  and adapter share `vio_yaw_offset_deg`. PX4 position is the local command
  reference, and takeoff bounds use the initial PX4 Z rather than assuming Z=0.
- Position-only setpoints at 50 Hz; velocity/acceleration feedforward is unset.
  Horizontal/vertical speeds default to 0.10/0.05 m/s, accelerations to
  0.30/0.20 m/s². The flight follower preserves each segment's slope and brakes
  at corners. It turns toward the route before translating when heading error
  exceeds 40 degrees, resuming below 15 degrees.
- Default center heights are 0.15–1.25 m above the initial PX4 position, with a
  0.5 m hover. The horizontal geofence is 3 m, with the 0.35 m envelope reserved
  inside it. Planner map-height bounds and adapter takeoff-relative bounds are
  separate; a complete proposed route outside the flight bounds is rejected.
- Exact map/metadata/pose/path generations and timestamps must match. Stale
  tracking, wrong frames, changed estimator origin, known obstacles, and
  excessive tracking error inhibit motion. Temporary planner faults latch HOLD;
  persistent faults request LAND after 3 s when armed. PX4/VIO resets while
  armed request LAND immediately. Existing tracking-loss and maximum-flight
  watchdogs remain active.
- Every moving command checks both actual-body-to-command and
  previous-command-to-command swept spheres against the current OctoMap,
  after conversion and final position limiting. HOLD is an emergency fallback,
  not a claim that newly occupied space around the vehicle is clear.
- Exact goals report `GOAL_REACHED`. An approximate endpoint reports
  `SAFE_APPROACH_REACHED`; it does not claim to reach the clicked waypoint.
  Terminal endpoints hold for 3 s then request LAND when armed.

## Bags, CPU and command audit

The flight launch records `offboard_planner_3d_unknown_*` under
`ros2_ws/flight_logs`, including actual PX4 setpoints/commands, raw VIO and
RTABMap health, paired maps/poses/paths, flight configuration/status and
`/perf/*`. CPU production comes from the running monitor. There is no new
camera or mapper instance in the flight launch.

Audit a completed flight bag offline:

```bash
cd /home/john/autonomous_drone_px4_vio
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
python3 scripts/audit_flight_3d_bag.py ros2_ws/flight_logs/REPLACE_WITH_BAG \
  --out /tmp/flight3d_command_audit.json
```

Choose a new output filename. The audit matches status to the exact published
PX4 float setpoint and map generation, independently checks frame conversion,
flight bounds and swept chords, and counts segments crossing unknown. A
passing unknown-policy audit does not establish observed physical clearance.

## Validation

Implementation and integration were done locally because the frame conversion,
flight state machine and command checks are tightly coupled; no subagents were
used for this adapter.

Commands from `ros2_ws` with the environment above:

```bash
colcon build --packages-select px4_vio_bridge --executor sequential --parallel-workers 1
colcon test --packages-select px4_vio_bridge --executor sequential --event-handlers console_direct+
colcon test-result --verbose
```

The full package run reports **764 tests, zero errors/failures, three skips**.
The new geometry tests cover full quaternion/yaw-offset conversion, unknown
policy versus occupied obstacles, relative height bounds, speed/acceleration
limits, steep segments/corners and persistent vehicle position error. The
mock-PX4 tests run the installed C++ node in isolated domains with all PX4
topics remapped; they cover automatic-arm suppression, duplicate-adapter
rejection, XYZ progress, yaw alignment, stale and inconsistent inputs, new
obstacles, and LAND on lost tracking or PX4/VIO resets. They record the actual
setpoints and run the independent bag auditor.

The final yaw-hold adjustment passes all three mock-PX4 cases, including
injected drift while turning. The 764 count above is the full run; the final
focused run selects only the flight integration CTest target. It was checked
with:

```bash
colcon test --packages-select px4_vio_bridge --executor sequential \
  --ctest-args -R '^test_flight_3d_integration$'
colcon test-result --verbose
```

For the latest wider-room bag (`global_planner_3d_monitor_20260915T080059Z`),
frozen-map simulation of generation 77 traverses **3.904 m** and reaches within
**4.98 cm** of the safe-approach endpoint in **46.42 s** of simulated motion.
All **2,321** float-valued command chords pass the independent native audit;
**2,084** cross unknown. Maximum horizontal/vertical speed is 0.09867/0.04990
m/s; maximum acceleration is 0.2792/0.1994 m/s². The earlier axis-by-axis
follower stopped after 5.7 cm on this same route; direction-preserving motion
and stopping at vertices resolve that simulated failure.

This replay uses perfect position tracking and a frozen recorded map. It
checks the route from the bag's floor-level start, without takeoff or flight
geofence sequencing; the mock-node tests cover those separately. It is neither
PX4 SITL nor evidence of physical flight clearance. Replay execution used
2.68 CPU seconds for 46.42 virtual seconds; live adapter CPU under the complete
camera/mapper load remains to be measured in the next flight bag.

Artifacts, replay source/input and logs are in
`/home/john/flight3d_adapter_20260915/`. No hardware launch, arming, service
change or physical flight was performed during implementation.
