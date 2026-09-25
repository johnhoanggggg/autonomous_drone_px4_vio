# Unknown-aware 3D flight adapter — September 15, 2026

## September 22 flight/ULog diagnosis and safety correction

The four armed 02:44--02:45 bags and their matching PX4 ULogs separate two
failures. The 02:44:42 flight entered `FOLLOWING`, but its horizontal setpoint
moved only about 1.2 cm while the vehicle oscillated about 32 cm. PX4 motor,
attitude, yaw-bias and saturation evidence was normal; the ULog delay analysis
instead measured approximately 450 ms of residual external-vision delay. The
monitor bag and flight bag were recording simultaneously, and both serialized
large RTAB-Map messages while one CPU core was saturated. The two later flights
landed immediately because a planner-fault timer started while disarmed and its
already-expired value carried across the armed boundary.

The adapter now starts the persistent-fault LAND grace period only while armed
and clears any pre-arm fault state on a healthy stream/engage sample. The flight
path uses `/rtabmap/vio_odom_info_lite` for feature health and recording. Both
the raw-VIO bridge and flight adapter enforce `max_vio_source_age:=0.30`, so
vision older than PX4 1.17's maximum supported external-vision delay cannot arm
or continue commanding the vehicle. The flight configuration records the
selected health topic and source-age limit.

`EKF2_EV_DELAY` remains at the observed controller value of 225 ms. Do not set
it to the overloaded flight's measured 450 ms: PX4 1.17 limits that parameter
to 300 ms, and compensating for avoidable compute backlog is not a valid fix.
Before another props-on flight, run one recorder only, perform a props-off
latency capture, and require the measured delay and source-age tail to remain
below 300 ms. Then retune `EKF2_EV_DELAY` from that clean capture if needed.

## September 22 VIO serialization-delay reduction

The apparent lightweight odometry-info topic was still carrying RTAB-Map's
entire F2M local bundle. In the observed graph that bundle had 1,412 camera
models and poses and made each `/rtabmap/vio_odom_info_lite` message about
718 kB. The mapper subscribed to the full odometry-info topic as well, forcing
the same growing bundle through another serialization path on every update.

The local RTAB-Map overlay now clears `local_bundle_ids`,
`local_bundle_models`, and `local_bundle_poses` when publishing the lite topic.
The mapper consumes that lite topic because it needs the scalar health,
covariance, timing, and transform fields rather than the historical bundle.
The normal flight bag also omits `/rtabmap/mapData`; native OctoMap, paired
metadata, snapshots, status, and commands remain recorded for the independent
command audit. Record MapData only for an explicit planner-replay diagnostic.

In a stationary, observation-only run with the flight-style recorder active,
the lite message fell from about 718 kB to 0.76 kB. A 29-second bag fell from
156.7 MiB to 1.2 MiB (about 99.2 percent), and raw VIO source age averaged
165 ms with a 204 ms maximum. The clean no-recorder baseline averaged 152 ms.
This is a substantial improvement over the approximately 450 ms residual delay
measured in the failed flight, but it is not a dynamic estimator-delay
calibration. Repeat the latency measurement with props removed and deliberate
translation/yaw before changing `EKF2_EV_DELAY` or attempting another flight.

A clean-reboot live check isolated the startup failure from the saved PX4
configuration. NSH confirmed `EKF2_EV_CTRL=11`, `EKF2_EV_DELAY=225`,
`EKF2_HGT_REF=3`, and `EKF2_GPS_CTRL=0`. With the monitor recorder and markers
disabled, ROS VIO reported about 126--155 ms source age. PX4's
`vehicle_visual_odometry` listener received the converted samples and EKF2
reached `local position: 1` within five seconds; ROS then reported
`xy_valid=true` and `dead_reckoning=false`. The failed launch had instead
started the bridge, second recorder, and auto-arm adapter together. Its fixed
10-second local-position watchdog expired during that cold-start load.

The 3D flight launch now starts its lightweight recorder and VIO bridge first,
then delays the flight adapter by `adapter_start_delay:=5.0`. The adapter's own
10-second position watchdog remains intact, giving a 15-second cold-start
window without weakening the in-flight validity checks. This mirrors the
working 2D sequence, where the VIO/PX4 bridge is already warm before its flight
adapter starts.

## September 22 two-flight throughput diagnosis

The 11:43 flights confirmed that the 0.10 m/s route limit was conservative,
but they spent much of their time in safety HOLD rather than moving at that
limit. The first flight saw two live bag recorders and the second saw three,
including an older monitor recorder and a previous flight recorder that had
not exited before the next launch. Pi CPU averaged 83--86 percent and reached
95--97 percent. Raw VIO source age exceeded 300 ms in 26/215 samples in the
first flight and 56/942 in the second. The longer flight was in FOLLOWING for
only about 56 percent of status samples; its actual following speed averaged
0.046 m/s even though position tracking remained good (0.036 m mean and 0.099
m maximum horizontal error). The resulting slowness was therefore mostly
compute-driven holds and replanning, not poor PX4 position tracking.

The flight launch now refuses to start while any older `ros2 bag record`
process is alive. This catches both a monitor accidentally launched with
`record_bag:=true` and a previous flight recorder that is still finalizing.
The adapter rate is reduced from 50 to 20 Hz, matching the working 2D adapter
and avoiding redundant full 3D chord checks while remaining comfortably above
PX4's offboard setpoint requirement. Route speed is exposed as
`route_speed:=...`, defaults to a measured, conservative 0.15 m/s, and retains
`command_speed:=...` as a backward-compatible override.

## September 22 explicit unknown-flight override

At the operator's explicit request, the dedicated
`offboard_planner_3d_unknown.launch.py` launch now commands the full
`/planner3d/reference_snapshot` with `allow_unknown:true`. Occupied voxels and
the geofence remain blocked, and the adapter still revalidates every chord
against the latest native OctoMap, but unknown voxels are treated as
traversable. This bypasses the unknown-space takeoff inhibition seen in the
02:33 attempts. `auto_arm` continues to default to false.

## September 22 planner throughput correction

The 00:55 flight also measured the 3D planner at roughly 1.1--1.4 s per map:
about 0.5 s preparing the conservative dense volume and another 0.7--0.9 s
building an optimistic reference volume, searching it, and invoking a
short-prefix fallback. That fallback commonly spent its search allowance for
about 5 mm of additional verified travel or returned no route.

Unknown-allowed planning now retains one native 5 cm observed grid and builds
only one inflated optimistic grid. Cells which are unknown in the observed
grid receive a high but traversable proposal cost; occupied cells and the hard
geofence remain lethal. The candidate is still checked with the unchanged
continuous swept-sphere verifier against the observed grid, where unknown is
blocked, and only that verified prefix is published as executable. The
conservative inflated duplicate and the under-15 cm fallback search have been
removed. The flight adapter continues to perform its independent validation
against the latest native OctoMap.

Inflation and A* now run in a single background job. Map, pose, and goal
callbacks remain responsive while it runs. There is no pending-job queue: if a
new map supersedes a result, that result is discarded and the newest coherent
snapshot is launched next. A 20 ms completion poll prevents a finished job
from waiting for the next 2 Hz planning timer and colliding with the following
1 Hz map. Planner timing now includes `grid_ms` and `worker_total_ms`, and
reference status records `known_fallback_used:false`.

An isolated 25 s replay of the original 00:55 monitor inputs through the
rebuilt planner accepted 11 current-generation plans. Worker time was
0.582--0.601 s, plus 0.092--0.097 s to convert each native OctoMap into the raw
dense grid. Over the comparable part of the original recording, the old dual
inflation/fallback implementation normally used about 1.17--1.34 s and reached
2.19 s once. The replay used an isolated ROS domain and no PX4 publishers.

## September 22 path-generation revalidation correction (00:55 flight)

Flight `offboard_planner_3d_unknown_20260922T005551Z` reached the 0.30 m hover
but never entered `ROUTE_START`. The preceding empty-snapshot correction was
effective: all 19 unknown-policy reference snapshots were non-empty. The
adapter instead reported `PATH_MAP_MISMATCH` for 896 status samples because
grid preparation plus A* took about 1.1–2.2 s and each route trailed the faster
OctoMap stream by one or two generations.

The adapter now retains each source route in the continuous odometry frame,
using the exact historical `map_from_odom` transform. For every newer map it
reprojects the route into that map and repeats the complete 3D swept-sphere and
geofence validation before marking the current generation executable. Future
routes, missing transform history, stale routes, occupied crossings, and
pose/map mismatch still hold. Status records both the source
`path_map_generation` and `path_validated_map_generation`.

An isolated replay of the failed bag through the fixed adapter, with all PX4
outputs remapped away from hardware, reaches `ROUTE_START` and `FOLLOWING`;
source generation 75 is revalidated against map generations 76, 77, and 78.
The mock-PX4 integration forces a one-generation lag in all three cases and
also proves that a newly occupied crossing still withdraws the route. The
complete package result is 765 tests with zero errors/failures and three skips.

## September 21 current-height goal correction (14:25 / 14:26 flights)

The next two flights used the corrected route-entry adapter but exposed a
planner-side height contract. In
`offboard_planner_3d_unknown_20260921T142531Z`, hover latched at bag time
7.77 s and the adapter entered `ROUTE` at 22.44 s. It immediately reported
`ROUTE_OBSTACLE_OR_GEOFENCE`. Exact map-to-NED conversion showed that the
route's last point was 0.1425 m above initial PX4 ground, below the adapter's
0.15 m minimum; every preceding route point was inside the flight bounds. The
goal had been clicked before takeoff with `goal_height_mode:=current`, so its
height had been latched near the floor rather than at the later hover.

The operator clicked again at bag time 25.22 s while hovering, but that click
coincided with a transient map/pose generation mismatch. The planner reported
`GOAL_HEIGHT_POSE_UNAVAILABLE`, cleared the goal, and permanently discarded the
click. The following flight,
`offboard_planner_3d_unknown_20260921T142614Z`, consequently received only
zero-point reference snapshots and remained `WAITING_FOR_ROUTE`. Its recorded
click was the old latched Foxglove message from the preceding flight (same
source timestamp), not a new accepted goal.

The ROS/PX4 3D monitor now sets
`current_goal_min_height_above_start:=0.30`. In current-height mode, a waypoint
clicked before takeoff therefore targets at least the 30 cm hover plane rather
than generating a route back toward the floor. The generic planner default is
still zero, so non-flight use retains the old literal current-height behavior.
The planner also buffers the newest current-height click when pose/map
generations are temporarily incoherent and resolves it automatically on the
next coherent snapshot. Status exposes `goal_height_pending` and the configured
height floor. The monitor must be restarted after rebuilding to load both the
new executable and launch argument.

The package rebuild passed. Focused validation passed the production planner
integration 2/2, mock-PX4 flight integration 3/3, and launch checks 12/12. The
complete package run passed all 42 CTest targets: 764 tests, zero
errors/failures, three skips.

## September 21 route-entry correction

Flight `offboard_planner_3d_unknown_20260921T140512Z` did leave active
takeoff: it first latched the 0.30 m hover at bag time 8.03 s and subsequently
published 1,361 `WAITING_FOR_GOAL` statuses. The apparent failure to exit
`CLIMB_HOLD` came from two issues. The internal base-state name remained
`CLIMB_HOLD` while the 3D adapter was using that state to hold at altitude and
wait for route authorization, and the only waypoint click was at bag time
2.00 s, before hover. The then-current policy required goal generation 2 after
hover, while every route snapshot remained at generation 1. The aircraft was
therefore inhibited from following the route; this was not a continued climb.

The default route-entry policy now matches the tested 2D workflow without
blindly accepting a stale preflight path. A goal clicked before takeoff remains
eligible, but the adapter waits for a new, coherent route snapshot for that
goal after hover. Exact map and pose generations must match; a lagging path is
accepted only under the newer revalidation contract described above. All
health, geofence and swept-volume gates remain in force. Set
`require_goal_after_hover:=true` only when a deliberate second click after
takeoff is wanted. While waiting, `flight_state` now reports
`WAITING_FOR_ROUTE`, and status includes `hover_ready`,
`minimum_goal_generation` and `post_hover_path` so this gate is observable.

The same bag had transient holds (53 `POSE_MAP_MISMATCH`, 32
`PX4_STATE_STALE_OR_INVALID`, eight `FINAL_COMMAND_OBSTACLE`, five
`POSE_STALE`, and two `MAP_STALE` statuses). They did not undo the latched
hover. They can still delay route entry until a coherent snapshot is present.

The rebuilt adapter passes all three mock-PX4 flight cases, including explicit
assertions that a preflight goal is retained, the post-hover route gate is
visible, and a new snapshot of that same goal starts following. Launch checks
pass 12/12. Current complete-package validation is recorded in the newer
current-height correction section above.

## September 21 takeoff parity correction

Flight `offboard_planner_3d_unknown_20260921T134353Z` confirmed that the
last-safe-command hold correction worked: the horizontal setpoint remained
fixed. The flight instead exposed two independent takeoff differences from the
tested 2D adapter. Its effective configuration used a 0.50 m hover target, a
0.05 m/s position ramp and no vertical velocity feedforward. Eighty-seven brief
health holds interrupted that ramp. The setpoint needed about 15 seconds to
cover 0.50 m, first reported `WAITING_FOR_GOAL` at bag time 19.32 s, and then
alternated back to `CLIMB` when the aircraft overshot the reach tolerance. The
still-active 25 s climb timer requested LAND at bag time 30.02 s. The bag
contained 36 `POSE_MAP_MISMATCH`, 26 `PX4_STATE_STALE_OR_INVALID`, 21
`POSE_STALE`, and four `VIO_HEALTH` hold statuses.

The 3D takeoff phase now uses the flight-tested 2D `ramp_z()` profile: 0.25 m/s
with a 0.12 m position leash, 0.05 m release taper and vertical velocity
feedforward. The default hover height is 0.30 m and the climb timeout is 15 s.
These parameters apply only to takeoff; the 3D route retains its conservative
0.10 m/s horizontal and 0.05 m/s vertical limits. Every ramped takeoff command
still passes the 3D geofence and swept-chord gates before publication, and a
fault cancels the velocity feedforward while freezing the last safe position.

Hover readiness is now a one-way latch like the 2D transition out of takeoff.
Once stable hover has been observed, overshoot while waiting for an authorized
route cannot re-enter `CLIMB` or revive its timeout. The production-node
mock flight test verifies the -0.25 m/s feedforward, taper to zero, a 12 cm
post-hover overshoot, the last-safe-command hold, route motion and terminal
fault handling for all three scenarios.

## September 21 flight hold correction

Flight `offboard_planner_3d_unknown_20260921T125106Z` never entered route
following. During `CLIMB_HOLD`, 60 brief `POSE_MAP_MISMATCH`, 26
`PX4_STATE_STALE_OR_INVALID`, and three `VIO_HEALTH` holds repeatedly replaced
the setpoint with the aircraft's current position. The published horizontal
command consequently followed physical drift by 0.512 m; measured PX4 motion
reached 0.519 m. The 25 s climb timeout then requested LAND, followed by
disarm. This was a takeoff-hold defect, not motion along the proposed route.

The adapter now freezes its last safe command for ordinary transient faults.
It rebases to the current PX4 position only for a fatal estimator-frame reset,
immediately before requesting LAND. Flight configuration records
`fault_hold_policy=last_safe_command`. The production-node integration test
injects a 12 cm vehicle disturbance while pose data goes stale and verifies no
horizontal setpoint jump and no command motion while held.

Do not infer flight clearance from the passing offline command audit: that run
allowed unknown cells and used the artificial 0.50 m start clearing. A fresh
props-off test is required before any further motor test.

The PX4/`px4_msgs` mismatch observed on September 21 is resolved in the shared
underlay. `/home/john/ros2_ws/src/px4_msgs` is pinned at tag `v1.17.0`, commit
`86d8239e962f6939e05c3737784f60c02fa884db`, and both the underlay and this
overlay were rebuilt. A live read-only `/fmu/out/estimator_status_flags` sample
decoded all 74 PX4 1.17 fields without a CDR error. The production-node flight
integration passed 3/3 cases and the complete package result was 764 tests with
zero failures. This corrects interface compatibility; it does not replace the
required props-off test or establish physical route clearance.

## Current implementation

`offboard_planner_3d_unknown.launch.py` now connects the full exploratory
reference (`/planner3d/reference_snapshot`) to PX4 under its explicit
unknown-traversable policy. It reuses the existing C++
`OffboardHover` sequencer, Offboard/arming requests, tracking watchdogs, yaw
ramp, landing and emergency controls. The amber full proposal is flight input
for this dedicated launch.

Occupied voxels are blocked by a 35 cm swept sphere: 25 cm robot radius plus
10 cm margin. Unknown voxels are traversable. The adapter revalidates the
reference against every newer native OctoMap before execution. The source
OctoMap is not modified by the adapter. The monitor's requested 50 cm
artificial start clearing remains an explicit test assertion and is recorded
in flight status; it is not a sensor measurement. Historical instructions
that 3D outputs have no flight consumer are superseded for this dedicated
launch.

## Start the monitor

Use the ROS-native setup, with the backend and monitor running once:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch px4_vio_bridge ros_planner_3d_monitor.launch.py \
  start_backend:=true foxglove:=true record_bag:=false \
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
  geofence_radius:=4.0 route_speed:=0.15
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
running. Corrected map poses must never feed PX4 external vision. By default,
the flight adapter starts five seconds after this bridge; override
`adapter_start_delay` only for a measured startup diagnostic.

Keep the monitor recorder disabled during flight. The adapter launch owns the
single flight recorder and contains the maps, commands, status, process load,
and lightweight VIO health needed by the command audit. Enabling both recorders
recreates the serialization load associated with the 450 ms VIO backlog. The
flight launch now aborts before starting any nodes if it finds an existing bag
recorder; wait for the previous recorder to exit before starting another flight.

For a later supervised flight, `auto_arm:=true` enables the existing arming
sequence. The adapter climbs to a 0.30 m hover and reports
`WAITING_FOR_ROUTE` until it receives a coherent post-hover route snapshot.
A Foxglove waypoint clicked before takeoff remains eligible; no second click is
required by default. With `goal_height_mode:=current`, clicked XY is latched at
the planner's current height. Set `require_goal_after_hover:=true` to restore
the stricter second-click policy. Normal carry tests cannot exercise PX4's
position response or the actual takeoff sequence.

Controls: **L** requests LAND; **K** forces disarm. The existing Foxglove
Twist control is `/planner3d/flight/teleop`: `linear.z=-1` for LAND and `-2`
for KILL. Keep the RC controls available as in the existing flight procedure.

## Motion and rejection behavior

- Full map-to-odometry quaternion conversion, then ENU to PX4 NED; the bridge
  and adapter share `vio_yaw_offset_deg`. PX4 position is the local command
  reference, and takeoff bounds use the initial PX4 Z rather than assuming Z=0.
- Setpoints publish at 20 Hz. Takeoff uses the tested 2D 0.25 m/s vertical
  velocity feedforward with a 0.12 m position leash and 0.05 m release taper;
  faults cancel that feedforward. Route motion remains position-only, with
  default horizontal/vertical speeds of 0.15/0.05 m/s and accelerations of
  0.30/0.20 m/s². The flight follower preserves each segment's slope and brakes
  at corners. It turns toward the route before translating when heading error
  exceeds 40 degrees, resuming below 15 degrees.
- Default center heights are 0.15–1.25 m above the initial PX4 position, with a
  0.30 m hover. The horizontal geofence is 3 m, with the 0.35 m envelope reserved
  inside it. Planner map-height bounds and adapter takeoff-relative bounds are
  separate; a complete proposed route outside the flight bounds is rejected.
- Exact map/metadata/pose generations and timestamps must match. A fresh route
  may trail the current map only after odometry-frame reprojection and complete
  validation against the current generation. Stale tracking, wrong frames,
  changed estimator origin, known obstacles, and excessive tracking error
  inhibit motion. Temporary planner faults latch HOLD;
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

The full package run reports **765 tests, zero errors/failures, three skips**.
The new geometry tests cover full quaternion/yaw-offset conversion, unknown
policy versus occupied obstacles, relative height bounds, speed/acceleration
limits, steep segments/corners and persistent vehicle position error. The
mock-PX4 tests run the installed C++ node in isolated domains with all PX4
topics remapped; they cover automatic-arm suppression, duplicate-adapter
rejection, XYZ progress, yaw alignment, stale and inconsistent inputs, new
obstacles, and LAND on lost tracking or PX4/VIO resets. They record the actual
setpoints and run the independent bag auditor.

The final yaw-hold adjustment passes all three mock-PX4 cases, including
injected drift while turning. The 765 count above is the full run; the final
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
