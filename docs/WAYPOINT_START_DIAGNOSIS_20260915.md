# Waypoint rejection: September 15, 05:08 and 05:15 tests

## Stationary test: the configured lower boundary blocks the envelope

Source: `ros2_ws/flight_logs/global_planner_3d_monitor_20260915T051504Z`.
Both waypoint clicks arrived. The body stayed near `(-0.093, -0.018, 0.056)` m.
The 50 cm test clearing was enabled and correctly centered; the bag even shows
its new per-click anchors. All 18 planning attempts reported `START_BLOCKED`.

The actual planner and follower launch parameters were **min_z=-0.20,
max_z=1.60**, with a **0.35 m** clearance radius. The lowest allowed body center
was therefore `-0.20 + 0.35 = 0.15 m`. The actual center was only 0.056 m:
the required sphere extended 9.4 cm through that lower boundary. The test map
clearing does not override configured geofence planes.

Direct probes of the recorded binary OctoMaps found **zero occupied or unknown
voxels inside the start's 35 cm sphere** at either click. This distinguishes
the boundary rejection from sensor-map clearance. The body was stationary;
movement was not the cause of this test's start rejection.

Replay with `min_z=-0.20` reproduces the rejection. The updated planner names it
`START_ENVELOPE_OUTSIDE_GEOFENCE` and includes the current start, requested goal,
configured bounds, and allowed body-center Z range in status messages.
Replay with the documented artificial test band `min_z=-0.80, max_z=1.60`
unblocks the start and creates short exploration paths, about 7.7 cm long.

## The distant waypoints still have insufficient mapped clearance

The clicks were `(1.669, -0.050, 0.0)` and `(1.175, 0.022, 0.0)` m.
Their required 35 cm envelopes overlap both occupied and unknown cells:

| Click | Nearest occupied surface | Unknown cells within 35 cm | Occupied cells within 35 cm |
|---|---:|---:|---:|
| First | 0.140 m | 948 | 95 |
| Second | 0.152 m | 540 | 37 |

Only a small component near the artificial start sphere is reachable. A direct
planner probe using the final map, wider test bounds and six target heights
from z=0.0 to z=0.7 m still found only **13 reachable 5 cm body-center voxels**.
Changing goal height alone cannot create a connected free route in this map.

The old follower incorrectly labelled arrival at a partial exploration endpoint
`GOAL_REACHED`. Path snapshots now carry `goal_exact` and `goal_terminal`.
The follower distinguishes `EXPLORATION_ENDPOINT_REACHED`,
`SAFE_APPROACH_REACHED`, and the actual `GOAL_REACHED`. The public
`/planner3d/follower/goal_reached` flag is true only for the requested waypoint.
The auditor checks clearance for the new partial-endpoint states as well.

## Earlier moving test: a separate anchoring problem

Source: `global_planner_3d_monitor_20260915T050850Z`.
The initial implementation kept the clearing at the first raw body pose. At
the two clicks, the body had moved 0.365 m and 0.300 m from that center. A 0.35 m
body envelope cannot fit in a 0.50 m cleared sphere at those offsets. The second
click also placed the body too close to the configured lower boundary.

The test override now re-anchors to the latest fresh, valid raw body pose on
each fresh waypoint click. It remains fixed between clicks and appears in
the next map generation. Each publication starts from the sensor tree, so old
artificial clearing does not accumulate. Invalid/stale tracking, wrong-frame
or future clicks, and duplicate click timestamps cannot move the anchor.
Default radius remains zero; the feature remains explicitly artificial.

## CPU and timings actually recorded live

These bags include CPU metrics. During active mapping:

| Metric | Moving test | Stationary test |
|---|---:|---:|
| Machine CPU median / max | 54.5% / 70.4% | 52.8% / 60.7% |
| OctoMap processing median / max | 64.7 / 393.5 ms | 27.9 / 47.3 ms |
| Pi throttle word | 0 | 0 |

The waypoint rejection in the stationary test is geometric, not a missed click
or a stale-map rejection. This does not imply that every pipeline latency spike
has been eliminated.

## Final stationary replay

With the wider test band and the final code, the same recorded body motion and
both recorded clicks produce **20 partial path snapshots and 218 follower chord
checks**. The exact-map audit passes with zero violations, explicitly marked
artificial clearance. All 109 reached-endpoint samples report
`EXPLORATION_ENDPOINT_REACHED`; none claim the requested waypoint was reached.
This verifies the corrected diagnostic behavior, not a full route to either
clicked point.

The earlier moving bag also replays with the new per-click anchors: both
recorded clicks select the corresponding current body pose. It now produces
12 partial path snapshots and 170 valid follower chord checks (zero audit
violations), with short 10–13 cm exploration paths. Further motion still leaves
the fixed per-click test volume and correctly triggers clearance/tracking
rejections. Neither replay reaches the distant requested waypoint.

## Verification and artifacts

Package build succeeded; all **38 CTest targets / 739 tests** passed, with zero
errors/failures and three intentional parity skips. The real-ROS integration
tests exercise the floor-envelope rejection, re-anchoring, duplicate and
invalid clicks, fixed anchors between clicks, and partial/exact endpoint
classification even when route geometry remains identical.

Commands used from `ros2_ws` after sourcing the workspace:

```bash
colcon build --packages-select px4_vio_bridge --executor sequential --parallel-workers 1
python3 -m pytest src/px4_vio_bridge/test/test_ros_planner_3d_integration.py -q
colcon test --packages-select px4_vio_bridge --executor sequential --event-handlers console_direct+
colcon test-result --verbose
```

Artifacts are under `/home/john/planner3d_waypoint_20260915T051504Z/` and
`/home/john/planner3d_waypoint_20260915T050850Z/`: source-message extracts,
captured live launch parameters, exact-map clearance probes, height probes,
replay bags/logs/reports and audits. This follow-up was handled locally by the
lead because diagnosis, message contracts and ROS integration were coupled.

## Next observation test

Use the command at the top of [HANDOFF_3D_NAVIGATION.md](HANDOFF_3D_NAVIGATION.md).
For the artificial on-ground startup diagnostic, preserve **min_z:=-0.8**;
`-0.2` reproduces this failure. That wider band is a test setting, not a measured
physical floor. A useful room-route test needs the camera/drone held off the
floor with enough actual clearance, and views ahead, to the sides, up and down
that establish a connected free volume around the drone envelope. Stop motion
before clicking and wait for the next map update. A short `EXPLORING` path means
only its endpoint is currently reachable; it does not validate the distant goal.
