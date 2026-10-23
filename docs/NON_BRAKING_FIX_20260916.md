# Correction and route-frame consistency — September 16

Branch: `fix/non-braking-lateral-excursions`, based on checkpoint `8f6b3e8`.

The September 16 07:16:23 flight published a roughly 15 cm position-reference
pulse between receipt of a new map correction and its matching body pose. The
07:18:45 flight had a smaller, predominantly longitudinal pulse. The old C++
callbacks independently replaced these inputs. A timer tick could therefore
reframe its command using the new transform and subtract the old-frame pose.
The velocity-feedforward limit did not constrain that position-reference jump.

## Change

The 2D C++ planner, follower and flight adapter now adopt a body pose and map
correction together only when their **source timestamps match exactly**. They
retain the previous complete pair while one half is in flight. Each constituent
keeps its original receipt age; unmatched or duplicate traffic cannot keep old
localization fresh indefinitely. Invalid corrections still inhibit motion.
Pending storage and completed-pair history are bounded.

Paths also carry coordinate provenance. `/planner/path.header.stamp` is now the
source timestamp of the localization pair used to generate its geometry, rather
than the publication time. A consumer must have that exact pair in its history
before accepting a path. Paths arriving before their localization are
queued with their original receipt time; no nearest-time transform is guessed.
Older path messages cannot overwrite newer ones or clear the follower's newer
route. Unmatched future-dated paths cannot advance the accepted-path ordering
marker and block subsequent valid paths. A missed source sample fails closed
until a usable fresh path arrives.

The adapter keeps the path's source correction and re-expresses its geometry
into the active pair's frame before installation. This fixes the companion
problem where it correctly reframed the retained limiter, then immediately
reinstalled its last cached path in old map coordinates. The planner periodically
republishes a clearance-revalidated retained route with current provenance even
when an unchanged planning key avoids another A* search.

The follower's correction-settling and map-generation gates remain in place.
Deferred-path processing preserves original receipt times and does not replay a
pre-settling path as a newly received one. Existing command speed, vector
acceleration, swept-chord clearance and geofence checks remain active. Fault-hold
escape now also requires fresh map/localization data. Its stale-displacement
message explicitly identifies the follower's VIO-frame output, avoiding the
misleading implication that raw VIO stopped arriving.

## Scope and deployment

Restart **all three C++ 2D nodes together** after switching/building this branch:
`cpp_astar_planner`, `cpp_route_follower`, and `cpp_flight_adapter`. Their configs
record `localization_pairing=exact_source_stamp` and
`path_stamp=localization_source_stamp`. Existing launch commands can be used with
the C++ planner/follower and adapter selected. An older/legacy producer that stamps
paths at publication time does not satisfy this contract; it will not produce a
usable route for these consumers. Legacy Python implementations are unchanged.
The separate 3D snapshot protocol is unchanged.

This fixes demonstrated coordinate-consistency defects. It does not eliminate
SLAM optimization corrections, prove independent centreline accuracy, or change
the planner's deliberately curved routes. The second flight's +12.8 cm route
waypoint was not evidence of a planner defect. Planner costs and clearance
margins were therefore preserved.

PX4 gains, EV delay, flight speed and acceleration settings are unchanged.
Position-only emergency fault holds still use their existing stop policy;
this patch does not promise a global position/velocity/jerk bound across every
fault, PX4 estimator reset, or mapping/odometry discrepancy. Weak physical
braking remains unresolved. No hardware flight or parameter write was performed.

## Verification

Package build passed. All **42 CTest targets passed**; `colcon test-result`
reports **777 tests, zero errors/failures, three existing skips**. The new
localization target passes nine C++ cases and the production-node integration
target passes both ROS scenarios. `git diff --check` passed. Build output retains
the existing overlay-path warning and GCC ARM ABI notes.

`test_localization_pair.cpp` covers both callback orders, interleaved and missing
samples, duplicate freshness, rejected/invalid stamps, exact path provenance,
retained-path reframing, and the measured flight's mixed-frame pulse mechanism.
It explicitly demonstrates a >14 cm error from the old mixed inputs and an
invariant physical displacement when the coherent pair is used.

`test_localization_pair_integration.py` runs the production binaries in isolated
DDS domains. A mock PX4 test remaps every FMU topic and separates localization
halves by multiple control ticks; it also sends paths before their complete pair,
delayed older paths, unmatched future-dated paths, and repeated stale localization. A planner/follower test
checks exact source stamps, continued route following, and an obsolete empty
path veto. The mock does not model aircraft dynamics or establish flight safety.

Commands, from `ros2_ws`:

```bash
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
colcon build --packages-select px4_vio_bridge
source install/setup.bash
ROS_DOMAIN_ID=177 colcon test --packages-select px4_vio_bridge --event-handlers console_direct+
colcon test-result --test-result-base build/px4_vio_bridge --verbose
```

The work used one read-only reviewer for path-acceptance/gate interactions; the
lead implemented the changes and ran verification. Build/test transcripts are
local artifacts at `/tmp/non_braking_build.log` and `/tmp/non_braking_tests.log`.
