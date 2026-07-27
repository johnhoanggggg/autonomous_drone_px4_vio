# Loop-Closure Correction — Parked Handoff

Split out of `HANDOFF.md` on 2026-07-27 so the main doc stays focused on flight
bring-up. **Nothing here is wired into PX4.** The correction node is observation
only; EKF2 still runs on continuous VIO exactly as it always has, so this work
can sit untouched indefinitely without affecting flight.

**Status: observation stage complete and measured. The A/B design decision is
open and is the thing to pick up first.** See "Where this stands" at the end.

---

## The mechanism

New node `map_correction` (`px4_vio_bridge/map_correction.py`). It pairs
`/rtabmap/vio_pose` with `/rtabmap/pose` by timestamp, solves the correction
that carries one onto the other, and slews an *applied* copy of that correction
toward the target under a rate limit — so a loop closure arrives as a drift
instead of a jump. **Nothing it publishes reaches PX4.** EKF2 still runs on raw
continuous VIO exactly as before.

The transform is 4-DOF (x, y, z, yaw). Roll and pitch are deliberately dropped:
EKF2 anchors attitude to gravity and a tilt correction through external vision
is never wanted.

| topic | meaning |
|---|---|
| `/vio/map_correction` | applied (rate-limited) correction, `PoseStamped` in `world` |
| `/vio/map_correction_target` | where it is heading, i.e. the raw SLAM solution |
| `/vio/map_correction/preview_pose` | the pose PX4 *would* get if this were wired in — overlay against `/rtabmap/vio_pose` and `/rtabmap/pose` |
| `/vio/map_correction/residual_m`, `/residual_deg` | how much correction is still pending |

Defaults: `translation_rate` 0.03 m/s, `yaw_rate_deg` 1.0, gates
`max_correction_m` 0.5 and `max_correction_yaw_deg` 15 (a correction bigger than
the gate is a fault to log, not something to ramp through for 30 s), deadband
1 cm / 0.2 deg so the target stops chasing SLAM re-optimization jitter. A VIO
reset sentinel clears the pairing history and freezes the correction — the odom
frame origin has moved, so every pairing solved before it is meaningless.
Launch args `map_correction`, `map_correction_translation_rate`,
`map_correction_yaw_rate_deg` on `rtabmap_slam_px4.launch.py` (node enabled by
default; the topics are in the Foxglove whitelist).

Verified end to end against a synthetic 20 cm closure: ramp measured 0.0304 m/s,
converged in 6.7 s (= 0.20 / 0.03) with no overshoot and residual settling to
exactly zero. `colcon test` 26 tests, 0 failures.

**Why the translation rate is the number that matters.** It is the fake velocity
the ramp would inject if this is ever wired into EKF2, so it has to stay small
against EKF2's velocity noise. If the injection path is built, velocity must
keep being finite-differenced from the *uncorrected* VIO track, or the ramp
becomes a fabricated velocity that EKF2 will happily fuse. `reset_counter` must
also not tick on corrections — that field means VIO reset.

**Not built, and the decision that gates it.** [offboard_hover.py:403](ros2_ws/src/px4_vio_bridge/px4_vio_bridge/offboard_hover.py#L403)
latches `(x0, y0)` at arm and holds it forever. So injecting the correction into
the estimator makes the vehicle *physically translate* by the correction, flying
to keep a now-stale setpoint. For a pure hover-hold, correcting the goal instead
of the estimate is mathematically a no-op. Injection only earns its risk once
something hands PX4 setpoints in map coordinates that do not know about the
correction. Decide after the measurements below.

## Observation runs

**Run 1 (`flight_logs/maploop_20260727T111207Z`) — inconclusive, redo needed.**
62 s handheld. Analysed with `scripts/analyze_map_correction.py` (new; `analyze_flight.py`
cannot read these bags, it requires an armed vehicle).

- **VIO tracking loss t=41.20–44.91 s**, 47 reset-sentinel samples. The node handled it
  correctly — all 47 rejected, history cleared, correction frozen — but it makes the run
  suspect. Features hit a minimum of 16 against the 160 floor.
- **No loop closures.** The two candidate events at t=45.16/45.24 land 0.25 s after VIO
  recovered; they are the correction re-solving against the restarted odom frame, not
  closures.
- **The loop was too small.** Max distance from start only **1.64 m** (path 4.99 m) — pacing,
  not a room-scale revisit. VIO has to drift before a closure can correct anything.
- **Useful number that did come out: the noise floor.** Stationary p99 of the per-sample
  target change is **1.19 cm / 0.43 deg**. The steady correction settled around 6 cm / 4 deg.
  Gates (0.5 m / 15 deg) never came close. An earlier eyeball estimate of ~3 cm noise from a
  5 s live probe was wrong — that sample caught motion, not rest.
- VIO ran at **9.2 Hz**, below the ~13.4 Hz the handoff records; `slam_publish_clouds:=true`
  was on. Turn clouds off for observation runs — they are XLink load this measurement does not
  need, and may have contributed to the dropout.

**Run 2 (`flight_logs/maploop_20260727T112219Z`) — GOOD, and it settles the
question.** 137 s handheld, clouds off, VIO 13.2 Hz (vs 9.2 Hz with clouds on — turning them
off fixed the rate), **no tracking loss**, 18.46 m path, 5.84 m max extent, returned to within
0.54 m of start. Features median 269, min 91.

| | value |
|---|---|
| noise floor (stationary p99, per sample) | **0.81 cm / 0.31 deg** |
| correction magnitude, p95 / max | **17.7 cm / 33.9 cm** |
| largest single-sample jump | **20.7 cm** |
| yaw correction, max | 3.80 deg |
| final correction after 18.5 m walked | 17.5 cm, 2.95 deg |

**Loop closure in this room is 20–40x the noise floor, so it is unambiguously real and the
smoothing mechanism is justified.** VIO drifted ~17.5 cm over 18.5 m, about 1 % of path length.
A 40 cm circle would have shown none of this — the drift has to accumulate first.

The 142 per-sample jumps over 4 cm cluster into **14 distinct episodes**, and they come in two
very different shapes:

- **Sustained churn.** Episode t=80.6–95.4 s ran 14.8 s with a largest jump of 20.7 cm but a
  *net* move of only 6.9 cm — the target oscillated and largely came back. A rate limiter
  handles this well; it averages the oscillation out instead of dragging the vehicle around.
  This is an argument **for** the ramp design.
- **A clean discrete closure.** At t=124.78 s the target stepped **17.8 cm in a single sample
  while the carrier was stationary**. That is exactly the failure mode the ramp exists for: raw,
  it is a 17.8 cm instantaneous jump with the vehicle sitting still.

**Two design consequences to act on before any injection work:**

1. **`max_correction_m=0.5` is too tight.** Observed max was 33.9 cm — only 1.5x margin. A
   bigger room or longer flight will exceed it.
2. **Worse, the rejection behaviour is wrong.** An over-gate candidate is refused and the
   *previous* latched target is kept, so if drift keeps growing every later candidate is also
   refused and the correction freezes permanently stale. Clamping to the gate and warning is
   strictly better than refusing forever.

Also note 33.9 cm is comparable to `max_horizontal_error` (0.35 m) in `offboard_hover` — a raw
closure injected in flight could trip the safety abort on its own.

## Where this stands (2026-07-27)

**Done.** Node built, unit tested (12 tests, part of the package's 26), verified against a
synthetic 20 cm closure (ramp 0.0304 m/s, converged in 6.7 s, no overshoot), wired into
`rtabmap_slam_px4.launch.py`, and **measured for real** on a 137 s handheld loop. The
reset-sentinel freeze path was exercised by a genuine VIO tracking loss and worked. The
measurement above is the deliverable: corrections here are 10–34 cm against a 0.8 cm noise
floor.

**Open, in order, when this is picked back up.**

1. **Fix the clamp-vs-reject defect** (see the two design consequences above). An over-gate
   candidate currently freezes the correction permanently and silently. Raise
   `max_correction_m` 0.5 → ~1.0 and clamp-with-warning instead of refusing. This is worth
   doing whichever design path is chosen.
2. **Decide A vs B**, now that the magnitudes are known:
   - **A — inject into EKF2.** The correction ramps into the estimator, and the vehicle
     physically translates by it because [offboard_hover.py:403](ros2_ws/src/px4_vio_bridge/px4_vio_bridge/offboard_hover.py#L403)
     latches `(x0, y0)` at arm. Given observed corrections reach 33.9 cm against a 0.35 m
     `max_horizontal_error` abort, this is the risky path. If taken: velocity must keep being
     finite-differenced from the *uncorrected* VIO track, and `reset_counter` must not tick on
     corrections.
   - **B — correct the goal, not the estimate.** EKF2 stays on continuous VIO and never sees a
     discontinuity. Safer, and the measured data makes it look better than it did. But it is a
     no-op until there are map-frame waypoints to correct — for a pure hover-hold it does
     nothing.
3. **Only if A:** validate in flight with the synthetic injection hook first —
   `ros2 param set /map_correction inject_translation "[0.1, 0.0, 0.0]"` steps the target on
   demand, so the ramp can be exercised at hover without waiting for a closure.

**How to re-run the measurement.** Launch *without* clouds (`slam_publish_clouds` costs ~4 Hz
of VIO rate and is not needed):

```bash
ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py slam_num_features:=400
ROS_DOMAIN_ID=42 python3 scripts/probe_map_correction.py     # live view
ros2 bag record -o flight_logs/maploop_$(date -u +%Y%m%dT%H%M%SZ) \
  --storage mcap --storage-preset-profile fastwrite \
  /rtabmap/vio_pose /rtabmap/pose /rtabmap/vio_feature_count \
  /vio/map_correction /vio/map_correction_target \
  /vio/map_correction/residual_m /vio/map_correction/residual_deg
python3 scripts/analyze_map_correction.py ros2_ws/flight_logs/<bag>
```

Walk 5–10 m of *extent* and return to the same spot in the same orientation. A 40 cm circle
shows nothing — VIO drift has to accumulate first.

**Files.** `px4_vio_bridge/map_correction.py`, `scripts/map_correction` (wrapper),
`test/test_map_correction.py`, `scripts/probe_map_correction.py` (live readout — the ROS 2 CLI
`topic echo` is unreliable here), `scripts/analyze_map_correction.py` (bag analysis;
`analyze_flight.py` cannot read these bags, it requires an armed vehicle).

