# Trajectory smoothness and wall strikes — handoff, 2026-09-07

> **September 16 separate branch:** [correction/route-frame consistency fix](NON_BRAKING_FIX_20260916.md).
> The 2D C++ planner, follower and adapter pair localization by exact source stamp
> and preserve each path's source correction. Restart all three nodes together;
> older publication-stamped paths are incompatible. This fixes reference pulses,
> not the unresolved physical braking response. No hardware flight performed.

> **Follow-up, 12:56/12:57 flights:** [retreating-target braking fix](CLUTTER_TRACKING_FOLLOWUP_20260913.md). Full-vector command improvement is visible in both logs; straight route plus wallward actual drift remains. Fixed feasible braking being rejected when a soft target retreats. No gain, margin or speed changes.

> **Latest work, 2026-09-13:** [strict command-vector fix and unresolved physical response](CLUTTER_TRACKING_FIX_20260913.md). C++ global adapter enables strict vector acceleration limits and stops at unrounded corners; joins cannot snap small offsets. Added and loaded estimator diagnostic topics over NSH. No further gain/tolerance changes or physical tests. This is not demonstrated tunnel readiness.

> **225 ms trial flown, 2026-09-13 12:11:** [results](ros2_ws/flight_logs/offboard_lateral_stop_20260913T121110Z/ANALYSIS.md). Three legs, third hold timed out; longest joint gate pass 0.848 s at 0.06 m/s versus 1.598 s at 0.08 m/s. Third-leg initial overshoot 9.7 cm, hold swing 16.3 cm, first reversal 2.03 s after brake marker. Reduced overspeed is not proof of corrected braking. EV delay stays 225; no further writes.

> **Current trial, 2026-09-13:** [EV_DELAY_TRIAL_20260913.md](EV_DELAY_TRIAL_20260913.md).
> Existing smooth lateral-stop tests establish oscillation independent of the
> planner. Four recent ULogs place residual VIO/gyro timing at 205–230 ms, versus
> configured 270 ms. Saved **EKF2_EV_DELAY=225 ms** while disarmed; readback
> verified. All PID gains stay unchanged, roll P=0.150. Power-cycle before
> repeating the same open-area lateral-stop test. No flight started; braking
> improvement is not yet demonstrated. Historical 270 ms statements below are
> superseded for the current native-VIO trial only.

> **Yaw/motor investigation:** [YAW_MOTOR_DIAGNOSIS_20260908.md](YAW_MOTOR_DIAGNOSIS_20260908.md).
> Reconstructed yaw P+I and 2 Hz filter agree with torque; diagonal motor split
> agrees with allocation. Net gyro/attitude yaw changes agree within ≈1°, without
> ruling out timing/transient estimation errors. MAIN PWM 400 Hz, matching ranges;
> no ESC/RPM telemetry. User reports BLHeli 30 A, 1700 KV, 6-inch props. Prepared
> a two-second low-output test and expected motor mapping; **no motors spun**.
> Physical confirmation that all props are removed is needed before testing.
> Roll P remains 0.150; no further control or ESC settings changed.

> **Roll-P trial analyzed; restored baseline:**
> [ROLL_RATE_TRIAL_RESULTS_20260908.md](ROLL_RATE_TRIAL_RESULTS_20260908.md).
> Trial `050015Z`, SD `20_51_29.ulg`, verified P=0.165. Moving roll-rate RMS
> improves 3.75→3.02°/s, but third overshoot remains 11.6 cm and final stationary
> hold spans 26.7 cm for ≈10 s before LAND. Yaw integral peaks 0.161. ROS tail
> is truncated; SD recovers the final stop. Calibration offsets also differ.
> Restored **MC_ROLLRATE_P=0.150** over USB while disarmed, readback verified.
> No further gain increase; yaw compensation/propulsion and estimator timing
> remain unresolved. Terminal heading stays deferred.

> **Current PX4 tuning trial, 2026-09-08:**
> [ROLL_RATE_TUNING_20260908.md](ROLL_RATE_TUNING_20260908.md): user requested
> tuning; changed only `MC_ROLLRATE_P` **0.150 -> 0.165 (+10%)** over USB while
> disarmed. Readback confirms the value is saved. No flight started. Next is the
> same open-area lateral sequence with a charged pack; evaluate roll tracking,
> stopping/hold excursions and yaw compensation before retaining the change.
> Baseline snapshots and rollback command are in that note. Earlier statements
> below that gains were unchanged describe the preceding baseline flights.

> **Lateral test flown and analyzed:**
> [LATERAL_STOP_RESULTS_20260908.md](LATERAL_STOP_RESULTS_20260908.md): bag
> `043031Z`, paired SD `20_22_29.ulg` (5,970,027 B, exact size verified).
> Four legs completed. Third leg 0.316 m/s vs 0.20 reference, 12.2 cm initial
> overshoot. Second stationary hold spans 25.8 cm laterally. Allocator reports
> all thrust/torque achieved, no output clipping; yaw integral grows to 0.135
> with fixed heading and increasingly split motor commands. This is not proof
> of a specific bad motor or battery cause. Inner response/propulsion and
> estimator timing remain the priorities; terminal heading stays deferred.

> **Next flight prepared:** [LATERAL_STOP_TEST_20260908.md](LATERAL_STOP_TEST_20260908.md)
> has the pasteable command for fixed-heading right/return/left/return, 35 cm
> each leg, 0.20 m/s peak, smooth braking and measured settling gates. New
> `offboard_lateral_stop` is built/installed; 17 new tests plus inherited hover
> checks pass (aggregate 620 tests, no errors/failures, 3 skips). No flight was
> launched by the preparation.
> PX4 logger now loads 26 custom topics, including rate integrals, allocator
> status, battery and actuator outputs; configured ESC status depends on
> hardware publication. Logger was restarted disarmed; no gain changes.

> **2026-09-08 runs 2/3 priority:** Terminal heading work is deferred by the
> user. [TUNNEL_BRAKING_20260908.md](TUNNEL_BRAKING_20260908.md) traces roll-rate
> braking response and documents a fixed acceleration-limit bypass when
> entering `DEGRADED`. The cap now decelerates from 0.20 to 0.06 m/s over ten
> 50 ms ticks. Wallward drift begins before this transition; the patch is not
> proof of resolved wall strikes. Actual-velocity stopping clearance and full
> vector continuity through joins/corners remain open. No PX4 gains changed.
> Built/installed; **603 tests, 0 errors, 0 failures, 3 skipped**. New offline
> analyzers: `scripts/analyze_command_continuity.py` and
> `scripts/analyze_rate_loop.py`.

> **2026-09-08 heading implementation:**
> [HEADING_CONTINUITY_20260908.md](HEADING_CONTINUITY_20260908.md) traces the
> 095742Z replacement-leg/vertex reversal and documents the new C++ heading
> preview and persistent reference. Recorded-state shadow removes that target
> excursion; 602 package tests pass with 3 skips. This is not flight validation
> or proof that the braking/clearance issue is solved.

> **Superseded diagnosis / follow-up:** Read
> [HOVER_TURN_20260907.md](HOVER_TURN_20260907.md) for the subsequent open-area
> test: a fixed-position hold reproduced poor rate tracking and growing yaw
> compensation, with low recorded loaded voltage. The new control topics are
> now captured and analyzed. Then read
> [WALL_STRIKES_20260907.md](WALL_STRIKES_20260907.md). Onboard logs show
> 0.419 m/s actual speed against only 0.141 m/s PX4 demand at the peak; the
> proposed 0.3 m/s cap would not bind this event. The offboard acceleration
> limit and instantaneous-yaw-freeze claims below are also corrected there.
> A tested early yaw slowdown is now built (610 tests, no failures); the
> control-response/hold-drift issue still needs physical validation. PX4 gains
> and speed limits remain unchanged. Three missing control topics were added
> to SD logging for the next PX4 boot. The historical handoff follows intact.

Branch `enclosed-spaces-tuning`. Everything below is C++ (`cpp_mode:=true`);
`planner_flight.py` is legacy and is deliberately NOT kept in step — the C++
join has diverged from it and the parity test still passes because its scenarios
do not reach the diverged branch.

## State of the tree

`334b28a "more confident flying"` carries the heading fix, the join cap and its
escalation, `min_vio_features` 75, and the escape-while-held. **Uncommitted** is
the follower-dropout ride-out: `follower_latch_grace` / `degraded_speed`, the
`speed_cap` argument on `PathCommandLimiter::update()`, the `DEGRADED` status,
their launch arguments, three regressions, and the last section of
`CORNER_WEAVE_20260906.md`.

`colcon test`: **607 tests, 0 errors, 0 failures, 3 skips.**

Build and test from the WORKSPACE ROOT (`ros2_ws/`). Running colcon from
`src/px4_vio_bridge` half-succeeds and reports a stale 504-test result — that
cost me three wasted cycles.

## What changed, and what each change is actually worth

Read `CORNER_WEAVE_20260906.md` for the full derivation; this is the summary.

| change | evidence it works |
|---|---|
| heading from the route tangent, not the tick's velocity (`preview_direction`) | at-rest preview NED 90.0° at every offset, was 6.4° at 3 cm |
| join lateral capped to `tan15° × along_speed` | in flight, joining median 30° → **15.0°**, >20° share 42% → 2.7% |
| escalation past `max_projection_error_` | command offset median 0.035 → 0.000–0.004 m |
| `min_vio_features` 150 → 75 | 135902Z and 140117Z spent 0.68 s and 0.42 s under 150 (watchdog trips at 0.25 s) and would both have landed; neither spent 0.15 s under 100 |
| escape-while-held | **never triggered in flight.** Unexercised |
| follower ride-out (`DEGRADED`) | 8 ride-outs in 20260907T080022Z; not yet measured against stop counts |

Two failed designs, recorded so they are not retried:

- **A merge-*time* bound** (`lateral ≥ offset / 1 s`) instead of the escalation.
  An absolute rate dominates the direction wherever the along-path rate is
  small, which is the failure the bound exists to prevent. Replayed p90 went
  straight back to 22–75°.
- **An unscoped lateral floor** (`max(floor, ratio × along)`). Out of a brake
  one tick of acceleration is the whole along-path rate, so a 0.02 m/s floor
  reads as 53° = atan(0.02/0.015). The floor is now used only when
  `along_speed` is exactly zero.

## The open problem: it hits the right wall

Two strikes, in `20260907T080337Z` (|a_xy| 6.2 m/s², t=22.45) and
`20260907T080022Z` (4.6 m/s², t=26.62). The 080337Z one is fully resolved:

| t | lat vs path | heading | course | cmd dir | nose−course | speed |
|---|---|---|---|---|---|---|
| 21.45 | +0.043 | 108° | 110° | 107° | +2° | 0.26 |
| 22.11 | −0.008 | 111° | 170° | 116° | +59° | 0.40 |
| 22.21 | −0.021 | 112° | 173° | 124° | +61° | 0.41 ← YAW_ALIGN |
| 22.41 | −0.055 | 116° | −180° | 139° | +64° | 0.40 |
| 22.51 | −0.065 | 119° | +70° | 142° | −49° | 0.22 ← impact |

1. **Twice the commanded speed.** 0.42 m/s against 0.20. Read over NSH:
   `MPC_XY_VEL_MAX` 1.0, `MPC_ACC_HOR_MAX` 5.0, `MPC_XY_P` 0.95,
   `MPC_XY_VEL_P_ACC` 1.8 / `_I_ACC` 0.4 / `_D_ACC` 0.2, `MPC_JERK_AUTO` 4.0.
   PX4 may use 5× the cruise and 17× the acceleration to chase the setpoint.
   At the planner's 0.30 m/s², 0.42 m/s needs 0.29 m to stop; the corridor gives
   0.30–0.46 m.
2. **Yaw cannot keep up with the bend.** Course rotates ~70° in a second, yaw
   slews at 20°/s, so the nose ends 64° off the direction of travel.
3. **The freeze is late and abrupt.** YAW_ALIGN triggers at 40° of error and cuts
   translation to zero, which removes steering without removing momentum. Lateral
   goes +0.021 → −0.065 m across the freeze: it slides right, into the wall.

Ruled out with data: trim (roll mean −0.27°/−0.33°), the planner routing near the
wall (lateral error unbiased, 49%/45% of samples each side, never worse than
7 cm), and map jumps (the correction was constant through the whole event).

**Unresolved link.** The actual course over-rotates well past the commanded
direction (173° vs 124° at 22.21). Best inference is that PX4 tracks the
position setpoint, not the feedforward: the vehicle had fallen behind, the
setpoint went round the corner, and the position-error vector rotates the course
faster than the feedforward direction. **This is reasoning, not measurement.**
Confirm it by comparing demanded against achieved acceleration through a bend
before acting on it.

## What to do next, in this order

1. **Cap PX4 to something near the mission.** `MPC_XY_VEL_MAX` 1.0 → ~0.3 and
   `MPC_ACC_HOR_MAX` 5.0 → ~1.0, over `scripts/nsh.py` (needs
   `.venv-mavlink/bin/python3`, pymavlink is not in the system env). **Not
   applied — the user has not approved it.** It also clamps position-mode flying
   generally, which is the trade to raise with them. This is the prerequisite for
   step 2 being safe.
2. **Stop freezing translation dead for a yaw alignment.** Either raise the slew
   from 20°/s or let translation continue at a low cap — the `speed_cap`
   argument added for the ride-out is the mechanism, already tested. Do not do
   this before step 1: freezing is what contained the 0.4 m/s momentum, badly,
   and removing it without capping speed makes corners faster, not safer.
3. **Measure the ride-out.** It has flown but its effect on stop count and median
   speed is unmeasured. Baseline to beat: 135902Z and 140117Z had 25 and 19 stops
   in 32.8 s and 26.5 s, median speed 0.147 and 0.122 against 0.206 and 0.193
   while in ROUTE.
4. Left alone deliberately: the ~2 Hz re-anchoring that opens most joins,
   `junction_deviation` (no corner under ~90° brakes at all), and the fact that a
   latched fault still clears the route limiter.

## Safety envelope changes made today

- A follower dropout no longer holds immediately: worst case from dropout to
  landing is now `follower_latch_grace + planner_fault_land_time` = **7.5 s**,
  where it was 6.0 s. Set `follower_latch_grace` to 0 to restore the old
  behaviour exactly.
- A fault hold may now move the aircraft, bounded: only in ROUTE, armed, only
  below the required clearance, ≤ 0.50 m of travel, inside the geofence, through
  the existing chord-admissibility rule, land countdown still running.

## Reproducing the analysis

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
# geometry replay of the heading/join behaviour over recorded flights
python3 scripts/replay_command_heading.py ros2_ws/flight_logs/offboard_global_2026090{6,7}T*Z
# occupancy + trajectory render; writes overview.png into the bag folder
python3 scripts/render_flight_map.py <bag>/<bag>_0.mcap -o <bag> --supersample 1
```

Pass the `.mcap` path, not the directory, for any bag whose `metadata.yaml` is
truncated (a killed flight) — `render_flight_map.py` reads the directory and
fails on those, while `analyze_flight.resolve()` handles both.

The ad-hoc measurement scripts (side clearance, drift decomposition, phase
timing) were written to the session scratchpad and are gone. They are small;
`scripts/analyze_yaw_tracking.py` and `scripts/analyze_flight.py` are the
reusable pieces, and `RecentSamples(...).at(stamp, max_age)` is the causal
receipt-time pairing every one of these analyses uses.

## Traps

- `/planner/follower/vio_displacement` **keeps publishing at 10 Hz through a
  follower dropout**, contradicting the comment in `planner_health_reason()`
  that says it stops. Dropouts ran 1.0–2.8 s across eight measured episodes; the
  staleness checks never bound them.
- Corner blending means `junction_deviation` 0.05 with 0.30 m/s² lets everything
  under ~90° through at full cruise. Corner braking effectively does not exist.
- `join_tolerance_` is 3 mm, so nearly every republished path opens a rejoin.
  `suffix_tolerance` (0.01) is what decides whether a shifted path counts as the
  same geometry instead — offsets below it never open a join at all, which
  invalidates any test that tries to exercise a 5 mm join.
