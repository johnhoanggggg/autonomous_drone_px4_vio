# Autonomous Drone PX4 VIO — Handoff

Authoritative current-state doc. Full chronological history is in `HANDOFF_ARCHIVE.md`
(kept for forensics; some of it is superseded — trust this file).

Side-work lives in its own docs: `HANDOFF_LOOP_CLOSURE.md` (SLAM loop-closure
correction — built, measured, not wired to PX4), `HANDOFF_VFH.md` (VFH2D —
built, never armed, and parked), and `HANDOFF_GLOBAL_PLANNER.md` (global A*
monitor — built and simulator-verified, not wired to PX4).

Last updated: 2026-08-04.

## What this project is

Raspberry Pi 5 + OAK-D Lite + Pixhawk 4 (PX4 v1.17.0). OAK-D **RTAB-Map VIO/SLAM** →
`/rtabmap/vio_pose` (continuous VIO pose) → `vio_to_px4_odometry` bridge → PX4
`/fmu/in/vehicle_visual_odometry` over uXRCE-DDS on TELEM2. EKF2 fuses external vision
for horizontal position, height, and yaw (no GPS). Goal: indoor autonomous flight.

- Project: `/home/john/autonomous_drone_px4_vio`
- Bridge pkg: `ros2_ws/src/px4_vio_bridge` (ROS 2 Jazzy, `ROS_DOMAIN_ID=42`)
- px4_msgs from `/home/john/ros2_ws/install`
- VIO script: `scripts/rtabmap_vio_ros2.py` (also `oak_d_vins_cpp/.../basalt/` — Basalt is a dead end here, too timing-fragile)

## Immediate handoff status (2026-07-26 / 07-27 — read before doing anything)

**2026-07-27 in one paragraph.** Two new autonomous modes were built and both flew
successfully on their first armed attempt: `offboard_waypoint` (Foxglove
click-to-fly, 4 waypoints) and `offboard_square` (scripted 0.40 m square with four
90 deg turns). Details in the two FLOWN sections below. Also added `battery_to_ros`
(battery on a Foxglove gauge) and taught `analyze_flight.py` to analyse waypoint
flights. **Two things need attention before the next flight: the battery was run to
2% SoC, and the landing bounce is getting worse (178 deg/s roll at contact) — both
are in Open Risks.** The 07-26 milestone below still stands unchanged.

**MILESTONE 2026-07-26: the yaw-torque bias is FIXED and confirmed gone.** The
operator changed the motor/prop configuration so that diagonally opposite props
now spin in the *same* direction (previously opposite props were spinning in
opposite directions — the mechanical defect the 2026-07-25 ULog analysis
predicted). This matches PX4's mixer geometry (`CA_ROTOR0/1_KM=+0.05` for
FR/RL, `CA_ROTOR2/3_KM=-0.05` for FL/RR) and the improvement is unambiguous.

Bag `offboard_hold_yaw_20260726T033215Z`, ULog `flight_logs/px4_03_30_57.ulg`
(retrieved over USB MAVLink FTP; PX4 clock ran ~78 s behind the Pi, so
`03_30_57.ulg` = the `033215Z` bag). Launch used was
`ros2 launch px4_vio_bridge offboard_hold_yaw.launch.py auto_arm:=true`.

Yaw torque setpoint, airborne steady state, normalised to ±1 full authority:

| ULog | τ_z mean | τ_z min | % samples negative | motor pair ratio |
|---|---|---|---|---|
| `px4_04_26_06` (07-25) | +0.1505 | −0.016 | 4.6 % | 2.18× |
| `px4_04_37_47` (07-25) | +0.1206 | +0.008 | 0.0 % | 1.85× |
| **`px4_03_30_57` (07-26)** | **+0.0040** | **−0.017** | **36.1 %** | **1.02×** |

A 30× reduction, and τ_z is now negative a third of the time — the allocator
drives both directions instead of leaning on one pair. Motor means are
0.381 / 0.341 / 0.359 / 0.347 (was 0.557 vs 0.256 per spin-direction pair).
The residual yaw trim (+0.004) is now **smaller** than the roll (−0.010) and
pitch (+0.018) trims, i.e. it is at noise level. **Do not spend more time
looking for an external yaw torque — there isn't one.**

Flight behaviour agrees:

- Yaw hold error peak-to-peak **1.96 deg** (YAW_HOLD) and **2.09 deg**
  (RETURN_HOLD). Was ±10–12 deg.
- Yaw gyro rms **3.66 deg/s**, peak **8.9 deg/s**. Was rms 8.7–32, peak 24.6–101.
- Steady-state yaw errors **flip sign** between the two holds (−1.34 deg out,
  +1.75 deg back). A torque bias produces the *same* sign both ways; opposite
  signs is plain proportional droop.
- Full sequence completed with zero safety triggers. Climb reached 0.30 m
  0.49 s after liftoff, overshot to 0.380 m, held 0.32–0.37 m. VIO clean
  (features median 305, min 220, no reset sentinels). Battery at 58 %.
  The K-press at t=22.47 s was on the ground (alt had been 0.043 m since
  t=21.0 s) — benign.
- Only leftover yaw item: slew achieves 4.18 / −3.85 deg/s against a 5.0 deg/s
  command — symmetric lag. Now that the bias is gone, this is the case for
  restoring `MC_YAWRATE_I` 0.04 → 0.1 (it was lowered only to cope with the
  old bias). Do this **after** the level calibration below.

### The left drift: roll level reference was ~4 deg out — SOLVED, flight-verified

This is what the operator observes as "**drifts left on ascent**". It is a
separate, pre-existing problem and has nothing to do with props or yaw.

Heading was 90 deg (east), so +North = body-**left**. The vehicle moved
**+0.30 m north = 0.26 m body-left** during the climb and then held there.

Cause:

- Roll sitting on the ground, pre-arm: **+3.95 deg**, spread only 0.04 deg
  across 703 samples.
- Roll in stable hover, velocity ≈ 0.01 m/s: **+3.66 deg**.
- Roll *torque* trim only **−0.010** — the airframe is balanced; nothing is
  being fought.

A vehicle hovering at zero velocity must have its thrust axis vertical.
Reporting +3.66 deg while doing so means the **reference** is wrong by that
much, not the airframe. On liftoff the controller holds "roll = 0" = about
4 deg physically left-side-down → 0.68 m/s² leftward → it slides left until the
position error commands enough tilt to cancel the offset, then parks there.
Predicted steady offset ≈0.3 m; observed 0.26 m.

The offset has been present all along and is **growing** (ground roll vs
airborne roll, from the bags):

| bag | ground roll | air roll |
|---|---|---|
| `20260721T120047Z` | +2.59 | +1.76 |
| `20260725T033528Z` | +2.08 | +1.31 |
| `20260725T043911Z` | +3.15 | +2.75 |
| `20260725T063122Z` | +4.14 | +4.13 |
| `20260726T033215Z` | **+3.96** | **+3.65** |

Air roll tracks ground roll roughly 1:1 — the signature of a reference offset,
not an aerodynamic force. `SENS_BOARD_X_OFF` is already **−7.2275 deg** from
some earlier level calibration and `CAL_ACC0_YOFF` is only +0.077, so this is a
stale board/mount offset (creeping soft mount, or that calibration was done on
a slope).

**SOLVED AND FLIGHT-VERIFIED 2026-07-26 05:10.** Two attempts were needed; the
first failed because of the USB-cable gotcha below.

- **Attempt 1 (04:11, QGC level horizon):** `SENS_BOARD_X_OFF` −7.2275 →
  −4.8028. Barely helped — hover roll only fell +3.66 → +2.98/+2.67 and the
  drift stayed at ~0.22 m left. The calibration was run over USB **with the
  cable pulling the FC over on the roll axis**, so it froze the cable tilt into
  the aircraft.
- **Attempt 2 (05:0x, QGC level horizon with the cable routed so it no longer
  tilts the FC):** `SENS_BOARD_X_OFF` → **−1.8316**, `SENS_BOARD_Y_OFF` →
  **+1.3140**. This one worked.

Flight `offboard_hold_yaw_20260726T051050Z` / `flight_logs/px4_05_09_17.ulg`:

| | before (`04_36_46`) | after (`05_09_17`) |
|---|---|---|
| hover roll | +2.67 deg | **−0.46 deg** |
| implied lateral bias | +0.46 m/s² | **−0.078 m/s²** |
| steady hover offset | **0.200 m LEFT** | **0.020 m right** |
| max horizontal excursion | 0.231 m | **0.099 m** |

The drift is gone — steady offset down 10x, max excursion down 57% and now
comfortably inside the 0.15 m pre-yaw gate. Residual −0.46 deg is not worth
chasing.

**Board-offset gain is 1.10 deg of reported roll per deg of `SENS_BOARD_X_OFF`,
confirmed twice**: on the bench (a +3.0 param step moved roll −0.1 → −3.4 deg,
applied live with no reboot) and in flight (+2.97 param step moved hover roll
+2.825 → −0.46). Use ~1:1 for any future correction. (The apparent gain of 0.34
between the −7.2275 and −4.8028 flights was never explained; it is moot now that
two clean measurements agree, and is most likely an artifact of attempt 1 having
been calibrated against a cable-tilted vehicle.)

**Params do persist.** `SENS_BOARD_X_OFF` read back as −1.8316 after a full
power cycle. An earlier note in this file speculated that the calibration was
being lost across reboots because of `param show` marker symbols — that was
wrong; PX4 autosaves. Do not read save state from those markers.

### Required next-session sequence (2026-07-26)

1. ~~Re-run the level calibration.~~ DONE, twice — see above.
2. ~~Re-fly and confirm the left offset collapses.~~ DONE 2026-07-26 05:10:
   0.200 m left → 0.020 m right.
3. ~~Set `MC_YAWRATE_I` 0.04 → 0.1 and re-fly.~~ DONE 2026-07-26 05:23, combined
   with the first 45 deg yaw test — see below.

### 45 deg yaw test + `MC_YAWRATE_I=0.1` (2026-07-26 05:23) — full success

Bag `offboard_hold_yaw_20260726T052349Z`, ULog `flight_logs/px4_05_23_55.ulg`.
Launched with `yaw_angle_deg:=45.0 yaw_timeout:=14.0`. Complete
climb/yaw-out/hold/yaw-back/hold/land sequence, **zero safety triggers**;
heading swept 100.3 → 145.8 → 98.5 deg.

**The integral gain did exactly what it was raised to do.** Steady-state droop
at the yaw-out hold, comparing like for like:

| | `MC_YAWRATE_I` | hold error mean | hold p2p |
|---|---|---|---|
| 15 deg test (`05_09_17`) | 0.04 | −0.47 deg | 1.72 deg |
| **45 deg test (`05_23_55`)** | **0.10** | **−0.00 deg** | 1.69 deg |

Droop eliminated on a manoeuvre three times larger. Whole-flight yaw error p2p
only grew 5.08 → 5.82 deg for 3x the angle.

Yaw plant is now as clean as it gets: τ_z mean **−0.0002**, 53.8 % negative,
spin-pair ratio **1.001x**, peak yaw rate 13.2 deg/s against a 60 abort.
Peak motor 0.547 — the lowest recorded, huge margin.

VIO held up through the larger rotation: features median 284, min 184, **zero**
samples below the 160 floor, no reset sentinels, and the 20 deg VIO/EKF yaw
disagreement gate never fired.

Drift stayed controlled: steady hover 0.032 m right, max excursion 0.183 m
(during the yaw itself — rotation coupling, well inside the 0.35 abort).
Hover roll −0.99 deg (ground roll that session was −1.95, so placement differs
run to run; the residual is small and not worth chasing).

**GOTCHA — `yaw_timeout` must scale with `yaw_angle_deg`.** It is a hard
per-leg deadline: `wait_for_yaw()` calls `trigger_landing()` if a leg has not
converged in time. The launch default is **6.0 s**, but each 45 deg leg took
**8.34 s / 8.40 s** at the standard 5 deg/s (the leg ends ~5 deg early on
`yaw_tolerance_deg`, so budget `(angle − tolerance) / rate` plus ~1 s of lag).
At the default this test would have aborted into AUTO.LAND on the first leg.
`max_flight_time` (40 s from arm) was not a constraint — the sequence used ~24 s.

### Corrections to the 2026-07-25 analysis below

- **The "pair imbalance wastes power" claim is wrong.** Hover power measured
  from `battery_status`: 269 W (07-26, balanced) vs 265–277 W (07-25, biased).
  Essentially unchanged. The lower thrust setpoint on 07-26 (0.357 vs 0.403) is
  battery-voltage scaling (11.44 V vs 10.67 V), not an efficiency gain.
- The mechanical-inspection recommendation was **correct** and has now been
  acted on; the props-off inspection item is closed.

### Housekeeping

- Several bags have an empty `metadata.yaml`, so `ros2 bag` / `rosbag2_py`
  cannot open them by directory. Point the reader at the `.mcap` file directly.
  `scripts/analyze_flight.py` resolves this automatically.

### FIXED 2026-07-26: 0-byte flight bags — confirmed in flight

**Confirmed working 2026-07-26 05:10.** `offboard_hold_yaw_20260726T051050Z`
recorded a full **2.48 MB** `.mcap` *and* a 31,663-byte `metadata.yaml` (previous
bags had an empty metadata file even when the mcap was fine). The
`.launchinfo` marker confirmed `storage_preset: fastwrite` from
`ros2_ws/src/.../offboard_hold_yaw.launch.py`. The 043728Z failure that could
not be explained at the time did not recur; the stray duplicate workspace had
been deleted in between, which remains the most likely explanation.


`offboard_hold_yaw_20260725T063349Z` and `offboard_hold_yaw_20260726T041845Z`
were both **0 bytes** — two complete flights lost on the ROS side (the 041845Z
flight had to be analysed from the ULog alone: no VIO feature counts, no bridge
health, no node state machine, no reset-sentinel check).

**Root cause:** MCAP buffers an entire chunk in memory before it touches the
disk, and `--max-cache-size` defaults to 100 MB. A flight bag is ~1.3 MB
compressed, so it never fills a chunk — under the old `zstd_fast` profile the
whole recording lived in RAM until the writer was closed cleanly. Any death that
skips cleanup (terminal closed → SIGHUP, SIGKILL, power loss) left a 0-byte
`.mcap`. The `04-18-45` launch directory's own `launch.log` is also 0 bytes,
confirming that tree died without running any cleanup.

Note this is *not* a "the shutdown path is broken" bug — the clean path works
fine (`launch.log` for the good 033215Z run shows `SIGINT` → `finished cleanly`
in 0.4 s). The bug is that the data was never durable in the first place.

**Fix** (in `launch/offboard_hold_yaw.launch.py`): storage preset `zstd_fast` →
**`fastwrite`** (no chunking, no compression), plus `sigterm_timeout` /
`sigkill_timeout` of 20 s on the recorder so a clean shutdown has room to write
the index and metadata.

Measured by SIGKILLing a live recorder mid-run:

| profile | file after kill | messages recovered |
|---|---|---|
| `zstd_fast` (old) | **0 bytes** | 0 |
| `zstd_fast` + `--max-cache-size 0` | **0 bytes** | 0 |
| **`fastwrite` (new)** | 155 KB | **1442** |

End-to-end with the exact launch command and a SIGHUP+SIGKILL of the whole
process group (i.e. closing the terminal): **872 messages recovered**, where the
old configuration produced nothing.

`--max-cache-size 0` does *not* help and was deliberately not used — the cache
layer was never the bottleneck, MCAP chunking was. **Do not "optimise"
`fastwrite` back to a compressed profile.** The only cost is a larger file, which
is irrelevant at 35 GB free; it also uses less CPU in flight.

`colcon build` + `colcon test` green (14 tests, 0 failures).

**UNRESOLVED: the 043728Z flight was still 0 bytes with the fix in place.** The
`fastwrite` edit landed at 04:32:02 and that flight recorded at 04:37:28, so the
fixed launch file was live. Running the *same* launch file afterwards wrote
425 KB. The 043728Z result is therefore not explained. Two candidate causes were
investigated and neither was confirmed:

1. **A second, stale colcon workspace.** `autonomous_drone_px4_vio/install/`
   (project root, untracked, built 2026-07-25 04:25) shadowed `ros2_ws/install/`
   if its `setup.bash` was sourced, and its launch files were real copies rather
   than symlinks, so they did **not** track source edits. The operator reports
   sourcing `ros2_ws/install`, which is correct and would *not* hit this — so
   this was a latent trap rather than the cause here. Its node code was diffed
   against source and was identical; only launch files diverged.
   **RESOLVED 2026-07-26: the stray root-level `build/`, `install/` and `log/`
   were deleted.** Verified afterwards that `px4_vio_bridge` now resolves only
   to `ros2_ws/install/…`, with `fastwrite` and the run marker present. Rebuild
   with `colcon build` **from inside `ros2_ws`** — never from the project root,
   which is what created the stray workspace.
2. **Keyboard `K` kill.** Ruled out: the 033215Z flight also ended with `K`
   (see its rosout) and recorded a full 1.28 MB bag.

Both failing runs (`041845Z`, `043728Z`) also produced a **0-byte
`launch.log`**, so the launch process tree died without flushing anything. The
good 033215Z run's `launch.log` shows the normal path (`SIGINT` → `finished
cleanly` in 0.4 s).

**Diagnostic added so the next failure explains itself:** the launch now writes
a `<bag_output>.launchinfo` sidecar *before* recording starts, containing the
resolved launch-file path, the storage preset, and a UTC timestamp. It is
written eagerly and survives a hard death. After any future flight — especially
an empty one — read that file first: it says unambiguously which launch file and
which profile actually ran.

### FLOWN 2026-07-27: first armed waypoint flight — SUCCESS

Bag `offboard_waypoint_20260727T121338Z`. Armed, 4 waypoints clicked in Foxglove,
**zero rejections, zero safety triggers, no manual intervention.** Stable hover
reached 2.28 s after CLIMB_HOLD; converged to as close as 0.022 m; typical settled
error 0.08–0.17 m. Yaw was the best on record (error p2p 3.58 deg, peak gyro
7.8 deg/s vs a 60 abort, `yawspeed=NaN` in all 2671 setpoints). Altitude 0.292–0.362 m.
VIO clean: features median 330, min 190, **0 samples below 160** — the lowered 80
floor was never exercised.

**Both watchdog changes were load-bearing.** Measured against the two gates the node
actually applies (`analyze_flight.py` now prints this split):

| phase | n | mean | max | gate | over |
|---|---|---|---|---|---|
| settled | 1892 | 0.135 m | 0.252 m | 0.35 | 0 |
| transit | 611 | 0.229 m | 0.405 m | 0.60 | 0 |

Max distance from the **takeoff latch** was 0.757 m. Under the pre-waypoint logic
(error vs takeoff, single 0.35 m gate) this flight aborts almost immediately; with
`hold_point` re-pointed but no transit gate, the 117 transit samples above 0.35 m
abort it too. Slew limiter exact: max step 5.000 mm across 2504 intervals, 0
violations. **Do not measure slew as a divided speed from bag timestamps** — receive
jitter (2 ms against a 20 ms median) makes an exact limiter look 2.7x over.

**Defect found and fixed: the idle timeout could never fire.** Settled hold error
averaged 0.135 m against `arrival_tol` 0.12 m, so the instantaneous `at_waypoint()`
test flickered false and reset `idle_t` every few ticks — `/waypoint/status` shows
`to_go=0.00m` with `idle=0/300s` right to the end, and the flight had to be stopped
by hand (which is why the bag is truncated before disarm). Arrival is now **latched**
on first touch of `arrival_tol` and cleared only by the next accepted click.

Two other things from that flight:

- **Battery finished at 11% SoC / 10.97 V under 23.9 A**, by far the lowest in the
  record (07-26 flights were 47–58%). This is what prompted `battery_to_ros` below.
- **The bag was truncated** by the hard stop — and `fastwrite` did its job:
  **8.3 MB fully readable** where `zstd_fast` would have left 0 bytes.
- Airborne roll read −2.07 deg (vs −0.46 on 07-26), but that window is full of
  deliberate translation, so it is **not** a clean zero-velocity reference. Fly a
  plain `offboard_hover` hold before concluding the level calibration has drifted.

### FLOWN 2026-07-27: `offboard_square` — square completed on the first armed attempt

Straight/turn/straight/turn/straight/turn/straight/turn. Subclasses
`OffboardWaypoint`, so the slew limiter, `hold_point` re-pointing and the
settled/transit gate split all carry over; only the target source changes. Foxglove
clicks are refused while it runs. Defaults `side_m=0.40`, `turn_deg=+90`, `sides=4`.
Full operating notes in README.

Bag `offboard_square_20260727T125433Z`. **All 4 sides, all 4 turns, zero safety
triggers**, first armed attempt. Headings 93 → −177 → −87 → 3 → **93** (exactly +90
each, back to start heading); corners closed exactly on the start point; all four
sides 0.400 m. 46.7 s for the square, ~11.7 s per side.

**The 15 deg/s turn rate — the main unflown risk — behaved.** Each 90 deg turn took
**6.08 s = 14.8 deg/s actual**, against a 10.0 s timeout (61% used); legs took 3.09 s
against 7.6 s (41%). The geometry-derived timeouts were correctly sized and nothing
came close to timing out. Peak yaw rate 23.2 deg/s against the 60 abort.

| phase | n | mean | max | gate | margin used |
|---|---|---|---|---|---|
| settled | 1816 | 0.111 m | 0.227 m | 0.35 | 65% |
| transit | 520 | 0.303 m | **0.444 m** | 0.60 | **74%** |

Settled error is *better* than the waypoint flight (0.135/0.252); transit is worse
(was 0.229/0.405) because each leg now starts from a position the preceding turn
displaced. **74% of the transit gate is the tightest number in the flight — lengthen
the sides or raise `waypoint_speed` and that is what breaks first.** Slew limiter
exact: 5.000 mm max step, 0 violations in 2335 intervals.

VIO held through four 90 deg turns but with visibly less margin than translation-only
flight: features median 272 (was 330), min 154, 2 samples below 160, none below 80.
The old 160 floor would not have aborted this either (2 samples is far short of the
0.25 s persistence). Yaw error mean −2.92 deg, p2p 9.89 — expected for a
mostly-turning flight; the mean is ramp lag, not droop.

**Two problems from this flight:**

1. **Battery finished at 2% SoC** (10.71 V under 23.3 A), after the previous flight
   already ended at 11%. That is below any usable reserve — deep discharge damages the
   pack and risks a brownout instead of a landing. `battery_to_ros` was **not running**
   (the stack had been up since before that node existed), so nothing was on screen;
   it would have read EMPTY for the entire flight. Restart the stack before flying.
2. **Hard landing, worse than 07-26.** Roll rate hit **178 deg/s** at t=57.41. Every
   gyro extreme is after the square finished (during the square: roll 15.1, pitch 12.5,
   yaw 23.2 max) — this is purely ground contact. Altitude touched 0.000 m at t=56.99
   then rose back to 0.177 m before the operator killed it at 58.60, i.e. it **bounced**.
   Second flight pointing at `MPC_LAND_SPEED`; see Open Risks.

Design notes that held up: corners are planned from the latched start pose, never
chained off actual position (chaining folds tracking error into the shape), and a
square that would not fit the geofence is **refused, not clamped** — clamping a corner
deforms it. `yaw_rate_deg` defaults to 15 rather than 5 because a 90 deg turn at
5 deg/s is 18 s and four of them do not fit in a flight; `leg_timeout` and
`turn_timeout` are computed as travel-time + margin, which is the fix for the
documented `yaw_timeout` footgun. If a leg times out, raise the *margin*.

### NEW 2026-07-27: `battery_to_ros` — battery indicator for Foxglove

Flattens `/fmu/out/battery_status_v1` into `std_msgs` on `/battery/{percent,voltage,
cell_voltage,current,power,level,status}` so Foxglove Gauge and Indicator panels can
bind directly. Starts with `rtabmap_slam_px4.launch.py`; `battery_monitor:=false` to
disable. `level` (0 OK / 1 LOW / 2 CRITICAL / 3 EMPTY) is the **worse** of percent
thresholds, per-cell voltage, and PX4's own `warning` enum, so an optimistic SoC
cannot mask a real low-voltage warning. Verified live against the vehicle: reads
`OK 100% 12.12V (4.04V/cell)` on a fresh pack. See README for panel setup.

### NEW 2026-07-27: interactive Foxglove waypoints — BUILT, BENCH-VERIFIED, NOW FLOWN

`offboard_waypoint` (+ `offboard_waypoint.launch.py`) flies to points clicked in the
Foxglove 3D panel. Full operating instructions are in `README.md`; what matters here:

- ~~It has never been armed.~~ **Flown successfully 2026-07-27** — see the section
  above for the flight results.
- Foxglove has no `InteractiveMarker` support. The interaction is the 3D panel's
  Publish tool → `geometry_msgs/PointStamped` on `/waypoint/clicked`.
- **`rtabmap_slam_px4.launch.py` changed**: the bridge now runs with
  `capabilities:=[clientPublish,connectionGraph]` and
  `client_topic_whitelist:=['^/waypoint/clicked(_pose)?$']`. Before this the browser
  could not publish at all. Keep that whitelist narrow — widening it to `['.*']` puts
  `/fmu/in/*` (setpoints, arm commands) in reach of any WebSocket client.
- **`OffboardHover` gained two overridable properties**, `hold_point` and
  `horizontal_error_limit`, and `check_flight_position` moved up from
  `OffboardHoldYaw`. Behavior of the existing nodes is unchanged (`hold_point`
  defaults to the takeoff latch); the waypoint node re-points the watchdog at the
  commanded position, because otherwise the 0.35 m hold gate lands the vehicle for
  successfully translating.
- **Transit lag is the tuning risk.** PX4 trails a moving setpoint by about
  `waypoint_speed / MPC_XY_P` ≈ 0.26 m at the 0.25 m/s default, so the tight 0.35 m
  gate applies only after the commanded point has been still for 1.0 s; a looser
  0.60 m `transit_horizontal_error` covers transit. If waypoint flights abort on hold
  error, that pair is the first thing to look at — not the VIO.
- Click bounds: wrong `frame_id` rejected; target clamped into a 1.5 m disc around the
  takeoff latch; **click z ignored** (Foxglove clicks land on the z=0 ground plane, so
  honoring it would fly into the floor); setpoint slews at 0.25 m/s.
- **`min_vio_features` is 80 in `offboard_waypoint.launch.py`, not the 160 the hover
  and yaw tests use.** Lowered 2026-07-27 at the operator's request after a run aborted
  at 134 features. Note 134 is below anything in the flight record (prior minimums 184
  and 220, medians 284–336), so the room, not the threshold, is the underlying change —
  the lower floor buys tolerance, not tracking quality.

Bench verification 2026-07-27 against a faked PX4/VIO (`fake_px4.py` harness, not
committed): 999 setpoints, max commanded step exactly 0.250 m/s, wrong-frame click
rejected without moving the target, a 40 m runaway click clamped onto the fence at the
correct bearing, `/waypoint/target` and `/waypoint/status` publishing. `colcon build`
and `colcon test` green: `colcon test-result` reports **45 tests, 0 failures**
(41 pytest cases across the four suites, of which 18 are the new
`test/test_offboard_waypoint.py`).

Known pre-existing gap this surfaced, NOT fixed: `main()` in all three offboard nodes
catches only `KeyboardInterrupt`, so a **SIGTERM** exits via
`rclpy.executors.ExternalShutdownException` without running `on_shutdown()` — i.e.
without the while-armed AUTO.LAND. Ctrl-C (SIGINT) and the launch shutdown path are
unaffected. Worth fixing deliberately rather than as a side effect of this work.

### VFH2D obstacle avoidance — PARKED 2026-08-04, see `HANDOFF_VFH.md`

A VFH+ obstacle-avoidance planner: `vfh2d.py` (the algorithm — no ROS, no numpy,
no vehicle), `vfh_obstacles.py` (`/rtabmap/obstacle_cloud` → body-frame
`(range, bearing)`), `vfh_telemetry.py` (everything Foxglove draws),
`vfh_monitor` (runs the planner live, **publishes nothing to PX4**) and
`offboard_vfh` (subclasses `OffboardWaypoint`; a click becomes a *goal* and the
setpoint becomes a 0.60 m carrot along the direction VFH picks every 0.2 s).
The implementation, tests, and operating notes are preserved in
**`HANDOFF_VFH.md`**, but this is no longer the current path-planning direction.
It remained unarmed. VFH is reactive local steering rather than route planning;
it has no global path or robust dead-end behavior, and its decisions depend
strongly on point-cloud density, the height slab, and short-lived obstacle
memory. No normal stack launch starts it.

Three things that matter if you touch nothing else:

- **It is dead without `slam_publish_clouds:=true`.** The obstacle cloud is off
  by default and the flight node's response to no data is to hold, then land.
- **`rtabmap_slam_px4.launch.py` changed**: `'^/vfh/.*$'` added to
  `foxglove_topic_whitelist`, or none of the telemetry reaches the browser. It
  stays read-only — nothing subscribes to `/vfh/*` and the client publish
  whitelist is untouched.
- The tuned clearance envelope is `robot_radius + safety_margin = 0.4 m`
  (`0.30 + 0.10 m`). At 1 m it enlarges an obstacle by 23.6 deg on each side.
  The ±35 deg steerable cone is the camera's ~70 deg FOV; outside it the world
  is *unknown*, not free.

If revisited, begin with the observation-only monitor and reassess the planner
architecture before considering any offboard use. The likely replacement is a
persistent occupancy/voxel representation with route search and a separate
local collision checker/controller.

### Global A* path monitor — BUILT, OBSERVATION ONLY, see `HANDOFF_GLOBAL_PLANNER.md`

A cost-aware, repeatedly replanned 2D A* monitor now consumes
`/rtabmap/grid`, `/rtabmap/pose`, and Foxglove goals. The DepthAI bridge can
publish `RTABMapSLAM.occupancyGridMap` as a strict ROS `OccupancyGrid` with
`slam_publish_grid:=true`. The planner blocks unknown space, inflates obstacles
to the 0.40 m vehicle envelope, forbids diagonal corner cutting, and publishes
candidate/accepted `Path` messages plus an inflated map and metrics. It cannot
command PX4.

The simulator verified replanning when a passage closes (6.00 m route at
2.0 ms, then 7.46 m at 15.6 ms). Live observation then produced a 3.42 m route
in 2.5 ms. Grid/cloud alignment was quantitative: 96/96 obstacle points landed
on occupied cells versus 47.9% under the vertically mirrored alternative.
An observation-only position follower and correction-aware replan gate are now
also built. The final 137 s loop-correction bag produced zero backward
cumulative-progress events, bounded relative-position motion, 0.87 ms median
A*, and 12.1 Hz VIO without gaps. Nothing published PX4 trajectory setpoints.

Next session: implement a separately reviewed adapter that applies the
follower's relative ENU displacement from PX4's current NED local position,
with `auto_arm=false`, HOLD-on-planner-fault, and the existing offboard safety
gates. Never send the absolute SLAM-world carrot to PX4. The fresh local
collision layer remains deferred only for controlled static environments and
is required before dynamic-environment use. Full details and props-off/live
gates are in `HANDOFF_GLOBAL_PLANNER.md`.

### Loop-closure correction — PARKED, see `HANDOFF_LOOP_CLOSURE.md`

Node `map_correction` estimates and rate-limits the SLAM loop-closure transform so a closure
arrives as a drift instead of a jump. **Observation only — nothing it publishes reaches PX4**,
so it can be ignored entirely while working on flight. Enabled by default in
`rtabmap_slam_px4.launch.py` (`map_correction:=false` to disable).

Measured 2026-07-27: corrections in this room are 10–34 cm against a 0.8 cm noise floor. Full
findings, the open A/B design decision, and how to re-run the measurement are in
`HANDOFF_LOOP_CLOSURE.md`.

### Flight-log analysis tooling

- `scripts/analyze_flight.py <bag_dir_or_mcap>` — needs the ROS 2 environment,
  not `.venv-mavlink`. Yaw tracking, drift resolved into **body** axes
  (left/right is the recurring failure mode), roll/pitch by flight phase, VIO
  health. Sorts by timestamp, so it also reads unindexed bags recovered from a
  killed recorder. If the bag contains `/waypoint/commanded` it adds a **WAYPOINT
  TRACKING** section (accepted clicks, per-waypoint convergence, slew-limiter
  check, hold error split into settled-vs-0.35 and transit-vs-0.60) and stops
  comparing takeoff-relative excursion to the 0.35 m abort — that number is the
  travel envelope for a waypoint flight, not an error.
- `.venv-mavlink/bin/python scripts/analyze_ulog.py <ulg> [<ulg> ...]` — motor
  outputs and control-allocator torques, which the bag does not contain.
  Accepts several logs for side-by-side comparison. Rotor index → position and
  the meaning of each motor pairing are documented in its docstring.

## Historical: handoff status as of 2026-07-25 (superseded above)

**MILESTONE 2026-07-25: first fully successful autonomous yaw test.** Bag
`offboard_hold_yaw_20260725T042730Z`: armed takeoff to 0.30 m, 15 deg yaw-out at
an exact 5.0 deg/s, 3 s hold, return to initial yaw, 2 s hold, AUTO.LAND,
disarm on ground — the complete sequence with **zero safety triggers** and no
manual intervention. (`...042650Z` is a benign K-key false start before engage.)

Flight quality from the bag:

- Climb reached the pre-yaw gate 2.4 s after CLIMB_HOLD began (the
  `MPC_TKO_RAMP_T` 3.0→1.0 fix worked; previous flight never got there in 8 s).
- Yaw tracking is still underdamped: vehicle lagged the finished slew by ~2 s,
  overshot the out-target by ~8 deg (113.2 vs 105.3), undershot the return by
  ~12 deg (77.8 vs 90.3), and oscillated ~±10 deg during holds. Peak gyro
  32.8 deg/s (abort threshold 60). ULog retrieved as
  `flight_logs/px4_04_26_06.ulg` — see the analysis below: the cause is a
  mechanical yaw-torque bias, not tuning.
- Max horizontal excursion 0.259 m (abort 0.35); altitude held 0.26–0.34 m.
- **Reproduced 04:39 with no changes** (bag `offboard_hold_yaw_20260725T043911Z`):
  full sequence completed again; same yaw signature — heading initially moved
  the WRONG way (83.9 deg while the setpoint ramped up), peaked at only ~102 of
  the 105.3 target, oscillated ±10–12 deg through both holds, peak gyro
  31.7 deg/s, altitude/climb crisp. YAW_BACK "completed" in 0.02 s because the
  oscillation happened to be passing 90 deg — the yaw hold is effectively never
  converged. Operator K-killed during LAND on the ground — benign.
  ULog retrieved as `flight_logs/px4_04_37_47.ulg` and it **independently
  confirms the yaw-torque bias**: mean +0.113, minimum exactly 0.000 (never
  negative for the entire flight), roll/pitch means ~0; hover motor pairs
  CCW 0.560/0.501 vs CW 0.291/0.259 (~1.9x). Two flights, same plant defect —
  proceed with the props-off mechanical inspection.

**ULog analysis of the successful flight (`flight_logs/px4_04_26_06.ulg`) —
the yaw sluggishness is MECHANICAL, not tuning:**

- No saturation anywhere: yaw torque peaked at 0.30 of ±1.0; motors peaked 0.76.
- **The vehicle carries a large static yaw-torque bias.** Mean commanded yaw
  torque over the whole flight was **+0.145** (never below −0.016), while roll
  and pitch means were ~0 (CG fine). The allocator holds heading by running the
  CCW motor pair (indices 0,1) at **~2.1× the CW pair** (≈0.55/0.52 vs
  0.27/0.24 mean).
- This explains everything observed: sluggish/asymmetric yaw (overshoot with
  the bias, undershoot against it) and the ±10 deg hold limit-cycle (rate
  integrator, `MC_YAWRATE_I=0.04` vs default 0.1, slowly chasing a big bias).
  ~~and the ~20 A hover draw (pair imbalance wastes power)~~ — **the power part
  of this was wrong**, see the 2026-07-26 corrections above.
- ~~**Before any tuning: props-off mechanical inspection.**~~ **DONE 2026-07-26
  and this diagnosis was CORRECT.** Diagonally opposite props had been spinning
  in opposite directions; they now spin in the same direction, matching the PX4
  mixer. τ_z mean dropped +0.145 → +0.004. See the 2026-07-26 section above.
- Only if a mechanical cause is fixed and a bias remains: consider restoring
  `MC_YAWRATE_I` to 0.1 (via NSH) before touching P gains. (2026-07-26: the
  bias is gone; restoring `MC_YAWRATE_I` to 0.1 is now the recommended next
  tuning step, but only after the level calibration.)
- Yaw gains as flown: `MC_YAW_P=2.8`, `MC_YAWRATE_P=0.2`, `MC_YAWRATE_I=0.04`,
  `MC_YAWRATE_FF=0`, `MC_YAW_WEIGHT=0.4`.

### Fixes applied 2026-07-25 (later session, verified against the raw bag)

The runaway diagnosis was re-verified directly from
`offboard_hold_yaw_20260725T040154Z` (not just from this doc): yaw-out yawspeed
accelerated 3.9→48.2 deg/s with an implied setpoint slew of ~82 deg/s vs the
configured 8; return leg −36.6→−52.6 deg/s; gyro peak −101.3 deg/s; NAV_LAND at
20.19 s. Also confirmed: max altitude 0.218 m (never reached 0.30 m), and during
YAW_HOLD the published yawspeed was 0 with a constant setpoint while gyro still
peaked ±83 deg/s — so the hold oscillation is PX4-side (tuning/actuator), not the
ROS bug, and still needs the ULog with actuator data.

Changes (all in `px4_vio_bridge`, rebuilt, `colcon test` 14/14 green):

1. **Variable collision fixed** in `offboard_hover.py`: configured slew rate is
   now `self.commanded_yaw_rate` (immutable after init); gyro Z is stored only
   as `self.measured_yaw_rate`. `ramp_yaw()` uses only the commanded rate.
2. **Yawspeed feed-forward now opt-in**: new `yaw_feedforward` param (default
   `false`) publishes `yawspeed=NaN`; PX4 derives its own rate.
3. **Armed climb timeout now lands**: in `offboard_hold_yaw.py` CLIMB_HOLD,
   an armed run enters YAW_OUT only when altitude is inside `reach_tol` AND
   horizontal error ≤ new `pre_yaw_max_horizontal_error` (default 0.15 m);
   climb timeout triggers AUTO.LAND. Dry runs still proceed on timeout to
   exercise the state machine.
4. **Defaults lowered** (node + launch): `yaw_rate_deg` 8→5, `yaw_angle_deg`
   →15, `max_yaw_rate_deg` 90→60.
5. **Regression tests added** in `test/test_offboard_hover.py`: repeated
   gyro callbacks cannot change ramp step, published yawspeed, or the
   commanded rate; ramp converges 15 deg in ~150 ticks at 50 Hz.

Three armed runs of the revised yaw launch were recorded:

| Bag | Outcome |
|---|---|
| `flight_logs/offboard_hold_yaw_20260725T040002Z` | VIO pose became stale during climb; safety requested AUTO.LAND at bag time 5.81 s. |
| `flight_logs/offboard_hold_yaw_20260725T040049Z` | Completed the nominal yaw-out/hold/back sequence, but required keyboard KILL while landing; RTAB-Map reset sentinel appeared at 27.56 s. |
| `flight_logs/offboard_hold_yaw_20260725T040154Z` | Latest and most useful run. Excessive-yaw-rate safety requested AUTO.LAND at 20.97 s; no VIO reset occurred before the abort. |

### Critical software bug found in the latest bag

`OffboardHover` currently uses the same instance variable, `self.yaw_rate`, for two
different quantities:

1. configured yaw-setpoint slew rate, initialized from `yaw_rate_deg`; and
2. measured absolute gyro Z rate, overwritten by `on_sensor_combined()`.

`ramp_yaw()` then uses the overwritten gyro value as both its step size and
`TrajectorySetpoint.yawspeed`. This creates positive feedback during a turn.

Evidence from `offboard_hold_yaw_20260725T040154Z`:

- Configured yaw rate was 8 deg/s.
- Published `yawspeed` accelerated from 3.9 to 48.2 deg/s on yaw-out.
- Published return `yawspeed` accelerated from -36.6 to -52.6 deg/s.
- PX4 gyro reached -101.3 deg/s.
- The new safety detector worked and requested AUTO.LAND after the gyro stayed
  above 90 deg/s for 0.10 s.

Required fix before any further armed run:

- Rename the configured value to something immutable such as
  `self.commanded_yaw_rate`.
- Store gyro data separately as `self.measured_yaw_rate`.
- Make `ramp_yaw()` use only `self.commanded_yaw_rate`.
- Add a regression test proving repeated gyro callbacks cannot change yaw-ramp
  step size or published `yawspeed`.
- Prefer disabling yaw-rate feed-forward for the next test (`yawspeed=0` or NaN)
  while slewing the yaw angle at 5 deg/s.

### Armed test after the fixes (bag `offboard_hold_yaw_20260725T041514Z`, ULog `flight_logs/px4_04_13_50.ulg`)

First armed run with the fixed node: **all software behaved correctly** (yawspeed
NaN in all 699 setpoints, yaw setpoint never moved, climb timeout correctly
commanded AUTO.LAND instead of starting the yaw test) — but the vehicle only
reached 0.102 m in the 8 s window.

ULog analysis (retrieved over USB MAVLink FTP; PX4 clock ran ~84 s behind the
Pi, so `04_13_50.ulg` = the 041514Z bag):

- Motors were nowhere near saturation (peak 0.47, mean ~0.35). Not a power or
  battery problem (10.7 V under 20 A at 47% charge — still charge before
  flying).
- Root cause: PX4's takeoff ramp (`MPC_TKO_RAMP_T=3.0`) overrides vz_sp with a
  ramp starting at ~+2.1 m/s DOWN. Three seconds of that against a grounded
  vehicle wound the vz integrator to ~+5.6 m/s² (downward). Afterward the
  commanded climb rate was only ~0.24 m/s (0.25 m position error × MPC_Z_P=1.0),
  so the integrator unwound at ~0.45 m/s²/s and thrust crept 0.26→0.35 — still
  below the true hover point (~0.45–0.5, consistent with MPC_THR_HOVER=0.5)
  when the 8 s timeout landed it. It would have taken off given a few more
  seconds.
- Mitigations applied: `MPC_TKO_RAMP_T` 3.0→1.0 (set via NSH, saved);
  `climb_timeout` default 8→15 s (node + launch); launch `max_flight_time`
  20→40 s so the longer climb plus yaw sequence fits.
- `scripts/nsh.py` now retries through a pymavlink 2.4.49 `TypeError` crash in
  `wait_heartbeat` (intermittent, message-arrival race).
- Note: horizontal drift during that climb was 0.235 m — above the new 0.15 m
  pre-yaw gate. If drift persists after the climb fix, the yaw test will
  (correctly) refuse to start; consider raising hover height first.

### Latest-flight findings that remain after accounting for that bug

- The vehicle never reached the requested 0.30 m altitude. Maximum estimated
  height in the complete bag was only 0.218 m; it never entered the 0.23–0.37 m
  reach band.
- Despite that, `climb_timeout` advanced into `YAW_OUT`. For an armed yaw test,
  climb timeout must request AUTO.LAND instead of proceeding.
- The vehicle drifted about 0.223 m horizontally during climb before the yaw test.
- During the fixed 111 deg yaw-hold setpoint, physical yaw still oscillated from
  about 75.8 to 124.4 deg. Gyro rate exceeded 60 deg/s during this hold, even
  though the setpoint had stopped moving. This indicates a remaining yaw
  control/actuator problem beyond the ROS variable-collision bug.
- Maximum horizontal hold error was 0.369 m, occurring during landing.
- VIO remained continuous in the latest run: no RTAB-Map reset sentinel, no PX4
  dead reckoning, and bridge reset counter stayed zero.
- Feature count was normally healthy (median 336/400), but fell to 122 near the
  yaw-rate abort. Eight samples were below the new 160-feature threshold.
- VIO/PX4 yaw disagreement reached about 23 deg at the abort. The yaw-rate gate
  fired first.
- The latest MCAP has no actuator/mixer outputs, so motor saturation, weak yaw
  authority, or an incorrect motor/prop configuration cannot yet be separated.
  Retrieve the corresponding PX4 ULog or ensure actuator-control/allocation data
  is logged before the next tethered test.

### Required next-session sequence (2026-07-25 — superseded by the 2026-07-26 list above)

1. ~~Fix and test the `self.yaw_rate` variable collision.~~ DONE 2026-07-25.
2. ~~Change armed climb timeout to AUTO.LAND; never start yaw unless altitude is
   actually reached and horizontal error is inside a tighter pre-yaw gate.~~ DONE
   (`pre_yaw_max_horizontal_error`, default 0.15 m).
3. ~~Use a first yaw profile of at most 10–15 degrees at 5 deg/s, with no yawspeed
   feed-forward.~~ DONE — now the launch defaults.
4. ~~Lower the airborne yaw-rate abort from 90 deg/s to approximately 55–60 deg/s.~~
   DONE — 60 deg/s default.
5. ~~Perform a props-off dry run and inspect the recorded trajectory setpoints.
   They must show a constant 5 deg/s angular slew and zero/NaN yawspeed.~~ DONE
   — verified in the 07-26 flight: all 809 setpoints had `yawspeed=NaN` and the
   yaw setpoint slewed at exactly 5.00 deg/s.
6. Before arming, require non-sentinel VIO, bridge output, valid local position,
   features comfortably above 160, and stable heading.
7. If an armed test is attempted, tether it, keep the RC kill ready, and retrieve
   the PX4 ULog immediately afterward.

### Live PX4 parameters verified/saved on 2026-07-25

- `EKF2_EV_CTRL=11`
- `EKF2_EV_POS_X=+0.100 m`
- `EKF2_EV_POS_Y=-0.036 m`
- `EKF2_EV_POS_Z=+0.056 m`
- `EKF2_EV_DELAY=270 ms`
- `MC_YAWRATE_MAX=60 deg/s`
- `MPC_XY_VEL_MAX=1.0 m/s`
- `MPC_Z_VEL_MAX_UP=0.5 m/s`

The EV lever arm is already handled by PX4. Do not also apply the translation in
the ROS bridge. A complete pre-change parameter backup is
`params_backup_20260725_pre_yaw_safety.params`.

### Build/test state

- Package rebuild completed successfully.
- `colcon test` reported 9 tests, 0 failures.
- Those passing tests do **not** cover the variable-collision bug described above.
- The working tree already contained uncommitted reset-sentinel rejection,
  rosbag-recording, tests, and documentation changes. Preserve them; inspect
  `git diff` before editing.

## Earlier validated status

- **EV position + yaw fusion working and validated.** `xy_valid`, `z_valid`, `v_xy_valid`,
  `heading_good_for_control` all true; EV yaw tracked physical rotation with correct sign/magnitude.
- **EV/VIO pipeline delay measured and configured.** Two props-off yaw-reversal captures against
  PX4 gyro measured 245 ms and 270 ms; live `EKF2_EV_DELAY` is now saved at `270 ms`.
- **TELEM2/DDS link is marginal** (works USB-free but near the edge — see Known Issues).

## Bring-up

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash && source /home/john/ros2_ws/install/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py
```
Starts: RTAB-Map VIO/SLAM, `vio_to_px4_odometry`, `px4_local_position_to_ros`, and
Foxglove (:8765). The systemd-owned MicroXRCEAgent runs independently on
`/dev/ttyAMA0` @921600. Foxglove-only variants:
`rtabmap_oak_foxglove.launch.py`, `basalt_oak_foxglove.launch.py`.

Rebuild: `colcon build --packages-select px4_vio_bridge` (note: **ament_cmake** — see Gotchas).

### Micro XRCE-DDS Agent ownership (important)

There must be exactly one owner of `/dev/ttyAMA0`. The normal configuration is the
system v3.0.1 MicroXRCEAgent managed by systemd. Accordingly,
`rtabmap_slam_px4.launch.py` defaults to `start_xrce_agent:=false`; enabling its
launch-owned fallback while the service is active creates serial-port contention
and can prevent the PX4 DDS session from connecting.

The repository service file is `systemd/micro-xrce-agent.service`. Install it once:

```bash
cd /home/john/autonomous_drone_px4_vio
sudo install -m 0644 systemd/micro-xrce-agent.service \
  /etc/systemd/system/micro-xrce-agent.service
sudo systemctl daemon-reload
sudo systemctl enable --now micro-xrce-agent.service
systemctl status micro-xrce-agent.service
```

Logs: `journalctl -u micro-xrce-agent.service -f`.

Manual fallback only: first run `sudo systemctl stop micro-xrce-agent.service`, then
launch with `start_xrce_agent:=true`. Never use both owners together. The legacy
`basalt_vio_px4.launch.py` still starts an agent unconditionally, so its use also
requires stopping the systemd service until that launch is migrated.

## PX4 params (saved to flash — the ones that matter)

| Param | Value | Why |
|---|---|---|
| `EKF2_EV_CTRL` | `11` | HPOS(1)+VPOS(2)+YAW(8). Must be a true **integer** (set via NSH, never MAVLink float). |
| `EKF2_HGT_REF` | `3` | Height reference = Vision. |
| `EKF2_GPS_CTRL` | `0` | No GPS. |
| `EKF2_EV_DELAY` | `270.0` | Compensates the gyro-measured VIO pipeline delay. |
| `EKF2_EV_POS_X/Y/Z` | `+0.100/-0.036/+0.056 m` | Calibrated camera lever arm in body FRD; PX4 applies the camera-to-FC position correction. |
| `MC_YAWRATE_MAX` | `60 deg/s` | Reduced after the 2026-07-25 yaw test measured a `-178 deg/s` reversal. |
| `MPC_XY_VEL_MAX` | `1.0 m/s` | Indoor test envelope. |
| `MPC_Z_VEL_MAX_UP` | `0.5 m/s` | Gentler short-hover takeoff. |
| `MPC_TKO_RAMP_T` | `1.0 s` | Was 3.0. The long ramp forces a descend-rate setpoint against the ground, winding the vz integrator down; caused the 2026-07-25 barely-took-off flight. |
| `UXRCE_DDS_SYNCC` | `0` | Timesync off — required or the uXRCE session never connects on this serial link. |
| `UXRCE_DDS_SYNCT` | `0` | Same. `timesync converged: false` is normal/OK even when fully connected. |
| `COM_ARM_WO_GPS` | `1` | Allow arming without GPS. |
| `COM_RC_IN_MODE` | `0` | RC transmitter expected (kept — RC is the manual-override/kill switch). |
| `SENS_BOARD_X_OFF` | `-4.8028` | Re-calibrated in QGC 2026-07-26 (was `-7.2275`, which left +3.95 deg of roll bias and caused the left drift). Ground roll now +0.9 deg. |
| `SENS_BOARD_Y_OFF` | `+0.9588` | Same calibration (was `-0.3085`). Ground pitch now +0.1 deg. |
| `MC_YAWRATE_I` | `0.1` | Restored to the PX4 default 2026-07-26 (was 0.04, lowered only to cope with the since-fixed yaw-torque bias). Flight-verified: eliminated the hold droop, −0.47 → −0.00 deg. |

Motor/mixer geometry (verified 2026-07-26, matches the corrected physical build):
`CA_ROTOR0` FR `PX=+1 PY=+1 KM=+0.05`, `CA_ROTOR1` RL `PX=-1 PY=-1 KM=+0.05`,
`CA_ROTOR2` FL `PX=+1 PY=-1 KM=-0.05`, `CA_ROTOR3` RR `PX=-1 PY=+1 KM=-0.05`.
Diagonally opposite rotors share a spin direction — if anyone re-props this
airframe, that is the rule to preserve.

## NSH access (PX4 shell from the Pi)

`scripts/nsh.py` runs NSH over MAVLink `SERIAL_CONTROL` on USB `/dev/ttyACM0` @115200
(venv `.venv-mavlink`). USB is **dev/debug only**; there is no NSH shell without it.

```bash
.venv-mavlink/bin/python scripts/nsh.py "param show EKF2_EV_CTRL" "ekf2 status"
```

`listener <topic>` works too and is the quickest live check of estimator output,
e.g. `listener vehicle_attitude 2` to read roll/pitch/yaw straight off the FC.

### Retrieving ULogs (`scripts/fetch_ulog.py`)

The bags do **not** contain actuator/mixer data, so any question about motor
saturation, yaw authority or torque bias needs the PX4 ULog. Pull it over
MAVLink FTP on the same USB link:

```bash
.venv-mavlink/bin/python scripts/fetch_ulog.py --list      # logs on the SD card
.venv-mavlink/bin/python scripts/fetch_ulog.py --latest    # newest -> ros2_ws/flight_logs/px4_<name>.ulg
.venv-mavlink/bin/python scripts/fetch_ulog.py /fs/microsd/log/2026-07-26/03_30_57.ulg out.ulg
```

`--latest` verifies the received byte count against the size the SD card
reports and fails loudly on a mismatch. A 1.36 MB log takes a couple of minutes.
Analysis needs `pyulog` (already in `.venv-mavlink`); the topics that matter are
`vehicle_torque_setpoint` and `actuator_motors`.

**Matching a ULog to a bag:** the PX4 clock runs tens of seconds *behind* the Pi
(78 s on 07-26, 84 s on 07-25), so `03_30_57.ulg` pairs with the bag stamped
`033215Z`. Match on duration and content, never on the name alone.

**pymavlink 2.4.49 note.** `scripts/nsh.py` now patches `mavutil.add_message()`
at import (and `fetch_ulog.py` inherits the fix by importing it). Upstream
stores a message with `_instances` left at None when its instance field is
unset, then subscripts that None when a later message of the same type does
carry an instance value — `TypeError: 'NoneType' object does not support item
assignment`, from any `recv_match()`. This is the real cause of the intermittent
`wait_heartbeat` crash previously worked around with a retry loop; the retry is
kept as belt-and-braces but should no longer trigger.

## Autonomous hover (offboard)

```bash
# props-off DRY RUN (never arms — validates plumbing):
ros2 run px4_vio_bridge offboard_hover --ros-args -p auto_arm:=false -p hover_height:=0.30 -p hold_time:=10.0
# LIVE (props on, RC bound as kill, area clear):
ros2 run px4_vio_bridge offboard_hover --ros-args -p auto_arm:=true  -p hover_height:=0.30 -p hold_time:=10.0
```
Node confirms arm/offboard from `/fmu/out/vehicle_control_mode` (this build does NOT publish
`vehicle_status`). Safety: `auto_arm` default false; aborts if OFFBOARD+ARM not confirmed in time;
`max_flight_time` watchdog → LAND; lost position in flight → LAND; Ctrl-C while armed → AUTO.LAND.
While armed, stale VIO/feature/bridge-output data, fewer than 160 tracked features for 0.25 s,
more than 20 degrees of VIO/EKF yaw disagreement for 0.20 s, yaw rate above 60 deg/s for 0.10 s,
or horizontal hold error above 0.35 m for 0.25 s commands AUTO.LAND (with a 1.0 s post-arm grace
period).
When the node runs in an interactive terminal, press **L** (no Enter) for controlled AUTO.LAND.
Pressing **K** sends PX4 forced-disarm commands for one second: the motors stop immediately, even in
the air. These Pi-side controls depend on the terminal process and TELEM2/DDS link, so they supplement
rather than replace the RC kill switch.

## Pre-flight gate (all green before `auto_arm:=true`)

- uXRCE `Running, connected`; DDS streaming (`px4_local_position` publishing, not stuck re-creating).
- `vehicle_local_position`: `xy_valid`/`z_valid`/`v_xy_valid`/`heading_good_for_control` all true.
- `estimator_status_flags`: no `reject_*`, no `fs_*`; `cs_ev_pos`/`cs_ev_yaw` true.
- Props on & secure, RC on with hand on kill, area clear.
- VIO initialized: move the drone (translation + gentle yaw) before arming so VIO converges (see Gotchas).

## Open risks before live flight

1. **TELEM2 link reliability on battery power, USB-free** — verify DDS is rock-solid on the vehicle's
   own power before trusting it airborne (see Known Issues).
2. **0.30 m is very low** — vision-only height (no rangefinder); ground effect can disturb VIO features.
   The 2026-07-25 flights held 0.26–0.34 m successfully, but 0.5–0.6 m remains preferable for VIO margin.
3. ~~Live arm+climb never exercised yet.~~ Done 2026-07-25: full arm/climb/yaw/land sequence flown clean.
4. ~~**Roll level reference is ~4 deg out**~~ — SOLVED and flight-verified 2026-07-26 05:10.
   Hover roll +2.67 → −0.46 deg; steady offset 0.200 m left → 0.020 m right; max excursion
   0.231 → 0.099 m. Root cause of the first failed attempt was the USB cable tilting the FC
   during calibration — see Known Issues.
5. **Landing touchdown is firm and getting worse — now the top open item.** 07-26
   recorded roll-rate spikes of +131 and +112 deg/s at ground contact; the 07-27
   square flight hit **178 deg/s** (t=57.41) and the altitude trace shows it touching
   0.000 m then rebounding to 0.177 m before the operator killed it — it **bounced**.
   Confirmed to be ground contact only: during the square itself the peaks were roll
   15.1 / pitch 12.5 / yaw 23.2 deg/s. Three flights now point at `MPC_LAND_SPEED`.

6. **Battery discipline. Two consecutive flights landed at 11% and then 2% SoC.**
   2% (10.71 V under 23.3 A) leaves no reserve for a go-around and deep-discharges the
   pack. `battery_to_ros` now puts `/battery/{percent,level,status}` on Foxglove
   (Gauge + Indicator, see README) — but it only exists in stacks started after
   2026-07-27 12:26, so **restart `rtabmap_slam_px4.launch.py`** or it will not be
   there. Start scripted flights on a full pack: the square draws ~249 W for ~60 s.

## Known Issues & Gotchas

- **THE USB CABLE TILTS THE FLIGHT CONTROLLER ON THE ROLL AXIS. Never level-calibrate
  with it plugged in.** The cable is removed for flight, so any attitude measured while
  it is connected — QGC Level Horizon, or `listener vehicle_attitude` over NSH — includes
  a tilt that will not be there in the air. Running QGC Level Horizon over USB freezes that
  tilt into `SENS_BOARD_X_OFF`, which is exactly what wasted the first calibration attempt
  on 2026-07-26 (see the drift section above). Symptom: bench roll readings that wander with
  cable routing — +0.9, −1.3, −0.1 and +2.21 deg were all observed within one hour on an
  unchanged vehicle and calibration.

  Consequences for method:
  - **The only trustworthy roll reference is hover roll from a ULog.** At zero velocity the
    thrust axis must be vertical, so true roll is 0 and anything the estimator reports is
    pure offset error. The floor is not a reference either — it is not flat.
  - **Absolute ground readings over USB are meaningless. Deltas are still valid**, provided
    the cable is not disturbed between the two reads — the tilt is common-mode and cancels.
    That is how the 1.10 gain was measured.
  - **Writes are unaffected.** Attitude does not matter when setting a parameter, so the
    correction can be computed from flight data and applied over USB.
  - If Level Horizon must be used, route/support the cable so it does not load the FC first,
    then verify against hover roll on the next flight.

- **Measured EV/VIO delay is about 260-270 ms.** On 2026-07-16, two hand-yaw captures correlated
  `/fmu/in/vehicle_visual_odometry` yaw rate with PX4 `/fmu/out/sensor_combined` gyro Z. Peaks were
  245 ms (correlation 0.582, 135.6 deg excitation) and 270 ms (correlation 0.985, 54.7 deg), so use
  about 260 ms as the current estimate. The peak-widths overlapped from 195 to 320 ms. Do not infer
  delay by correlating EV with `vehicle_local_position`: EKF IMU propagation can make local position
  lead the delayed EV observation. Repeat with `scripts/measure_ev_fusion_delay.py`; live
  `EKF2_EV_DELAY` was verified saved at `270.0 ms` on 2026-07-25.

- **TELEM2/DDS link is marginal.** USB `/dev/ttyACM0` is dev/debug; flight comms is TELEM2/uXRCE
  `/dev/ttyAMA0` @921600. A cold uXRCE session sometimes gets stuck re-establishing its whole
  datawriter graph (0 telemetry to ROS: `px4_local_position` publishes 0, probes get 0). Observed
  2026-07-09: it established after a USB reconnect and then kept streaming at 50 Hz **even with USB
  back out**, and came up clean USB-free across several Pixhawk power-cycles. Fragile phase is
  **establishment**, not steady state → marginal serial link (likely an unreliable TELEM2 cable;
  grounding margin helps at the edge). Fixes to try before flight: short shielded/twisted cable
  (TX/RX twisted with a solid common GND), `SER_TEL2_BAUD=460800` for margin. **Without USB there is
  no NSH**, so USB-free DDS recovery = physical Pixhawk power-cycle. Health check: `create_datawriter`
  count in the launch log should stop increasing once established; `Published PX4 local position`
  should keep increasing (each log line = 100 msgs).

- **EKF2 int/bitmask params corrupt if set over MAVLink/QGC as float.** PX4 stores the float's raw
  bit pattern as the int (e.g. `EKF2_EV_CTRL` became `1077936128` = float `3.0` bits → no EV bits set →
  no fusion). Set/verify via NSH `param show`/`param set` only. (Also captured in memory
  `px4-int-param-float-corruption`.)

- **uXRCE bring-up:** use the system `/usr/local/bin/MicroXRCEAgent` (v3.0.1), NOT the project-local
  v2.4.3 (never handshakes). Needs `UXRCE_DDS_SYNCC/SYNCT=0`. If `Running, disconnected` after an agent
  restart with USB present: NSH `uxrce_dds_client stop; uxrce_dds_client start -t serial -d /dev/ttyS2 -b 921600`.
  (Also in memory `px4-uxrce-vio-bringup`.)

- **This PX4 build publishes `vehicle_control_mode`, not `vehicle_status`.** Use
  `flag_armed`/`flag_control_offboard_enabled` for arm/offboard state.

- **`px4_vio_bridge` is ament_cmake, not ament_python.** `setup.py` entry_points are vestigial/ignored.
  To add a node executable: (1) create `scripts/<name>` wrapper (`from px4_vio_bridge.<mod> import main`,
  chmod +x), (2) add it to `install(PROGRAMS ...)` in `CMakeLists.txt`, (3) rebuild. Verify with
  `ros2 pkg executables px4_vio_bridge`.

- **VIO "shake to align" at boot is normal.** VIO yaw origin is arbitrary and VIO needs motion to
  converge; EKF2 then does `reset_yaw_to_vision`. Roll/pitch are correct immediately (gravity). Make a
  deliberate pre-arm move-to-initialize a checklist step. A *repeatable residual* offset after
  convergence = static camera mount → bake into bridge `vio_yaw_offset_deg`.

- **DepthAI 3.7.1 is the isolated OAK startup failure.** There are 244 saved OAK
  crash dumps across multiple days; at least 185 contain the same device-side
  `RTEMS_FATAL_SOURCE_INVALID_HEAP_FREE` / MIPI `Invalid config steps` assertion. On 2026-07-13 a
  camera-only test reproduced it with no StereoDepth, VIO, SLAM, ROS, image/depth publication, or
  point clouds. Under 3.7.1, CAM_B failed at both 640x400 and native 640x480, and at 30, 15, and
  10 FPS, without producing a frame; CAM_C worked. The identical CAM_B test and a two-mono-camera
  test both worked immediately under DepthAI 3.5.0. The full RTAB-Map VIO/SLAM executable then ran
  at ~13.4 Hz on `/rtabmap/vio_pose`. Keep `depthai==3.5.0` from `requirements-depthai.txt`; the
  executable fails fast if 3.7.1 is installed. The host-side `X_LINK_ERROR` messages are consequences
  of the device firmware crash, not an XRCE failure or evidence that the OAK hardware needs an RMA.

- **Never publish a `TransformStamped` topic that Foxglove can see unless it belongs in the TF
  tree.** Foxglove grafts *any* `TransformStamped` topic into its transform tree, not just `/tf`.
  `map_correction` originally published the correction as `TransformStamped` with
  `frame_id=map` / `child_frame_id=odom`; this pipeline's tree is only `world`→`camera`, so those
  became a **disconnected second root**. The 3D panel resolved against `odom`, reported
  `Missing transform from frame <camera> to frame <odom>`, and **the point clouds and all frames
  vanished** — with nothing actually wrong in the SLAM graph. Fixed by publishing the correction as
  `PoseStamped` in `world`. If a panel is already stuck this way, set its follow/display frame back
  to `world`; stale frames clear when the publisher restarts.

- **Camera feed in Foxglove: use the compressed topic.** The feed now defaults to JPEG
  `CompressedImage` on `/rtabmap/image/compressed` (best-effort, keep-last-1) — raw `/rtabmap/image`
  (256 KB/frame) backed up over the WebSocket and the delay grew unbounded. In Foxglove point an
  Image panel at `/rtabmap/image/compressed`. Tune via launch args `rtabmap_image_format`
  (`jpeg`/`raw`), `rtabmap_image_jpeg_quality`, `rtabmap_image_publish_stride`. Image publish uses a
  non-blocking `tryGet`, so it can't stall the `/basalt/pose` → PX4 path. Enabled in the Foxglove-only
  launches; opt-in in the main launch (`rtabmap_publish_image:=true`).

- **Keep the RTAB-Map VIO feature target at the tested value.** On 2026-07-14,
  `slam_num_features:=700` left VIO stuck at the identity pose while `400`
  initialized immediately under the same motion. The feature target therefore
  defaults to `400` on this OAK-D Lite; the cause of the higher-count failure
  has not been isolated.

- **Keep `RTABMapVIO.transform` single-consumer in the combined SLAM graph.** Fan-out directly to
  both `RTABMapSLAM.odom` and the ROS bridge left VIO publishing identity poses. Publish raw VIO to
  ROS from `RTABMapSLAM.passthroughOdom` instead. With that wiring, the live obstacle map grew from
  3,090 to 3,900 points with nine distinct updates over 20 seconds.

- **ROS 2 CLI is flaky here** (`ros2 topic list/echo` miss BEST_EFFORT topics / hang). Prefer a small
  rclpy probe with matching QoS, or the launch-log grep checks above.

## ROS topics (reference)

`/basalt/pose` (VIO in), `/fmu/in/vehicle_visual_odometry` (bridge out),
`/fmu/out/vehicle_local_position_v1`, `/fmu/out/vehicle_control_mode`,
`/fmu/out/estimator_status_flags`, `/px4/local_position/{pose,odometry,path}`,
`/vio/yaw_offset/{pose,odometry,path}` (yaw-offset tester), `/rtabmap/{path,odometry}`,
`/vio/map_correction{,_target}` and `/vio/map_correction/{preview_pose,residual_m,residual_deg}`
(loop-closure correction, observation only),
`/waypoint/{clicked,clicked_pose}` (Foxglove click in, ENU `world`),
`/waypoint/{target,commanded,status}` (waypoint node out).
`/planner/{path,candidate_path,inflated_map,status}` (global planner monitor),
`/planner/follower/{carrot,lookahead,displacement,status,progress,path_progress,remaining,cross_track,path_generation}`
(position-only route follower monitor; observation only).

## Safety

Props removed for all bench/estimator work. No powered/offboard flight until: pre-flight gate green,
props secure, RC bound as manual override / kill switch, area clear, and the props-off dry run
(`auto_arm:=false`) has passed. Do not fly on the TELEM2 link until it's verified stable on battery power.
