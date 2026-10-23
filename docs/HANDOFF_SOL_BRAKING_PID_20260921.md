# Sol session handoff — roll-rate braking trial, 2026-09-21

## Newest gain-0.75 repeat and adapter decision

Read [the full repeat result](LATERAL_DAMPING_GAIN075_REPEAT_RESULTS_20260921.md).
With `MPC_XY_P=0.95`, velocity I=0.40 and roll-rate P=0.150 restored, the 10:12
flight passed all four holds using damping gain 0.75. Worst endpoint overshoot
was 4.7 cm and worst hold swing 10.0 cm. Exact ULog pairing shows 1.109 s of
continuous joint-gate dwell immediately before NAV_LAND; the missing ROS tail
is not a timeout. This is the second 4/4 gain-0.75 result. The opt-in,
default-OFF global/tunnel adapter port is now implemented; read
[the implementation note](GLOBAL_DAMPING_OPT_IN_20260921.md). It preserves the
0.10 m/s² correction cap, 0.10 s filter and 0.30 m/s² total cap and excludes
climb, LAND, fault/command holds, degraded ride-out, escape and NaN
pure-position safety modes. Do not raise gain/cap or enable it by default. Run
and review the full-graph props-off check before the first armed tunnel trial.

## Newer velocity-integral trial

After restoring position P to 0.95, the user flew `MPC_XY_VEL_I_ACC=0.30`
with roll-rate P 0.150 and damping OFF. Read
[the full result](MPC_XY_VEL_I030_RESULTS_20260921.md). It passed three holds
and reached the final return, but the last hold swung 29.2 cm and timed out;
yaw tracking was worse and inferred velocity compensation did not decrease.
Restore `MPC_XY_VEL_I_ACC=0.40`; do not reduce it further or combine 0.30 with
damping. Live post-flight readback was Disarmed with 0.30 still saved. No
parameter was written during analysis.

## Newer position-gain trial

After restoring roll-rate P to 0.150, the user flew `MPC_XY_P=0.75` with
damping OFF. Read [the full result](MPC_XY_P075_RESULTS_20260921.md). It reduced
early position-loop velocity demand and early lateral hold motion, but again
timed out on leg 3; its late third-hold swing was 27.1 cm versus 14.2 cm for the
0.95 baseline, and yaw tracking was worse. Restore `MPC_XY_P=0.95`. The live
post-flight readback was Disarmed with 0.75 still saved. No parameter was
written during analysis.

## Completed-flight update

The selected experiment has now flown: **MC_ROLLRATE_P 0.165 then 0.150**, both
with adapter damping OFF. Read [the full results](ROLL_RATE_TRIAL_RESULTS_20260921.md).
Both flights passed the first two holds and timed out on the third. P=0.165
initially arrested motion more strongly in parts of the sequence but then had
far worse renewed hold motion; equal-window roll-rate tracking did not improve.
Retain 0.150 and do not escalate roll-rate P. A post-flight live readback
confirmed Disarmed, no failsafe and saved `MC_ROLLRATE_P=0.1500`. The ULogs are
`reports/roll_rate_0854_0914_20260921/23_32_28.ulg` (0.165) and
`23_52_43.ulg` (0.150). No parameter was written during analysis.

Read this first, then [the migration handoff](HANDOFF_SOL_MIGRATION_TUNNEL.md)
for the larger tunnel context. This note supersedes the previous recommendation
to repeat only the adapter-damping comparison.

## Original request and pre-flight state (historical)

The user asked what to turn up/down for better braking, whether to tune PID,
then requested a test command and a handoff for a new Sol session.
The selected experiment was **MC_ROLLRATE_P 0.150 → 0.165 (+10%) with
adapter brake_damping OFF**. The commands below are retained as history; the
trial is complete and the current recommendation is in the update above.

**At the time of the original handoff, commands had been provided but not
executed.** The completed-flight update above now supersedes this pre-flight
state.

Repo: /home/john/autonomous_drone_px4_vio
Branch: tune/migration-tunnel-smoothing
HEAD: 893cbea (proven ros-rtabmap-migration).
Work is uncommitted; preserve all modified/untracked files and local reports.
No commit/push requested. Abandoned experimental work remains at fc99cc4 on
fix/velocity-aware-stop-recovery; do not apply it or the backup stash.

At this historical point, the opt-in damping experiment existed only in the
Python lateral-stop adapter, with defaults OFF / gain 0.5 / cap 0.10 / filter
0.10 s, and the instruction was not to port it into the tunnel during that
trial. The newer adapter decision at the top supersedes that restriction; the
global C++ controller and 0.20 m/s / 0.30 m/s² route limits remain unchanged.

## Original user-run test commands (completed; do not repeat automatically)

With USB connected, first check status and the baseline:

```bash
cd /home/john/autonomous_drone_px4_vio
.venv-mavlink/bin/python3 scripts/nsh.py 'commander status' 'param show MC_ROLLRATE_P'
```

Only with output explicitly confirming Disarmed, set the trial and read back:

```bash
.venv-mavlink/bin/python3 scripts/nsh.py 'param set MC_ROLLRATE_P 0.165' 'param show MC_ROLLRATE_P'
```

Require readback 0.1650 before treating the flight as the trial. This changes
the FC parameter persistently; launching ROS alone does not set it.
Disconnect USB before flight. Use the established open-area lateral-test
setup, usual VIO/PX4 bridge running, only one flight adapter, comparable charged
battery, and at least 1 m clear to either side. This test has no obstacle
avoidance. Then launch (automatically arms, flies and records):

```bash
cd /home/john/autonomous_drone_px4_vio
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
source ros2_ws/install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge offboard_lateral_stop.launch.py \
  auto_arm:=true brake_damping:=false
```

Defaults retain 35 cm right / return / left / return, 0.20 m/s, acceleration
0.30 m/s², height 0.30 m, unchanged hold gates and watchdogs.
L requests landing; K force-disarms. Land if new twitching/buzzing or worsened
oscillation appears. Let the recorder finish closing after the flight.

Rollback / baseline comparison, with USB reconnected and disarmed status
confirmed again:

```bash
cd /home/john/autonomous_drone_px4_vio
.venv-mavlink/bin/python3 scripts/nsh.py 'commander status'
```

Then:

```bash
.venv-mavlink/bin/python3 scripts/nsh.py 'param set MC_ROLLRATE_P 0.150' 'param show MC_ROLLRATE_P'
```

Verify 0.1500. Use the same ROS command above for an OFF-damping baseline.
Do not silently leave an unhelpful trial gain installed. No rebuild is required
for these launch/FC parameter changes.

## Why this trial, and what is already known

PX4's demanded braking can precede achieved braking tilt. In the third latest
flight at bag time 18.0 s, lateral velocity was +0.158 m/s, internal acceleration
demand −0.390 m/s², but tilt-equivalent acceleration +0.039 m/s².
Tilt-equivalent acceleration is a near-hover approximation, not measured net
acceleration. It does not isolate a motor fault, PID gain or estimator delay.

Roll/pitch rate P/I/D/K recorded in the latest ULogs:
0.150 / 0.100 / 0.003 / 1.0. Attitude P=4.0.
Horizontal position P=0.95; velocity P/I/D=1.8 / 0.4 / 0.2.
EKF2_EV_DELAY=225 ms. No change to these other parameters is selected.
Damping gain 1.0 and position P 0.75 were discussed as separate possible
experiments, **not implemented or selected for the next run**.

This exact roll-P value was tested on September 8 and rolled back. See
[prior roll-P results](ROLL_RATE_TRIAL_RESULTS_20260908.md): moving roll-rate
RMS improved about 20%, but final hold still swung 26.7 cm, with yaw issues and
calibration differences confounding comparison. Therefore 0.165 is a known
test point, not a proven fix. Do not automatically escalate to another gain.
A lateral position test screens end-to-end behavior; it is not isolated
rate-loop identification.

## Latest recordings and evidence

Read [0.75 results](LATERAL_DAMPING_GAIN075_RESULTS_20260921.md).

| ROS bag suffix | Damping | Outcome |
| --- | --- | --- |
| 20260921T082729Z | ON 0.75 | All four holds passed; max directional overshoot 10.4 cm |
| 20260921T082834Z | OFF | First hold timed out; eventual overshoot 17.4 cm |
| 20260921T082906Z | ON 0.75 | All four movements; final hold apparently timed out; max overshoot 15.9 cm |

All names begin offboard_lateral_stop_ under ros2_ws/flight_logs/.
First-three-second lateral velocity RMS: 0.023 / 0.036 / 0.088 m/s.
Loaded voltages: 11.16 / 10.92 / 10.77 V. Battery/order/setup differences
prevent a clean causal gain comparison. OFF initially stopped within about
2 mm beyond target, then moved away again: renewed hold motion is a key fault.

reports/lateral_damping_0827_20260921/ contains decoded JSON, comparison.json,
alignment.json, analysis scripts, three plots and matching onboard files:
23_06_28.ulg / 23_07_33.ulg / 23_08_05.ulg.
These were downloaded read-only from SD /fs/microsd/log/2026-09-18/.
Exact timestamp matching, not filename clocks, establishes pairing.

Third MCAP is truncated around 33.5 s; matching ULog continues through 56.9 s.
Final stationary hold starts 43.354 s; landing requested 53.353 s.
Timeout attribution is inferred; ROS failure message is missing.
Do not classify recorder truncation as flight-controller failure.
OFF ULog has a 131 ms dropout in its first hold; continuous ROS data cross-check
the first-stop metrics. Other details and sampling uncertainties are in the report.

Earlier 0.5 ON/OFF/ON batch at 081124 / 081216 / 081254:
[LATERAL_DAMPING_RESULTS_20260921.md](LATERAL_DAMPING_RESULTS_20260921.md).
Neither enabled flight completed; OFF failed first hold.

## What Sol should do after the new flight

1. Discover the newest bags. Verify recorded brake_damping=false and matching
   onboard MC_ROLLRATE_P (0.165 trial versus 0.150 baseline); don't infer from
   user commands alone. Check for other parameter changes.
2. Extract with scripts/analyze_lateral_stop.py in the sourced ROS environment.
   Filter debug to LATERAL_MOVE/LATERAL_HOLD before summarize: LAND retains
   phase HOLD/last-leg fields and would contaminate stop statistics.
3. Pair the onboard ULog using exact vehicle_local_position timestamps and
   coordinate agreement. scripts/fetch_ulog.py supports read-only USB download.
   Reuse the analysis approach, not hardcoded names, time windows, expected
   outcomes or third-flight truncation recovery in the existing report script.
4. Compare corresponding braking/hold phases: initial and eventual overshoot,
   hold swing, residual speed, settling, roll angle/rate tracking, yaw behavior,
   motor activity, voltage and estimator resets. Compare equal-duration windows
   as well as whole-hold outcomes; different hold durations confound amplitudes.
5. Retain the gain only if tracking and braking improve without worse
   oscillation. A lower rate-error RMS alone is insufficient.
6. Record the actual final FC parameter state and update this handoff/NEXT_SESSION.
   Keep the global adapter unchanged while this experiment is unresolved.

## Tools and validation

System python3 with the sourced ROS overlays reads bags.
.venv-mavlink/bin/python3 supplies pyulog, numpy, matplotlib and pymavlink.
Relevant scripts: scripts/analyze_control_response.py,
scripts/analyze_attitude_response.py, scripts/analyze_rate_loop.py.
The last script reconstructs roll/pitch P/D and inferred I using exact sample
timestamps; latest first-hold output is first_hold_rate_terms.csv/json inside
the 0827 report folder. It does not identify an optimal gain.

Prior runtime build and 83 focused tests passed when damping was implemented:
49 lateral, 20 hover, 14 launch. These are historical, not new test results.
This handoff preparation only checks documentation, commands and local paths;
no hardware commands or runtime tests were executed. Lead-only task.

PX4 reference:
[rate PID tuning](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter#rate-pid-tuning).
