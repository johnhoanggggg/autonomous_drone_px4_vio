# GPT-5.6 Sol handoff — minimal migration tunnel smoothing

Updated 2026-09-21. Read this before older tunnel/trajectory handoffs.

**New immediate task:** [Sol braking/PID trial handoff](HANDOFF_SOL_BRAKING_PID_20260921.md).
The user requested a command for the proposed roll-P 0.165 / adapter-damping-OFF
trial. That handoff contains setup, readback, rollback and next-flight analysis
instructions. No parameter write or flight has been performed by the assistant.
It supersedes the damping-only next-test recommendation below.

Latest follow-up: [lateral-stop braking/damping analysis](LATERAL_BRAKING_REVIEW_20260921.md).
Both September 17 tests show renewed motion during fixed-target holds, plus
attitude/rate-response lag. It proposes an explicitly enabled, bounded
velocity-damping trial in the lateral-stop adapter before changing the global
adapter. The user then approved implementation: the lateral-stop adapter now
has an opt-in damping trial, built and tested offline. Read
[the trial instructions](LATERAL_DAMPING_TRIAL_20260921.md) before the next run.
The global/tunnel adapter remains the proven baseline. Damping has now flown
ON/OFF/ON: [comparison and next trial](LATERAL_DAMPING_RESULTS_20260921.md).
The initial 0.5 trial improved first-stop settling but completed no full run.
The newer [0.75 comparison](LATERAL_DAMPING_GAIN075_RESULTS_20260921.md)
supersedes that next-trial proposal. At 08:27/08:28/08:29 settings were
0.75 ON / OFF / 0.75 ON: first completed all four holds, OFF failed the first,
third reached all four legs but apparently timed out at the final hold.
Matching ULogs recover its truncated ROS tail. Third ON run was much wobblier
(first-stop lateral velocity RMS 0.088 versus first ON 0.023 m/s); battery
conditions differ and demand/achieved-tilt lag remains. Keep 0.75 as a test
setting; repeat matched-condition comparisons before raising gain/cap or
porting to the tunnel. Defaults remain OFF/gain 0.5; no runtime changes in
either results-analysis session.

## User intent and repository state

The user flew the restored old branch and confirmed that it cleared the tunnel.
They want less wobble and more wall clearance, with minimal deviation from that
successful controller. They requested a new branch and a handoff for Sol.
Their subsequent clarification is: **"im trying to solve the overshoot"**.
Do not treat a lower cruise cap as the requested fix. The proposed 0.18 m/s
trial has been withdrawn; both launch files are restored to the baseline.

- Repository: `/home/john/autonomous_drone_px4_vio`.
- Working branch: `tune/migration-tunnel-smoothing`.
- Base/HEAD at handoff: `893cbea`, from `ros-rtabmap-migration`.
- Changes described below are uncommitted. No push was requested or performed.
- Experimental work was previously saved in commit `fc99cc4` on
  `fix/velocity-aware-stop-recovery`; do not cherry-pick its controller redesign.
- A named backup stash also remains. Do not pop it onto this branch.
- No live ROS nodes, hardware services or flight were started in this session.
  After the clarification, the onboard ULog was retrieved over USB using
  read-only NSH directory listing and MAVFTP. No PX4 parameters were changed.

Treat successful tunnel completion as the primary regression criterion. Avoid
another corridor-tracker rewrite, new braking/recovery state machine, or broad
retuning. Keep raw VIO, mapping correction, and PX4 local frames distinct.

## Flight evidence

Bag: `ros2_ws/flight_logs/offboard_global_20260921T072654Z`.
It was flown on the restored migration baseline, **before** this tuning.
Times below are seconds from the first selected bag message, not boot time.

| Event / measurement | Evidence |
| --- | --- |
| ROUTE entry | 7.847 s |
| First goal reached | 26.239 s; 18.39 s after ROUTE entry |
| Remaining route at first arrival | 0.110 m; follower arrival tolerance 0.12 m |
| Peak measured horizontal speed before first arrival | 0.295 m/s at 25.811 s |
| Recorded command cruise cap | 0.20 m/s in both adapter and follower |
| Route position-target lead | p95 0.195 m, maximum 0.228 m |
| Lead during degraded following | maximum 0.239 m |
| Minimum mapped centre-to-obstacle clearance | 0.214 m at 15.856 s |
| Configured radius / planning requirement | 0.25 m / 0.28 m |
| Longest ROUTE setpoint gap | 0.0601 s; nominal publication 20 Hz |

The map and PX4 local-position plots show lateral overshoot around a smoother
position-target track. Translation speed also repeatedly rises above the
published velocity feedforward. This supports tracking overshoot as a contributor
to wobble; it does not isolate PX4 gains, feedforward, replanning, or frame
rebasing as the sole cause. A position target plus velocity feedforward is not
a hard cap on measured vehicle speed. Target lead includes deliberate lookahead,
so its entire magnitude must not be treated as accidental tracking error.

At 15.139 s, the follower reports cross-track 0.10/0.08 m and invalidates while
the body is near the 0.28 m clearance requirement. The adapter uses its existing
0.06 m/s degraded crawl for 1.5 s, then enters HOLD at 16.648 s. Minimum clearance
occurs during that degraded interval. HOLD lasts 1.20 s, with 8.8 cm net PX4 XY
movement, and clearance improves to about 0.349 m. ROUTE resumes at 17.849 s.
This recovery eventually succeeds; do not casually remove it to eliminate pauses.

The interior traversal from 17.849–22.999 s has minimum mapped clearance about
0.349 m. A CORNER_HOLD at 22.999–24.197 s coincides with another 8.9 cm net
movement. Final approach minimum clearance is about 0.304 m, and speed reaches
0.295 m/s. There is room to improve tracking without changing the navigation
architecture.

The 0.214 m clearance is centre-to-obstacle distance, **not** spare airframe
clearance. It is 3.6 cm inside the configured circular footprint. This suggests
possible contact, but does not prove physical contact: grid resolution is 3 cm,
localization/maps change, and no independent contact sensor was analysed.

## Separate arrival issue, still unresolved

The tunnel is traversed, but the automatic goal-hold/landing sequence does not
finish normally. There was only one clicked goal.

- Goal true at 26.239 s, false at 26.539 (`CLEARANCE_BLOCKED`), true at 26.638,
  false at 27.037 (`CROSS_TRACK_EXCEEDED`).
- Adapter logs `GOAL_REACHED holding 0.0/3.0s` twice. Its timer resets when goal
  validity/fault conditions fail, and normal pursuit resumes.
- It subsequently attempts a large heading change near the goal.
- At 33.448 s the exact-origin VIO reset sentinel triggers AUTO.LAND.
  Kill-key handling appears at 35.294 s, DONE at 36.296 s.

Inspect `offboard_global_planner_node.cpp` around the `goal_reached_` branch and
the follower's arrival/clearance handling before changing terminal behavior.
A future small fix should distinguish terminal arrival from temporary follower
invalidity, stop chasing beyond the same goal, preserve localization/landing
fault handling, and reset correctly for a genuinely new goal. Do not simply
ignore faults or permanently latch an old goal. Add focused regression tests if
implementing that behavior. This session did **not** implement a terminal latch.

## Current changes: lateral-stop experiment, global controller unchanged

- Both launch speed defaults remain **0.20 m/s**; the 0.18 trial was withdrawn.
- Both acceleration limits remain **0.30 m/s²**. No C++ algorithm changes.
- Python lateral-stop adapter now has optional velocity-error acceleration
  damping; disabled by default. See the trial document for its 83 passing tests,
  settings, debug telemetry and the user-run comparison-flight command.
- Radius, safety margin, lookahead, yaw, corner blending, fault recovery,
  feedforward and goal-hold behavior retain the migration settings.
- README examples use the matched speeds and correct an outdated margin
  override from 0.00 to the existing/recorded 0.03. The margin default itself
  was not changed.

Reducing speed might lessen the excursion but does not identify or correct the
braking/response problem. No flight on 0.18 was recorded in this session.
Keep baseline `max_carrot_speed:=0.20` and `command_speed:=0.20` while diagnosing
overshoot; explicit launch arguments override defaults.

## Next steps, in order

1. Continue from the matching onboard analysis below. It confirms that PX4
   already demands braking during the entrance overshoot. Investigate the
   lag between demand, achieved attitude/rate and estimated translation;
   distinguish controller response from estimator timing before choosing gains.
2. Keep the proven route adapter. An earlier reference braking adjustment could
   reduce approach demand, but does not by itself fix the demonstrated response
   mismatch. Avoid another navigation recovery state machine or blind gain
   increases. If proposing controller tuning, isolate one adjustment and use
   the existing open-area lateral-stop procedure to assess stopping and hold
   oscillation before interpreting another tunnel run.
3. Validate any resulting small change against tunnel completion, entrance
   clearance and lateral excursion, without stacking unrelated changes.
4. Address the arrival issue as a separate scoped change with tests.
5. Measure target discontinuities, cross-track velocity,
   replan times and correction updates together before choosing another tune.
   Correction translation jumps reached 5.4 cm at 9.089 s and 4.3 cm at 18.990 s;
   these can move the rebased reference, but are not proof of the sole cause.
   If the planned route itself hugs a wall, consider a small *soft* clearance
   cost adjustment separately. Raising the hard margin or corner-cutting
   allowance can regress tunnel feasibility/clearance.

The current bag gives a concrete overshoot example: at 15.0 s, measured NED x
is -1.227 m, position target -1.092 m (13.5 cm in the return direction), and
measured vx is still -0.087 m/s, moving away. At 16.0 s measured x is -1.329 m,
target -1.109 m and vx -0.043 m/s. Position feedback should be distinguished
from the small still-negative velocity feedforward here. The reference limiter
does not take measured velocity as an input to its braking schedule.

Read the newer findings near the top of `HANDOFF_TRAJECTORY_SMOOTHNESS.md` and
`WALL_STRIKES_20260907.md`. Earlier matched ULogs showed demanded braking with
opposite achieved tilt, and fixed-position/lateral-stop tests reproduced
oscillation independently of the planner. These are historical evidence, not
proof of the current flight's internal control response. Historical gain/delay
readbacks must not be treated as the current vehicle settings without checking.

## Matching onboard log: confirmed braking-response mismatch

Retrieved `/fs/microsd/log/2026-09-18/22_07_06.ulg` to
`reports/migration_tunnel_20260921_0726/onboard/px4_22_07_06.ulg`.
SD/file size matches: 4,326,834 bytes. The SD date is not the host date.
1545/1713 bag local-position timestamps match exactly, with zero x-position
difference. Nonmatching samples include the bag time outside onboard logging.
Median boot-to-bag time offset: -129.975776675 seconds. Control demand/response
are paired causally on the onboard clock; that offset only labels the plot.
One 37 ms dropout occurs at bag-aligned 5.323 s, before ROUTE entry.

| Approximate bag time | Estimated NED vx | PX4 internal vx demand | PX4 ax demand | Tilt-equivalent ax |
| --- | --- | --- | --- | --- |
| 15.0 s | -0.081 m/s | +0.111 m/s | +0.346 m/s² | +0.205 m/s² |
| 15.5 s | -0.108 m/s | +0.155 m/s | +0.503 m/s² | +0.255 m/s² |
| 16.0 s | -0.046 m/s | +0.194 m/s | +0.409 m/s² | +0.425 m/s² |

Negative x is the outward excursion at this turn. Despite return demand,
estimated x moves from -1.228 m at 15.0 s to about -1.332 m at 16.1 s, another
10.4 cm. Velocity becomes positive around 16.2 s. This is direct evidence
that adding a stronger adapter speed cap is not enough to explain/correct the
overshoot. Tilt-equivalent acceleration assumes near-hover thrust; it is not
measured translational acceleration or proof of a motor fault. Demand/response
lag exists, but the data above do not isolate a single gain or estimator cause.

At the 0.295 m/s actual speed peak, internal PX4 velocity demand is 0.206 m/s.
Internal demand elsewhere reaches 0.390 m/s, so position feedback can also
raise demand above the adapter's 0.20 velocity feedforward. Both facts matter;
do not equate either feedforward or an instantaneous actual speed with demand.

Entrance roll/pitch angle-error RMS is about 1.30/1.39 degrees, rate-error RMS
5.75/5.72 deg/s. Interior values are 1.25/1.06 degrees and 5.24/4.99 deg/s.
Motor commands peak at 0.661 in the entrance window and 0.743 inside; none
exceeds 0.95 in those windows. This does not establish physical motor response.

The ULog records firmware `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`,
`EKF2_EV_DELAY=225`, roll/pitch rate P=0.15, I=0.10, D=0.003, attitude P=4.0,
`MPC_XY_P=0.95`, XY velocity P=1.8/I=0.4/D=0.2 and `MPC_XY_VEL_MAX=1.0`.
No parameter changes appear in the ULog. These are flight-recorded settings,
not a fresh live readback or suggested replacements.

[Entrance braking plot](../reports/migration_tunnel_20260921_0726/onboard/entrance_braking.png).
The same folder contains `alignment.json`, `control.csv/json`,
`attitude.csv/json/png` and raw ULog. Reproduce the two analyses with:

```bash
.venv-mavlink/bin/python scripts/analyze_control_response.py \
  reports/migration_tunnel_20260921_0726/onboard/px4_22_07_06.ulg \
  --time-offset -129.975776675 --start 7.847 --end 26.239 \
  --out reports/migration_tunnel_20260921_0726/onboard/control
.venv-mavlink/bin/python scripts/analyze_attitude_response.py \
  reports/migration_tunnel_20260921_0726/onboard/px4_22_07_06.ulg \
  --time-offset -129.975776675 --window entrance:13:16.7 \
  --window interior:18:25 --plot \
  --out reports/migration_tunnel_20260921_0726/onboard/attitude
```

## Artifacts and method

Local reports (git-ignored, retained in this workspace):

- [Trajectory on map](../reports/migration_tunnel_20260921_0726/flight_maps.png)
- [Speed, positions, clearance and yaw](../reports/migration_tunnel_20260921_0726/flight_performance.png)
- [Metrics](../reports/migration_tunnel_20260921_0726/metrics.json)
- `reports/migration_tunnel_20260921_0726/072654.json`: decoded messages.
- Same folder: `extract.py`, `analyze.py`, `probe.cpp`, `probe`, event text,
  decode-error counts, extraction/build/check logs.

The map picture overlays historical corrected body poses on the map at first
arrival; localization/map changes can shift historical tracks. Quantitative
clearances instead use each sample's latest grid and exactly source-stamp-paired
body/correction, with the migration `grid_clearance.cpp` implementation and
occupied threshold 65. The probe was rebuilt from this branch. Primary metrics
end at first arrival to exclude the later reset. Timing uses bag receipt times.
The extraction reports decode failures for 66 failsafe-flag and 79 vehicle-status
messages; conclusions above use decoded localization, setpoints, follower and
adapter logs, not a claim to have fully decoded PX4 status/failsafe telemetry.

To regenerate plots from the retained decoded JSON:

```bash
cd /home/john/autonomous_drone_px4_vio
g++ -std=c++17 -O2 -I ros2_ws/src/px4_vio_bridge/include \
  reports/migration_tunnel_20260921_0726/probe.cpp \
  ros2_ws/src/px4_vio_bridge/src/grid_clearance.cpp \
  -o reports/migration_tunnel_20260921_0726/probe
.venv-mavlink/bin/python reports/migration_tunnel_20260921_0726/analyze.py
```

## Verification performed this session

```bash
cd /home/john/autonomous_drone_px4_vio
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
colcon build --base-paths ros2_ws/src --build-base ros2_ws/build \
  --install-base ros2_ws/install --packages-select px4_vio_bridge \
  --symlink-install --parallel-workers 2
source ros2_ws/install/setup.bash
ctest --test-dir ros2_ws/build/px4_vio_bridge --output-on-failure \
  -R '^(test_localization_pair_cpp|test_grid_clearance_cpp|test_planner_flight_cpp|test_localization_pair_integration|test_backend_launch)$'
git diff --check
```

Before the speed trial was withdrawn, incremental package build passed. Five
selected test targets passed in 17.02 s.
Build emitted the existing external depthai overlay `local_setup.*` warning;
it did not fail. A separate launch-construction check confirmed both 0.18
defaults, both explicit 0.20 overrides, unchanged 0.30 acceleration and installed
launch/source byte equality, without starting nodes. No new tests were added for
that reversible parameter change. Work was performed by the lead agent only.

The installed C++ adapter remains byte-identical to the restored migration build:
SHA256 `8cf86687d55f68178f8db9365890a0d8a6a5cdf7efebbfc3f94424a3d9fb0577`.
Launch files use symlink installation; the restored source defaults are 0.20.
After withdrawal, both source and installed launch files were checked byte for
byte against `893cbea` and matched exactly. No further runtime edits were made;
the follow-up changes are documentation and offline analysis, so the runtime
suite was not repeated.
The previous experimental build
and restoration logs remain under
`/home/john/flight_branch_backups/migration_restore_20260921T071536Z`.
Build/tests establish static compatibility, not flight performance.
