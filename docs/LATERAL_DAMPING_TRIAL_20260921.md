# Lateral-stop velocity damping trial

**Latest, 2026-09-21:** [gain 0.75 ON/OFF/ON results](LATERAL_DAMPING_GAIN075_RESULTS_20260921.md).
One full completion; OFF failed its first hold; the other ON run reached the
final hold but remained unsettled. Oscillation is still inconsistent, with
different battery conditions. Retain 0.75 only as a test override; do not
increase gain/cap or port to the tunnel yet.
The [initial gain 0.5 results](LATERAL_DAMPING_RESULTS_20260921.md) are historical.
The preparation and validation record below describes the initial implementation.

Branch: `tune/migration-tunnel-smoothing`, based on migration `893cbea`.
Background: [lateral-stop analysis](LATERAL_BRAKING_REVIEW_20260921.md).
The user authorized implementing and testing the proposed damping experiment.
Implementation, installation, offline checks and tests are complete. No live
flight or PX4 parameter change was performed.

## Change

The lateral-stop adapter now accepts `brake_damping:=true`. It is **false by
default**; the tunnel/global C++ adapter and its launch files remain unchanged.

During reference braking and lateral hold, the added horizontal feedback is:

```text
raw correction = damping_gain * (reference NED velocity - measured NED velocity)
target correction = vector-limit(raw correction, damping_limit)
filtered correction = first-order filter(target correction, damping_time_constant)
published acceleration = vector-limit(reference acceleration + filtered correction, acceleration)
```

Trial defaults: gain **0.5 s⁻¹**, correction cap **0.10 m/s²**, filter time
constant **0.10 s**. The existing total horizontal acceleration-feedforward
cap stays **0.30 m/s²**, cruise **0.20 m/s**, leg distance **0.35 m**.
Position/velocity references, heading, arrival/dwell gates, geofence and abort
logic retain the original profile. Normal exits from braking/hold decay the
correction toward zero, so a small residual can taper into the following phase.
Climb, landing, abort and other inactive states clear it immediately.

Measured velocity must be finite, valid and received within 100 ms. Missing or
stale velocity clears the correction; the existing 250 ms flight-state stale
data guard is retained. Callback gaps over 150 ms or non-increasing callback
time reset the damping filter. Disabled/zero-gain operation returns the original
acceleration exactly, including lower-acceleration baseline configurations.

`/lateral_stop/debug` retains its existing reference fields and adds `damping`:
reason, velocity age, measured NED velocity, reference acceleration, raw/filtered
correction, actual correction after total clipping, and published acceleration.
The config contains the enable flag and all trial parameters. The existing bag
recorder captures this topic automatically.

## User-run comparison flight

Use the existing SLAM/VIO/PX4 bridge setup and run this **in place of** the
tunnel flight adapter. Use the established open-area test space with at least
1 m clear on either side; this scripted test has no obstacle avoidance.

```bash
cd /home/john/autonomous_drone_px4_vio
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
source ros2_ws/install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge offboard_lateral_stop.launch.py \
  auto_arm:=true brake_damping:=true \
  damping_gain:=0.5 damping_limit:=0.10 damping_time_constant:=0.10
```

That command arms and flies the right / return / left / return sequence; it
records automatically. Existing keyboard controls remain `L` to request landing
and `K` to force-disarm. For the baseline comparison, use the same setup and
arguments with **`brake_damping:=false`**. No rebuild is needed to switch.
Try to compare similar battery condition; the historical final legs had lower
loaded voltage than the initial ones.

Compare endpoint overshoot, peak-to-peak hold swing, residual speed after the
reference stop, time satisfying the settling gate and roll/pitch-rate tracking.
Do not retain a change that reduces position overshoot but increases sustained
attitude oscillation or settling time. Verify the recorded enable flag and
correction trace before attributing any difference to damping. The historical
baseline's largest hold swings were 22.5 cm and 18.7 cm.

Only consider porting to the tunnel adapter after this test demonstrates a
benefit. Additional feedback can interact with the observed response delay;
the successful command-bound checks do not establish flight stability.

## Implementation and verification

- `px4_vio_bridge/brake_damping.py`: small ROS-independent damping helper.
- `px4_vio_bridge/offboard_lateral_stop.py`: optional integration and telemetry.
- `launch/offboard_lateral_stop.launch.py`: typed opt-in launch arguments.
- Tests cover disabled/zero-gain baseline equivalence, direction/frame rotation,
  correction and total vector caps, smooth entry/release, timing reset,
  stale/invalid data, unchanged targets, flight-state exclusions, the four-leg
  sequence, debug serialization and launch argument forwarding.

Package build passed. Final relevant results: **83 cases passed**, no failures,
errors or skips: lateral-stop 49, inherited hover 20, launch 14. An initial test
caught a float32 debug-serialization error; the published values are now cast to
Python floats and the integration test passes. The inherited external depthai
overlay setup warning remains nonfatal.

```bash
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
colcon build --base-paths ros2_ws/src --build-base ros2_ws/build \
  --install-base ros2_ws/install --packages-select px4_vio_bridge \
  --symlink-install --parallel-workers 2
source ros2_ws/install/setup.bash
ctest --test-dir ros2_ws/build/px4_vio_bridge --output-on-failure \
  -R '^(test_offboard_lateral_stop|test_offboard_hover|test_backend_launch)$'
.venv-mavlink/bin/python reports/lateral_braking_20260921/analyze.py
git diff --check
```

All three targets passed together; the lateral-stop target was rerun after the
final lower-acceleration compatibility check was added. The offline analysis
now imports the **implemented helper** and evaluates it on both September 17
recordings. Both input-matching checks, fixed-hold-command checks, and damping
vector-bound assertions passed. This is recorded-input evaluation, not a
closed-loop vehicle simulation. See `reports/lateral_braking_20260921/` for
`damping_build.log`, `damping_checks.log`, `damping_final_lateral_check.log`,
`implemented_shadow.log`, numerical results and plots.

Installed helper/launch files are symlinked to source. The global C++ executable
retains SHA256 `8cf86687d55f68178f8db9365890a0d8a6a5cdf7efebbfc3f94424a3d9fb0577`.
Changes remain uncommitted. Work was performed by the lead agent only.
