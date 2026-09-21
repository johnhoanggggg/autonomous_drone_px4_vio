# First damping flight comparison — ON / OFF / ON

**Later trial:** the proposed 0.75 setting below has now flown. See
[08:27–08:29 results](LATERAL_DAMPING_GAIN075_RESULTS_20260921.md) for the
current conclusion. This document retains the initial 0.5 trial record.

Flights: September 21, 08:11:24, 08:12:16 and 08:12:54 UTC.
Branch `tune/migration-tunnel-smoothing`. This follows the
[implemented trial](LATERAL_DAMPING_TRIAL_20260921.md).

## Result

The recorded settings confirm **damping ON / OFF / ON**, all at gain 0.5,
correction limit 0.10 m/s² and filter time constant 0.10 s when enabled.
"OFF" disables the added damping; the original cosine-ramp braking remains.
Cruise, reference acceleration, distances and settling gates match across runs.

Damping is associated with better settling at the first stop in these runs.
Both enabled flights passed the first hold and continued; the disabled flight
timed out there. However, no flight completed all four legs. Repeated motion
during later stationary holds remains. This is promising evidence for the
small damping experiment, not a demonstrated solution to tunnel overshoot.

| Flight | Damping | First hold outcome | First 3 s lateral velocity RMS | First 3 s lateral swing | Final outcome |
| --- | --- | --- | --- | --- | --- |
| 08:11:24 | ON | Passed after 3.05 s | 0.019 m/s | 4.3 cm | Timed out in leg 2 hold |
| 08:12:16 | OFF | Timed out after 10.05 s | 0.042 m/s | 5.9 cm | Stopped after leg 1 |
| 08:12:54 | ON | Passed after 5.25 s | 0.024 m/s | 4.7 cm | Reached leg 3, then landed after its 10 s hold |

RMS is calculated over the same first three seconds after the reference stops.
This avoids presenting the baseline's full ten-second swing against shorter
enabled holds as an equal-duration comparison. Whole first-hold swings were
4.3 / 17.2 / 4.7 cm respectively; those hold durations differ.
The initial reference stops were short of the endpoint in all three runs.
The baseline's later endpoint overshoot reached 6.7 cm during renewed motion.

First-three-second roll-rate tracking-error RMS was 3.07 / 3.94 / 2.63 deg/s
respectively. Pitch-rate errors were 2.38 / 3.01 / 2.18 deg/s. These windows do
not show increased attitude oscillation with damping, but do not establish
stability at a larger gain or in the tunnel.

## What is still wrong

- Flight 1's return hold has **14.5 cm lateral swing** and 5.9 cm endpoint
  overshoot. Speed exceeds the settling gate about 39% of the hold; position
  fails about 13%. Forward drift reaches 11.3 cm. The best reconstructed dwell
  after the minimum hold time is only about 0.83 s, below the required 1 s.
- Flight 2's first hold has **17.2 cm swing**, with speed above the gate about
  61% of the time. It briefly calms earlier, but does not maintain the necessary
  dwell once the minimum hold duration is satisfied.
- Flight 3 passes its first two holds, then has **12.8 cm swing** and 10.6 cm
  endpoint overshoot at the left endpoint. Speed fails about 64% of this hold;
  position about 26%. Best reconstructed dwell is about 0.51 s.

These are real moving holds with constant onboard position targets and zero
reference velocity. Do not solve the timeout by relaxing the settling gates.
They correctly prevent another leg from starting while the vehicle drifts.
Roll/pitch response and estimator timing remain relevant to the underlying
problem; a stronger reference brake alone does not explain renewed hold motion.

## Confirmed application and onboard recovery

In enabled runs, all recorded active debug samples report braking/hold or phase
release; none reports stale/invalid velocity or callback-gap reset. Maximum
velocity ages are 33 ms and 42 ms. The disabled run reports `disabled` throughout.
Maximum recorded correction norms are **0.057 / 0 / 0.080 m/s²**. Onboard hold
acceleration is nonzero in enabled runs, zero in the disabled run, and remains
within the configured 0.10 correction cap. The option actually reached PX4.

Matching onboard files were retrieved read-only over USB; downloaded sizes
match the SD directory listing:

| ROS flight | PX4 SD file under `/fs/microsd/log/2026-09-18/` | Bytes | Exact local-position timestamp matches |
| --- | --- | --- | --- |
| 081124 | `22_50_46.ulg` | 4,150,539 | 1484 |
| 081216 | `22_51_37.ulg` | 3,491,229 | 1238 |
| 081254 | `22_52_16.ulg` | 5,079,888 | 1403 |

All exact matches have zero x-position discrepancy. The SD clock differs from
the host, so filenames alone were not used to pair logs. Relevant recorded
PX4 controller, board-offset and EV-delay parameters are identical between
flights, with no parameter changes in the ULogs.

The third ROS bag has empty metadata and a truncated final MCAP record. It
ends at about 34.24 s, during the third hold. Its complete ULog continues to
42.70 s: PX4 starts landing at about **38.40 s** and records external disarm
at **41.71 s**. The third leg target remains stationary through the reconstructed
ten-second hold; there is no fourth leg. Landing timing agrees with the configured
hold timeout, although the ROS timeout message itself is missing. The final
hold metrics use ULog data through reference-stop + 10 s, before landing.

Control quantities are paired on the PX4 clock. ROS/debug phase boundaries have
approximately one control tick of alignment uncertainty; reconstructed dwell
values are diagnostic estimates, not a replacement for the adapter's decision.
The baseline ULog has an 80 ms dropout during the first movement, before the
first-hold comparison window. Other nonzero dropouts are before lateral motion.

## Next controlled trial

Keep this change limited to the lateral-stop adapter. Try **gain 0.75 s⁻¹**,
keeping the correction cap **0.10 m/s²**, smoothing **0.10 s**, speed **0.20 m/s**
and total acceleration-feedforward cap **0.30 m/s²**. No code change is required:

```bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge offboard_lateral_stop.launch.py \
  auto_arm:=true brake_damping:=true damping_gain:=0.75 \
  damping_limit:=0.10 damping_time_constant:=0.10
```

Use the established open-area setup and source environment described in the
trial document. This command arms and flies. The default remains 0.5; the
proposed gain had not yet been flown at this report's initial writing. The added correction was below its cap in
these runs, so increasing the cap is not the first change to make. Increasing
gain may improve damping, but could worsen a delayed response; compare later
hold swing and attitude/rate oscillation before retaining it.

There is only one disabled control run, flight 1's first-hold voltage was about
11.18 V versus 10.95 V for flights 2/3, and flight 3 started at a different
position and roughly 80° heading versus 90°. These are not identical starting
conditions. Treat the improvement as an encouraging trend, and repeat a
baseline under similar conditions when comparing the next gain. Do not yet
copy the helper into the successful tunnel controller.

## Artifacts and validation

- [All three flights](reports/lateral_damping_flights_20260921/all_flights.png)
- [Equal-duration first-stop comparison](reports/lateral_damping_flights_20260921/first_stop_comparison.png)
- [Metrics and alignment](reports/lateral_damping_flights_20260921/comparison.json)
- [Analysis script](reports/lateral_damping_flights_20260921/analyze.py)

The same git-ignored folder holds decoded bag data, alignment evidence and
the three onboard files. Reproduce with:

```bash
.venv-mavlink/bin/python reports/lateral_damping_flights_20260921/analyze.py
git diff --check
```

The script passed its setting, data-validity, fixed-hold-position,
zero-reference-velocity and acceleration-bound assertions. Both figures were
inspected. No runtime code/settings were changed, so the package tests were
not rerun. This session only analysed recordings and updated handoff documents;
no flight commands or parameter writes were sent. Work was performed by the
lead agent only.
