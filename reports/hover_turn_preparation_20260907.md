# Hover-turn capture preparation

The user requested the short open-area hover-turn recording after the wall
strike diagnosis. No armed sequence has been started yet.

- Started `rtabmap_slam_px4.launch.py`, ROS domain 42. Log:
  `/tmp/hover_turn_vio_20260907.log`.
- PX4 disarmed; local XY/Z/velocity valid, heading good for control; EV
  position/yaw fusion active with no reported estimator rejection flags.
- VIO feature count approximately 400. Battery initially reported 73%, then
  the ROS monitor reported 65%; recheck before the actual flight.
- Restarted the idle logger with `logger start -b 64 -t -m all`. It confirmed
  **21 topics from logger_topics.txt**, including the three added control
  topics. This supersedes the previous report's pending-reboot requirement.
- `input_rc` has **never published** this boot and `COM_RC_IN_MODE=0`.
  Physical area/USB clearance and RC readiness still need the operator's input.
  README's offboard-hover section requires a bound transmitter as the manual
  override/kill switch.

Prepared bounded sequence uses the existing flown hover-yaw node: 0.30 m,
15° out at 5°/s, hold 5 s, return to initial heading, hold 5 s, AUTO.LAND.
Maximum flight watchdog 40 s; existing tracking/position watchdogs remain on.
There is no commanded horizontal travel. The node begins its yaw leg after
the existing altitude/position gate; it does not implement a timed initial
hover dwell.

Run only after physical readiness and fresh telemetry checks:

```bash
source /opt/ros/jazzy/setup.bash
source /home/john/autonomous_drone_px4_vio/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch px4_vio_bridge offboard_hold_yaw.launch.py \
  auto_arm:=true hover_height:=0.30 \
  yaw_angle_deg:=15.0 yaw_rate_deg:=5.0 \
  hold_time:=5.0 return_hold_time:=5.0 max_flight_time:=40.0
```

The launch starts its MCAP recorder before the flight node, names the bag
automatically, and closes it after completion. The PX4 logger starts on arm.
After landing, retrieve the newest ULog and verify the new control topics are
present before interpreting the response.
