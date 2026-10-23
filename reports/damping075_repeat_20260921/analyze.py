"""Analyse the 10:12 gain-0.75 damping repeat against its matched ULog."""
import json
import math
import sys
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from analyze_attitude_response import analyse as attitude_analyse
from analyze_control_response import analyse as control_analyse
from analyze_lateral_stop import summarize


bag = json.loads((OUT / "101233.data.json").read_text())
ulog_path = OUT / "00_49_15.ulg"
ulog = ULog(str(ulog_path))
topics = {item.name: item.data for item in ulog.data_list if item.multi_id == 0}
lp = topics["vehicle_local_position"]

# Use identical PX4 timestamps carried by ROS, rather than wall-clock filenames.
by_boot = {round(row["boot"] * 1e6): row for row in bag["local"]}
matches = [(i, by_boot[int(stamp)]) for i, stamp in enumerate(lp["timestamp"])
           if int(stamp) in by_boot]
assert len(matches) > 1000
assert max(abs(float(lp["x"][i]) - row["x"]) for i, row in matches) == 0
offset = float(np.median([row["time"] - lp["timestamp"][i] / 1e6
                          for i, row in matches]))

active = [row for row in bag["debug"]
          if row["state"] in ("LATERAL_MOVE", "LATERAL_HOLD")]
result = summarize(dict(bag, debug=active))
result["flight"] = "101233"
result["ros_tail_truncated"] = True
result["alignment"] = {
    "ulog": str(ulog_path.relative_to(ROOT)),
    "bytes": ulog_path.stat().st_size,
    "offset": offset,
    "matches": len(matches),
    "position_difference": 0.0,
    "onboard_end": float(ulog.last_timestamp / 1e6 + offset),
    "events": [{"time": message.timestamp / 1e6 + offset,
                "text": message.message.strip()}
               for message in ulog.logged_messages],
    "dropouts": [{"time": dropout.timestamp / 1e6 + offset,
                  "duration_ms": dropout.duration}
                 for dropout in ulog.dropouts],
}

# ROS stopped during the final hold. The first external NAV_LAND command gives
# its real end; below we independently verify that all gates had remained true
# for the required one-second dwell immediately before this command.
commands = topics["vehicle_command"]
land_commands = [float(stamp / 1e6 + offset)
                 for stamp, command, external in
                 zip(commands["timestamp"], commands["command"], commands["from_external"])
                 if int(command) == 21 and bool(external)]
assert land_commands
result["legs"][-1]["ros_recording_end_s"] = result["legs"][-1]["hold_end_s"]
result["legs"][-1]["hold_end_s"] = min(land_commands)

t = lp["timestamp"] / 1e6 + offset
yaw = bag["debug"][0]["yaw0"]
origin = np.asarray(bag["debug"][0]["origin"])
lateral = np.array([-np.sin(yaw), np.cos(yaw)])
forward = np.array([np.cos(yaw), np.sin(yaw)])
position = np.column_stack([lp["x"], lp["y"]]) - origin
right = position @ lateral
front = position @ forward
velocity = np.column_stack([lp["vx"], lp["vy"]])
vr = velocity @ lateral
speed = np.linalg.norm(velocity, axis=1)
yaw_error = np.degrees(np.arctan2(np.sin(lp["heading"] - yaw),
                                  np.cos(lp["heading"] - yaw)))

trajectory = topics["trajectory_setpoint"]
tt = trajectory["timestamp"] / 1e6 + offset
ref_position = (np.column_stack([trajectory["position[0]"],
                                 trajectory["position[1]"]]) - origin)
ref_velocity = np.column_stack([trajectory["velocity[0]"],
                                trajectory["velocity[1]"]])
ref_acceleration = np.column_stack([trajectory["acceleration[0]"],
                                    trajectory["acceleration[1]"]])

windows = [(f"leg{leg['leg'] + 1}_hold_first3",
            leg["reference_stop_s"], leg["reference_stop_s"] + 3)
           for leg in result["legs"]]
_, attitude = attitude_analyse(ulog_path, offset, windows)
result["attitude_first3"] = attitude["windows"]

for leg in result["legs"]:
    start = leg["reference_stop_s"]
    end = leg["hold_end_s"]
    hold = (t >= start) & (t <= end)
    first3 = hold & (t < start + 3)
    target = leg["target_m"]
    gates = {
        "position": np.hypot(right - target, front) <= .1,
        "speed": speed <= .06,
        "yaw": np.abs(yaw_error) <= 5,
        "altitude": np.abs(-lp["z"] - .3) <= .07,
    }
    wire = (tt >= start + .1) & (tt <= end - .1)
    acceleration = np.linalg.norm(ref_acceleration[wire], axis=1)
    assert acceleration.max() <= .100001
    assert np.max(np.ptp(ref_position[wire], axis=0)) == 0
    assert np.max(np.abs(ref_velocity[wire])) == 0
    began = None
    dwell_segments = []
    combined_gate = np.logical_and.reduce(list(gates.values()))
    hold_indices = np.flatnonzero(hold)
    for index in hold_indices:
        if combined_gate[index] and began is None:
            began = index
        elif not combined_gate[index] and began is not None:
            dwell_segments.append((float(t[began]), float(t[index - 1]),
                                   float(t[index - 1] - t[began])))
            began = None
    if began is not None:
        dwell_segments.append((float(t[began]), float(t[hold_indices[-1]]),
                               float(t[hold_indices[-1]] - t[began])))
    sign = 1 if leg["direction"] == "right" else -1
    after_brake = (t >= leg["brake_s"]) & (t <= end)
    leg.update(
        hold_duration_s=end - start,
        hold_span_m=float(np.ptp(right[hold])),
        first3_span_m=float(np.ptp(right[first3])),
        first3_speed_p95=float(np.percentile(speed[first3], 95)),
        first3_lateral_speed_rms=float(np.sqrt(np.mean(vr[first3] ** 2))),
        gate_failure_fraction={name: float(np.mean(~passed[hold]))
                               for name, passed in gates.items()},
        longest_joint_gate_dwell_s=max((segment[2] for segment in dwell_segments), default=0.),
        joint_gate_dwell_segments=dwell_segments,
        endpoint_overshoot_m=float(max(0., np.max(sign * (right[after_brake] - target)))),
        hold_max_forward_error_m=float(np.max(np.abs(front[hold]))),
        hold_max_abs_yaw_error_deg=float(np.max(np.abs(yaw_error[hold]))),
        hold_speed_above_gate_fraction=float(np.mean(speed[hold] > .06)),
        onboard_hold_acceleration_max=float(acceleration.max()),
        onboard_hold_acceleration_p95=float(np.percentile(acceleration, 95)),
        hold_correction_above_095_fraction=float(np.mean(acceleration > .095)),
    )

result["estimator_reset_counters"] = {
    key: np.unique(lp[key]).tolist() for key in lp if "reset_counter" in key
}
assert all(len(values) == 1 for values in result["estimator_reset_counters"].values())
result["damping_reasons"] = dict(Counter(row["damping"]["reason"] for row in active))
result["maximum_velocity_age_s"] = max(row["damping"]["velocity_age_s"] for row in active)
result["max_applied_correction_m_s2"] = max(
    math.hypot(*row["damping"]["applied_correction_ned"]) for row in active)

battery = topics["battery_status"]
bt = battery["timestamp"] / 1e6 + offset
first_start = result["legs"][0]["reference_stop_s"]
first3_battery = (bt >= first_start) & (bt < first_start + 3)
result["first_hold_voltage_median"] = float(np.median(battery["voltage_v"][first3_battery]))
result["first_hold_current_median"] = float(np.median(battery["current_a"][first3_battery]))
result["parameters"] = {key: value for key, value in ulog.initial_parameters.items()
                        if key.startswith(("MPC_XY", "MC_ROLL", "MC_PITCH",
                                           "MC_YAW", "EKF2_EV_DELAY"))}
result["parameter_changes"] = ulog.changed_parameters
assert not result["parameter_changes"]

landing_times = [event["time"] for event in result["alignment"]["events"]
                 if "Landing at current position" in event["text"]]
result["completion_evidence"] = {
    "final_hold_end_s": result["legs"][-1]["hold_end_s"],
    "first_nav_land_command_s": min(land_commands),
    "first_landing_event_s": min(landing_times),
    "continuous_joint_gate_dwell_before_land_s":
        result["legs"][-1]["joint_gate_dwell_segments"][-1][2],
    "classification": "completed_all_four_holds",
}
assert result["completion_evidence"]["continuous_joint_gate_dwell_before_land_s"] >= 1.0

# Add a compact comparison with the previous gain-0.75 ON/OFF/ON batch.
history = json.loads((ROOT / "reports/lateral_damping_0827_20260921/comparison.json").read_text())
comparison = []
for previous in history + [result]:
    comparison.append({
        "flight": previous["flight"],
        "damping": previous["config"]["brake_damping"],
        "completed_holds": sum(leg["hold_duration_s"] < 9 for leg in previous["legs"]),
        "hold_count": len(previous["legs"]),
        "max_endpoint_overshoot_m": max(leg["endpoint_overshoot_m"] for leg in previous["legs"]),
        "max_hold_span_m": max(leg["hold_span_m"] for leg in previous["legs"]),
        "first3_lateral_speed_rms_m_s": previous["legs"][0]["first3_lateral_speed_rms"],
        "first_hold_voltage_median_v": previous["first_hold_voltage_median"],
    })
result["historical_comparison"] = comparison

(OUT / "analysis.json").write_text(json.dumps(result, indent=2) + "\n")

# Plot the complete lateral sequence and four equal-duration hold windows.
fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, layout="constrained")
axes[0].plot(t, right, label="Estimated position")
axes[0].plot(tt, ref_position @ lateral, label="Reference")
axes[1].plot(t, vr, label="Estimated lateral velocity")
axes[2].plot(tt, ref_acceleration @ lateral, label="Published lateral acceleration")
for leg in result["legs"]:
    for axis in axes:
        axis.axvspan(leg["brake_s"], leg["reference_stop_s"], color="orange", alpha=.13)
        axis.axvline(leg["hold_end_s"], color="green", alpha=.25)
for axis, ylabel in zip(axes, ("Lateral position (m)", "Lateral velocity (m/s)",
                               "Feedforward acceleration (m/s²)")):
    axis.set_ylabel(ylabel)
    axis.grid(alpha=.2)
    axis.legend(fontsize=8)
axes[-1].set_xlabel("Bag-aligned seconds; orange = reference braking, green = passed hold")
fig.suptitle("10:12 gain-0.75 repeat: all four holds passed")
fig.savefig(OUT / "flight.png", dpi=160)

fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, layout="constrained")
for leg in result["legs"]:
    keep = (t >= leg["reference_stop_s"]) & (t < leg["reference_stop_s"] + 3)
    elapsed = t[keep] - leg["reference_stop_s"]
    label = f"leg {leg['leg'] + 1}"
    axes[0].plot(elapsed, right[keep] - leg["target_m"], label=label)
    axes[1].plot(elapsed, speed[keep], label=label)
    axes[2].plot(elapsed, yaw_error[keep], label=label)
axes[0].axhline(.1, color="gray", ls="--"); axes[0].axhline(-.1, color="gray", ls="--")
axes[1].axhline(.06, color="gray", ls="--")
axes[2].axhline(5, color="gray", ls="--"); axes[2].axhline(-5, color="gray", ls="--")
for axis, ylabel in zip(axes, ("Endpoint error (m)", "Horizontal speed (m/s)",
                               "Yaw error (deg)")):
    axis.set_ylabel(ylabel); axis.grid(alpha=.2); axis.legend(fontsize=8)
axes[-1].set_xlabel("Seconds after reference stop")
fig.suptitle("First three seconds of each hold")
fig.savefig(OUT / "holds_first3.png", dpi=160)

print(json.dumps({
    "alignment": result["alignment"],
    "completion_evidence": result["completion_evidence"],
    "historical_comparison": comparison,
    "legs": result["legs"],
    "first_hold_voltage_median": result["first_hold_voltage_median"],
    "first_hold_current_median": result["first_hold_current_median"],
    "maximum_velocity_age_s": result["maximum_velocity_age_s"],
    "max_applied_correction_m_s2": result["max_applied_correction_m_s2"],
}, indent=2))
