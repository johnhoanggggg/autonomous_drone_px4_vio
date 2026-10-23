"""Compare the 0.165 and 0.150 MC_ROLLRATE_P lateral-stop flights."""
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from analyze_attitude_response import analyse as attitude_analyse, wrapped_error
from analyze_lateral_stop import summarize


FLIGHTS = [
    dict(flight="085406", ulog="23_32_28.ulg", bytes=5_861_396,
         rollrate_p=0.165, ros_tail_truncated=False),
    dict(flight="091448", ulog="23_52_43.ulg", bytes=4_758_798,
         rollrate_p=0.150, ros_tail_truncated=True),
]


def aggregate_tracking(columns, masks):
    keep = np.logical_or.reduce(masks)
    result = {"samples": int(keep.sum())}
    for axis in ("roll", "pitch", "yaw"):
        angle_error = wrapped_error(columns[f"{axis}_actual_deg"][keep],
                                    columns[f"{axis}_target_deg"][keep])
        rate_error = (columns[f"{axis}_rate_actual_deg_s"][keep]
                      - columns[f"{axis}_rate_target_deg_s"][keep])
        result[axis] = dict(
            angle_error_rms_deg=float(np.sqrt(np.mean(angle_error ** 2))),
            angle_error_max_abs_deg=float(np.max(np.abs(angle_error))),
            rate_error_rms_deg_s=float(np.sqrt(np.mean(rate_error ** 2))),
            rate_error_max_abs_deg_s=float(np.max(np.abs(rate_error))),
        )
    motors = np.array([columns[f"motor_{i}"][keep] for i in range(4)])
    result["motors"] = dict(peak=float(motors.max()),
                            fraction_above_095=float(np.mean(motors > 0.95)))
    return result


results = []
alignments = []
fig, axes = plt.subplots(4, 2, figsize=(14, 11), sharex="col", layout="constrained")
first_fig, first_axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True,
                                     layout="constrained")

for col, spec in enumerate(FLIGHTS):
    flight = spec["flight"]
    bag = json.loads((OUT / f"{flight}.data.json").read_text())
    ulog_path = OUT / spec["ulog"]
    assert ulog_path.stat().st_size == spec["bytes"]
    log = ULog(str(ulog_path))
    data = {item.name: item.data for item in log.data_list if item.multi_id == 0}
    local = data["vehicle_local_position"]

    # Pair on exact PX4 sample timestamps; filenames and host clocks are not enough.
    ros_by_boot = {round(row["boot"] * 1e6): row for row in bag["local"]}
    matches = [(i, ros_by_boot[int(ts)]) for i, ts in enumerate(local["timestamp"])
               if int(ts) in ros_by_boot]
    assert len(matches) > 1000
    offset = float(np.median([row["time"] - local["timestamp"][i] / 1e6
                              for i, row in matches]))
    position_difference = max(abs(local["x"][i] - row["x"]) for i, row in matches)
    assert position_difference == 0

    landing = min(message.timestamp / 1e6 + offset for message in log.logged_messages
                  if "Landing at current position" in message.message)
    status = data["vehicle_status"]
    changes = np.r_[True, (np.diff(status["nav_state"]) != 0)
                    | (np.diff(status["arming_state"]) != 0)]
    alignment = dict(
        flight=flight, ulog=spec["ulog"], bytes=spec["bytes"], offset=offset,
        exact_local_position_matches=len(matches),
        maximum_position_difference_m=float(position_difference),
        onboard_end_s=float(log.last_timestamp / 1e6 + offset),
        landing_event_s=float(landing),
        status_transitions=[dict(time_s=float(t / 1e6 + offset),
                                 arming_state=int(a), nav_state=int(n))
                            for t, a, n in zip(status["timestamp"][changes],
                                               status["arming_state"][changes],
                                               status["nav_state"][changes])],
        dropouts=[dict(time_s=float(item.timestamp / 1e6 + offset),
                       duration_ms=int(item.duration)) for item in log.dropouts],
    )
    alignments.append(alignment)

    # LAND keeps the last leg in debug fields, so retain only active test states.
    active = [row for row in bag["debug"]
              if row["state"] in ("LATERAL_MOVE", "LATERAL_HOLD")]
    result = summarize(dict(bag, debug=active))
    result.update(flight=flight, rollrate_p=spec["rollrate_p"],
                  ulog=spec["ulog"], alignment=alignment,
                  ros_tail_truncated=spec["ros_tail_truncated"], events=bag["events"])
    assert result["config"]["brake_damping"] is False
    assert len(result["legs"]) == 3

    trajectory = data["trajectory_setpoint"]
    trajectory_time = trajectory["timestamp"] / 1e6 + offset
    yaw0 = bag["debug"][0]["yaw0"]
    origin = np.array(bag["debug"][0]["origin"])
    lateral_axis = np.array([-math.sin(yaw0), math.cos(yaw0)])
    forward_axis = np.array([math.cos(yaw0), math.sin(yaw0)])
    position = np.column_stack([local["x"], local["y"]]) - origin
    velocity = np.column_stack([local["vx"], local["vy"]])
    right = position @ lateral_axis
    forward = position @ forward_axis
    lateral_velocity = velocity @ lateral_axis
    horizontal_speed = np.linalg.norm(velocity, axis=1)
    local_time = local["timestamp"] / 1e6 + offset
    yaw_error = np.degrees(np.arctan2(np.sin(local["heading"] - yaw0),
                                      np.cos(local["heading"] - yaw0)))
    reference_position = (np.column_stack([trajectory["position[0]"],
                                           trajectory["position[1]"]]) - origin) @ lateral_axis
    reference_velocity = np.column_stack([trajectory["velocity[0]"],
                                          trajectory["velocity[1]"]]) @ lateral_axis

    # The final hold timed out in both flights. Use the last stationary target
    # publication before the onboard landing event, recovering the truncated bag.
    final_leg = result["legs"][-1]
    stationary_final = ((trajectory_time < landing)
                        & (np.abs(reference_velocity) < 1e-6)
                        & (np.abs(reference_position - final_leg["target_m"]) < 1e-3))
    assert stationary_final.any()
    final_leg["hold_end_s"] = float(trajectory_time[stationary_final][-1])
    final_leg["hold_end_recovered_from_onboard"] = True
    assert 9.8 < final_leg["hold_end_s"] - final_leg["reference_stop_s"] < 10.2

    battery = data["battery_status"]
    battery_time = battery["timestamp"] / 1e6 + offset
    for leg in result["legs"]:
        start, brake = leg["start_s"], leg["brake_s"]
        stop, end = leg["reference_stop_s"], leg["hold_end_s"]
        sign = 1 if leg["direction"] == "right" else -1
        move = (local_time >= start) & (local_time <= end)
        after_brake = (local_time >= brake) & (local_time <= end)
        hold = (local_time >= stop) & (local_time <= end)
        first3 = hold & (local_time < stop + 3)
        at_brake = int(np.argmin(np.abs(local_time - brake)))
        at_stop = int(np.argmin(np.abs(local_time - stop)))
        reversals = np.flatnonzero(after_brake & (sign * lateral_velocity <= 0))
        reversal_time = local_time[reversals[0]] if len(reversals) else end
        first_stop = after_brake & (local_time <= reversal_time)
        radial_error = np.hypot(right - leg["target_m"], forward)
        gates = dict(position=radial_error <= result["config"]["settle_position"],
                     speed=horizontal_speed <= result["config"]["settle_speed"],
                     yaw=np.abs(yaw_error) <= 5,
                     altitude=np.abs(-local["z"] - result["config"]["hover_height"]) <= .07)
        all_gates = np.logical_and.reduce(list(gates.values()))
        began = None
        longest = eligible = 0.0
        for timestamp, good in zip(local_time[hold], all_gates[hold]):
            if good:
                if began is None:
                    began = timestamp
                longest = max(longest, timestamp - began)
                if timestamp - stop >= result["config"]["settle_hold"]:
                    eligible = max(eligible, timestamp - began)
            else:
                began = None
        loaded = ((battery_time >= stop) & (battery_time < stop + 3)
                  & (battery["current_a"] > 20))
        leg.update(
            peak_speed_m_s=float(np.max(sign * lateral_velocity[move])),
            speed_at_brake_m_s=float(sign * lateral_velocity[at_brake]),
            speed_at_reference_stop_m_s=float(sign * lateral_velocity[at_stop]),
            average_braking_deceleration_m_s2=float(
                sign * (lateral_velocity[at_brake] - lateral_velocity[at_stop])
                / (stop - brake)),
            first_reversal_after_brake_s=float(reversal_time - brake),
            first_stop_endpoint_overshoot_m=float(max(
                0, np.max(sign * (right[first_stop] - leg["target_m"])))),
            endpoint_overshoot_m=float(max(
                0, np.max(sign * (right[after_brake] - leg["target_m"])))),
            hold_duration_s=float(end - stop),
            hold_span_m=float(np.ptp(right[hold])),
            first3_span_m=float(np.ptp(right[first3])),
            first3_lateral_velocity_rms_m_s=float(
                np.sqrt(np.mean(lateral_velocity[first3] ** 2))),
            first3_horizontal_speed_p95_m_s=float(
                np.percentile(horizontal_speed[first3], 95)),
            hold_max_forward_error_m=float(np.max(np.abs(forward[hold]))),
            hold_max_abs_yaw_error_deg=float(np.max(np.abs(yaw_error[hold]))),
            hold_speed_above_gate_fraction=float(np.mean(~gates["speed"][hold])),
            gate_failure_fraction={name: float(np.mean(~values[hold]))
                                   for name, values in gates.items()},
            longest_joint_dwell_s=float(longest),
            eligible_joint_dwell_s=float(eligible),
            loaded_voltage_median_v=float(np.median(battery["voltage_v"][loaded])),
            loaded_current_median_a=float(np.median(battery["current_a"][loaded])),
        )

    windows = []
    for leg in result["legs"]:
        number = leg["leg"] + 1
        windows.extend([(f"leg{number}_brake", leg["brake_s"], leg["reference_stop_s"]),
                        (f"leg{number}_hold_first3", leg["reference_stop_s"],
                         leg["reference_stop_s"] + 3)])
    attitude_columns, attitude_summary = attitude_analyse(ulog_path, offset, windows)
    result["attitude_windows"] = attitude_summary["windows"]
    result["tracking_aggregate"] = {}
    for name, ranges in {
        "all_braking": [(leg["brake_s"], leg["reference_stop_s"]) for leg in result["legs"]],
        "all_holds_first3": [(leg["reference_stop_s"], leg["reference_stop_s"] + 3)
                             for leg in result["legs"]],
    }.items():
        masks = [(attitude_columns["time_s"] >= lo) & (attitude_columns["time_s"] < hi)
                 for lo, hi in ranges]
        result["tracking_aggregate"][name] = aggregate_tracking(attitude_columns, masks)

    result["parameters"] = {name: log.initial_parameters[name] for name in (
        "MC_ROLLRATE_P", "MC_ROLLRATE_I", "MC_ROLLRATE_D", "MC_ROLLRATE_K",
        "MC_PITCHRATE_P", "MC_PITCHRATE_I", "MC_PITCHRATE_D", "MC_PITCHRATE_K",
        "MC_ROLL_P", "MC_PITCH_P", "MC_YAW_P", "MPC_XY_P", "MPC_XY_VEL_P_ACC",
        "MPC_XY_VEL_I_ACC", "MPC_XY_VEL_D_ACC", "EKF2_EV_DELAY",
        "SENS_BOARD_X_OFF", "SENS_BOARD_Y_OFF", "SENS_BOARD_Z_OFF")}
    assert abs(result["parameters"]["MC_ROLLRATE_P"] - spec["rollrate_p"]) < 1e-5
    result["parameter_changes_during_flight"] = log.changed_parameters
    assert not result["parameter_changes_during_flight"]
    result["estimator_reset_counters"] = {
        key: np.unique(local[key]).tolist() for key in local if "reset_counter" in key}

    # Whole-flight and equal first-stop visuals.
    first_stop = result["legs"][0]["reference_stop_s"]
    plot_end = result["legs"][-1]["hold_end_s"]
    keep = (local_time >= result["legs"][0]["start_s"] - .3) & (local_time <= plot_end)
    ref_keep = ((trajectory_time >= result["legs"][0]["start_s"] - .3)
                & (trajectory_time <= plot_end))
    axes[0, col].plot(local_time[keep], right[keep], label="Estimated")
    axes[0, col].plot(trajectory_time[ref_keep], reference_position[ref_keep], label="Reference")
    axes[1, col].plot(local_time[keep], lateral_velocity[keep])
    axes[2, col].plot(local_time[keep], forward[keep])
    axes[3, col].plot(local_time[keep], yaw_error[keep])
    axes[0, col].set_title(f"{flight}: MC_ROLLRATE_P={spec['rollrate_p']:.3f}")
    for row, label in enumerate(("Lateral position (m)", "Lateral velocity (m/s)",
                                 "Forward position (m)", "Yaw error (deg)")):
        axes[row, col].set_ylabel(label)
        axes[row, col].grid(alpha=.25)
        axes[row, col].axhline(0, color="gray", lw=.6)
    axes[0, col].legend(fontsize=8)
    axes[3, col].set_xlabel("Bag-aligned seconds")

    first = ((local_time >= first_stop) & (local_time < first_stop + 3))
    relative_time = local_time[first] - first_stop
    label = f"P={spec['rollrate_p']:.3f}"
    first_axes[0].plot(relative_time, right[first] - .35, label=label)
    first_axes[1].plot(relative_time, lateral_velocity[first], label=label)
    first_axes[2].plot(relative_time, forward[first], label=label)
    first_axes[3].plot(relative_time, yaw_error[first], label=label)
    results.append(result)

# Only the selected gain and ordinary flight counters may differ.
p0 = ULog(str(OUT / FLIGHTS[0]["ulog"])).initial_parameters
p1 = ULog(str(OUT / FLIGHTS[1]["ulog"])).initial_parameters
parameter_differences = {key: [p0.get(key), p1.get(key)] for key in sorted(set(p0) | set(p1))
                         if p0.get(key) != p1.get(key)}
assert set(parameter_differences) == {"COM_FLIGHT_UUID", "LND_FLIGHT_T_LO", "MC_ROLLRATE_P"}

fig.suptitle("Roll-rate P trial: damping OFF in both flights")
fig.savefig(OUT / "all_flights.png", dpi=160)
for axis, label in zip(first_axes, ("Lateral endpoint error (m)", "Lateral velocity (m/s)",
                                    "Forward position error (m)", "Yaw error (deg)")):
    axis.set_ylabel(label)
    axis.grid(alpha=.25)
    axis.axhline(0, color="gray", lw=.6)
    axis.legend(fontsize=8)
first_axes[-1].set_xlabel("Seconds after first reference stop")
first_fig.suptitle("Equal-duration first hold comparison")
first_fig.savefig(OUT / "first_hold_comparison.png", dpi=160)

(OUT / "alignment.json").write_text(json.dumps(alignments, indent=2) + "\n")
(OUT / "comparison.json").write_text(json.dumps(dict(
    flights=results, parameter_differences=parameter_differences), indent=2) + "\n")

for result in results:
    print(f"\n{result['flight']} P={result['rollrate_p']:.3f}")
    for leg in result["legs"]:
        print("leg", leg["leg"] + 1,
              "hold", round(leg["hold_duration_s"], 2),
              "first_over", round(100 * leg["first_stop_endpoint_overshoot_m"], 1),
              "eventual_over", round(100 * leg["endpoint_overshoot_m"], 1),
              "first3_rms", round(leg["first3_lateral_velocity_rms_m_s"], 3),
              "span", round(100 * leg["hold_span_m"], 1))
    print("tracking", json.dumps(result["tracking_aggregate"], indent=2))
print("parameter differences", json.dumps(parameter_differences, indent=2))
