"""Compare MPC_XY_VEL_I_ACC 0.40 baseline with the 0.30 trial."""
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
from analyze_control_response import analyse as control_analyse, recent_indices
from analyze_lateral_stop import summarize

FLIGHTS = [
    dict(flight="091448", vel_i=.40,
         bag=ROOT / "reports/roll_rate_0854_0914_20260921/091448.data.json",
         ulog=ROOT / "reports/roll_rate_0854_0914_20260921/23_52_43.ulg",
         bytes=4_758_798, expected_legs=3),
    dict(flight="095541", vel_i=.30, bag=OUT / "095541.data.json",
         ulog=OUT / "00_32_45.ulg", bytes=6_130_740, expected_legs=4),
]


def tracking_summary(columns, ranges):
    keep = np.logical_or.reduce([(columns["time_s"] >= lo) & (columns["time_s"] < hi)
                                 for lo, hi in ranges])
    result = {"samples": int(keep.sum())}
    for axis in ("roll", "pitch", "yaw"):
        angle_error = wrapped_error(columns[f"{axis}_actual_deg"][keep],
                                    columns[f"{axis}_target_deg"][keep])
        rate_error = (columns[f"{axis}_rate_actual_deg_s"][keep]
                      - columns[f"{axis}_rate_target_deg_s"][keep])
        result[axis] = dict(
            angle_error_rms_deg=float(np.sqrt(np.mean(angle_error ** 2))),
            rate_error_rms_deg_s=float(np.sqrt(np.mean(rate_error ** 2))))
    motors = np.array([columns[f"motor_{i}"][keep] for i in range(4)])
    result["motors"] = dict(peak=float(motors.max()),
                            fraction_above_095=float(np.mean(motors > .95)))
    return result


results = []
alignments = []
fig, axes = plt.subplots(4, 2, figsize=(14, 11), sharex="col", layout="constrained")
first_fig, first_axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True,
                                     layout="constrained")

for col, spec in enumerate(FLIGHTS):
    assert spec["ulog"].stat().st_size == spec["bytes"]
    bag = json.loads(spec["bag"].read_text())
    log = ULog(str(spec["ulog"]))
    data = {item.name: item.data for item in log.data_list if item.multi_id == 0}
    local = data["vehicle_local_position"]
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
    alignment = dict(flight=spec["flight"], ulog=str(spec["ulog"].relative_to(ROOT)),
                     bytes=spec["bytes"], offset=offset,
                     exact_local_position_matches=len(matches),
                     maximum_position_difference_m=float(position_difference),
                     ros_end_s=float(bag["local"][-1]["time"]),
                     onboard_end_s=float(log.last_timestamp / 1e6 + offset),
                     landing_event_s=float(landing),
                     dropouts=[dict(time_s=float(item.timestamp / 1e6 + offset),
                                    duration_ms=int(item.duration)) for item in log.dropouts])
    alignments.append(alignment)

    active = [row for row in bag["debug"]
              if row["state"] in ("LATERAL_MOVE", "LATERAL_HOLD")]
    result = summarize(dict(bag, debug=active))
    result.update(flight=spec["flight"], mpc_xy_vel_i_acc=spec["vel_i"],
                  alignment=alignment, events=bag["events"])
    assert result["config"]["brake_damping"] is False
    assert len(result["legs"]) == spec["expected_legs"]

    trajectory = data["trajectory_setpoint"]
    trajectory_time = trajectory["timestamp"] / 1e6 + offset
    yaw0 = bag["debug"][0]["yaw0"]
    origin = np.asarray(bag["debug"][0]["origin"])
    lateral_axis = np.array([-math.sin(yaw0), math.cos(yaw0)])
    forward_axis = np.array([math.cos(yaw0), math.sin(yaw0)])
    local_time = local["timestamp"] / 1e6 + offset
    position = np.column_stack([local["x"], local["y"]]) - origin
    velocity = np.column_stack([local["vx"], local["vy"]])
    right = position @ lateral_axis
    forward = position @ forward_axis
    lateral_velocity = velocity @ lateral_axis
    horizontal_speed = np.linalg.norm(velocity, axis=1)
    yaw_error = np.degrees(np.arctan2(np.sin(local["heading"] - yaw0),
                                      np.cos(local["heading"] - yaw0)))
    reference_position = (np.column_stack([trajectory["position[0]"],
                                           trajectory["position[1]"]]) - origin) @ lateral_axis
    reference_velocity = np.column_stack([trajectory["velocity[0]"],
                                          trajectory["velocity[1]"]]) @ lateral_axis

    # Both flights land on a hold timeout; use the last stationary setpoint
    # before landing for a clean final-hold boundary.
    final_leg = result["legs"][-1]
    stationary_final = ((trajectory_time < landing)
                        & (np.abs(reference_velocity) < 1e-6)
                        & (np.abs(reference_position - final_leg["target_m"]) < 1e-3))
    assert stationary_final.any()
    final_leg["hold_end_s"] = float(trajectory_time[stationary_final][-1])
    final_leg["hold_end_recovered_from_onboard"] = True
    assert 9.8 < final_leg["hold_end_s"] - final_leg["reference_stop_s"] < 10.2

    control, _ = control_analyse(spec["ulog"], offset)
    control_time = control["time_s"]
    lateral_demand = np.column_stack([control["vx_demand"], control["vy_demand"]]) @ lateral_axis
    lateral_accel_demand = np.column_stack([control["ax_demand"],
                                            control["ay_demand"]]) @ lateral_axis

    # Reconstruct the residual after known velocity P, D and trajectory
    # acceleration. This is mostly integrator during stationary holds, but is
    # not a directly logged integrator state.
    sp = data["vehicle_local_position_setpoint"]
    si, valid_sp = recent_indices(sp["timestamp"], local["timestamp"], 200_000)
    ti, valid_tr = recent_indices(trajectory["timestamp"], local["timestamp"], 200_000)
    internal_velocity = np.column_stack([sp["vx"][si], sp["vy"][si]]) @ lateral_axis
    internal_accel = np.column_stack([sp["acceleration[0]"][si],
                                      sp["acceleration[1]"][si]]) @ lateral_axis
    measured_accel = np.column_stack([local["ax"], local["ay"]]) @ lateral_axis
    trajectory_accel = np.column_stack([trajectory["acceleration[0]"][ti],
                                        trajectory["acceleration[1]"][ti]]) @ lateral_axis
    inferred_residual = (internal_accel
                         - 1.8 * (internal_velocity - lateral_velocity)
                         + .2 * measured_accel - trajectory_accel)

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
        control_first3 = (control_time >= stop) & (control_time < stop + 3)
        residual_first3 = first3 & valid_sp & valid_tr
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
        longest = eligible = 0.
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
            gate_failure_fraction={name: float(np.mean(~values[hold]))
                                   for name, values in gates.items()},
            longest_joint_dwell_s=float(longest), eligible_joint_dwell_s=float(eligible),
            first3_internal_velocity_demand_rms_m_s=float(
                np.sqrt(np.mean(lateral_demand[control_first3] ** 2))),
            first3_internal_acceleration_demand_rms_m_s2=float(
                np.sqrt(np.mean(lateral_accel_demand[control_first3] ** 2))),
            first3_inferred_velocity_residual_mean_m_s2=float(
                np.mean(inferred_residual[residual_first3])),
            first3_inferred_velocity_residual_rms_m_s2=float(
                np.sqrt(np.mean(inferred_residual[residual_first3] ** 2))),
            loaded_voltage_median_v=float(np.median(battery["voltage_v"][loaded])),
            loaded_current_median_a=float(np.median(battery["current_a"][loaded])))

    # Compare the same first three holds; the baseline never reached hold four.
    common_ranges = [(leg["reference_stop_s"], leg["reference_stop_s"] + 3)
                     for leg in result["legs"][:3]]
    attitude_columns, _ = attitude_analyse(spec["ulog"], offset,
                                            [(f"leg{i+1}", lo, hi)
                                             for i, (lo, hi) in enumerate(common_ranges)])
    result["tracking_first_three_holds_first3"] = tracking_summary(
        attitude_columns, common_ranges)
    result["parameters"] = {name: log.initial_parameters[name] for name in (
        "MPC_XY_P", "MPC_XY_VEL_P_ACC", "MPC_XY_VEL_I_ACC", "MPC_XY_VEL_D_ACC",
        "MC_ROLLRATE_P", "MC_ROLLRATE_I", "MC_ROLLRATE_D", "MC_ROLLRATE_K",
        "EKF2_EV_DELAY", "SENS_BOARD_X_OFF", "SENS_BOARD_Y_OFF", "SENS_BOARD_Z_OFF")}
    assert abs(result["parameters"]["MPC_XY_P"] - .95) < 1e-5
    assert abs(result["parameters"]["MPC_XY_VEL_I_ACC"] - spec["vel_i"]) < 1e-5
    assert abs(result["parameters"]["MC_ROLLRATE_P"] - .15) < 1e-5
    result["parameter_changes_during_flight"] = log.changed_parameters
    assert not result["parameter_changes_during_flight"]
    result["estimator_reset_counters"] = {
        key: np.unique(local[key]).tolist() for key in local if "reset_counter" in key}
    assert all(len(values) == 1 for values in result["estimator_reset_counters"].values())

    lo, hi = result["legs"][0]["start_s"] - .3, result["legs"][-1]["hold_end_s"]
    keep = (local_time >= lo) & (local_time <= hi)
    reference_keep = (trajectory_time >= lo) & (trajectory_time <= hi)
    axes[0, col].plot(local_time[keep], right[keep], label="Estimated")
    axes[0, col].plot(trajectory_time[reference_keep], reference_position[reference_keep],
                      label="Reference")
    axes[1, col].plot(local_time[keep], lateral_velocity[keep])
    axes[2, col].plot(local_time[keep], forward[keep])
    axes[3, col].plot(local_time[keep], yaw_error[keep])
    axes[0, col].set_title(f"{spec['flight']}: MPC_XY_VEL_I_ACC={spec['vel_i']:.2f}")
    for row, label in enumerate(("Lateral position (m)", "Lateral velocity (m/s)",
                                 "Forward position (m)", "Yaw error (deg)")):
        axes[row, col].set_ylabel(label)
        axes[row, col].grid(alpha=.25)
        axes[row, col].axhline(0, color="gray", lw=.6)
    axes[0, col].legend(fontsize=8)
    axes[3, col].set_xlabel("Bag-aligned seconds")

    first_stop = result["legs"][0]["reference_stop_s"]
    first = (local_time >= first_stop) & (local_time < first_stop + 3)
    relative = local_time[first] - first_stop
    label = f"I={spec['vel_i']:.2f}"
    first_axes[0].plot(relative, right[first] - .35, label=label)
    first_axes[1].plot(relative, lateral_velocity[first], label=label)
    first_axes[2].plot(relative, forward[first], label=label)
    first_axes[3].plot(relative, yaw_error[first], label=label)
    results.append(result)

baseline_params = ULog(str(FLIGHTS[0]["ulog"])).initial_parameters
trial_params = ULog(str(FLIGHTS[1]["ulog"])).initial_parameters
parameter_differences = {
    key: [baseline_params.get(key), trial_params.get(key)]
    for key in sorted(set(baseline_params) | set(trial_params))
    if baseline_params.get(key) != trial_params.get(key)}
allowed_differences = {"MPC_XY_VEL_I_ACC", "COM_FLIGHT_UUID", "LND_FLIGHT_T_LO",
                       "CAL_ACC0_XOFF", "CAL_ACC0_YOFF", "CAL_ACC0_ZOFF"}
assert set(parameter_differences) == allowed_differences

fig.suptitle("Horizontal velocity-I trial: MPC_XY_P=0.95, roll-rate P=0.150, damping OFF")
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
(OUT / "comparison.json").write_text(json.dumps(
    dict(flights=results, parameter_differences=parameter_differences), indent=2) + "\n")

for result in results:
    print(f"\n{result['flight']} I={result['mpc_xy_vel_i_acc']:.2f}")
    for leg in result["legs"]:
        print("leg", leg["leg"] + 1, "hold", round(leg["hold_duration_s"], 2),
              "first/eventual overshoot cm", round(100*leg["first_stop_endpoint_overshoot_m"], 1),
              round(100*leg["endpoint_overshoot_m"], 1),
              "span cm", round(100*leg["hold_span_m"], 1),
              "first3 vrms", round(leg["first3_lateral_velocity_rms_m_s"], 3),
              "residual rms", round(leg["first3_inferred_velocity_residual_rms_m_s2"], 3))
    print("tracking", json.dumps(result["tracking_first_three_holds_first3"], indent=2))
print("parameter differences", json.dumps(parameter_differences, indent=2))
