#!/usr/bin/env python3
"""Offline roll/pitch PID reconstruction for the flown PX4 rate controller.

Uses .venv-mavlink/bin/python3. Match torque and gyro by EXACT timestamp_sample,
then use a causal rate target no older than 10 ms. The residual is an inferred
integral contribution, not logged rate_ctrl_status. Its incremental agreement
with the unsaturated integral equation is a consistency check, not proof that
allocation or physical actuators are unsaturated. Yaw's extra torque filter is
deliberately excluded. No vehicle connection, simulation, or gain fitting.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--time-offset", type=float, required=True)
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument("--end", type=float, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    log = ULog(str(args.ulog))
    params = log.initial_parameters
    if params.get("MC_BAT_SCALE_EN", 0):
        raise ValueError("battery torque scaling requires a different reconstruction")
    if any(name.startswith("MC_") for _, name, _ in log.changed_parameters):
        raise ValueError("MC parameter changes require time-varying gains")
    torque = log.get_dataset("vehicle_torque_setpoint").data
    gyro = log.get_dataset("vehicle_angular_velocity").data
    target = log.get_dataset("vehicle_rates_setpoint").data
    gi = np.clip(np.searchsorted(gyro["timestamp_sample"], torque["timestamp_sample"]),
                 0, len(gyro["timestamp_sample"]) - 1)
    ri = np.searchsorted(target["timestamp"], torque["timestamp"], side="right") - 1
    age = (torque["timestamp"].astype(float) - target["timestamp"][np.maximum(ri, 0)]) / 1e6
    t = torque["timestamp"] / 1e6 + args.time_offset
    keep = ((gyro["timestamp_sample"][gi] == torque["timestamp_sample"]) &
            (ri >= 0) & (age >= 0) & (age <= .010) &
            (t >= args.start) & (t < args.end))
    if not keep.any():
        raise ValueError("no exact sample matches in the requested window")
    ri = np.maximum(ri, 0)
    columns = dict(time_s=t, target_age_s=age)
    summary = dict(ulog=str(args.ulog), time_offset_s=args.time_offset,
                   window_s=[args.start, args.end], exact_pairs=int(keep.sum()),
                   firmware=log.msg_info_dict.get("ver_sw", "unknown"), axes={})
    dt = np.diff(torque["timestamp_sample"].astype(float)) / 1e6
    pairs = keep[1:] & keep[:-1] & (dt > 0) & (dt < .012)
    for i, axis in enumerate(("roll", "pitch")):
        prefix = "MC_" + axis.upper() + "RATE_"
        k = params[prefix + "K"]
        measured = gyro[f"xyz[{i}]"][gi]
        requested = target[axis][ri]
        error = requested - measured
        p = k * params[prefix + "P"] * error
        d = -k * params[prefix + "D"] * gyro[f"xyz_derivative[{i}]"][gi]
        ff = params[prefix + "FF"] * requested
        output = torque[f"xyz[{i}]"]
        integral = output - p - d - ff
        for name, values in dict(target_deg_s=np.degrees(requested),
                                  actual_deg_s=np.degrees(measured), torque=output,
                                  p=p, d=d, ff=ff, inferred_i=integral).items():
            columns[axis + "_" + name] = values
        # Approximate intersample validation: PX4 integrates at control-loop
        # dt, whereas logging may omit intermediate iterations. Exclude gaps.
        prediction = (k * params[prefix + "I"] * error[:-1] *
                      np.maximum(0, 1 - (error[:-1] / np.radians(400))**2) * dt)
        residual = np.diff(integral) - prediction
        summary["axes"][axis] = dict(
            inferred_i_percentiles=np.percentile(integral[keep], [1, 50, 99]).tolist(),
            increment_error_abs_percentiles=np.percentile(
                np.abs(residual[pairs]), [50, 90, 99]).tolist() if pairs.any() else None)
    columns = {name: values[keep] for name, values in columns.items()}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.with_suffix(".csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(zip(*columns.values()))
    args.out.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
