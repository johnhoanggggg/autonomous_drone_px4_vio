#!/usr/bin/env python3
"""Pair actual attitude/rates with PX4 targets on the onboard clock.

Uses .venv-mavlink/bin/python3. --window NAME:START:END can be repeated.
Times are PX4 boot seconds plus --time-offset (default: since ULog start).
Writes paired CSV, per-window JSON, and an optional plot. No live connections.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from pyulog import ULog

from analyze_control_response import recent_indices


def euler_degrees(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q, axis=0)
    q = q / np.where(norm > 1e-6, norm, np.nan)
    w, x, y, z = q
    return np.degrees(np.array([
        np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)),
        np.arcsin(np.clip(2*(w*y - z*x), -1, 1)),
        np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))]))


def wrapped_error(actual, target):
    return (actual - target + 180) % 360 - 180


def analyse(path, offset, windows):
    log = ULog(str(path))
    data = {item.name: item.data for item in log.data_list if item.multi_id == 0}
    gyro = data["vehicle_angular_velocity"]
    stamps = gyro["timestamp"]
    offset = -log.start_timestamp / 1e6 if offset is None else offset
    valid = np.ones(len(stamps), dtype=bool)

    def paired(topic, fields):
        nonlocal valid
        source = data[topic]
        idx, usable = recent_indices(source["timestamp"], stamps, 50_000)
        valid &= usable
        return np.array([source[key][idx] for key in fields])

    actual = euler_degrees(paired("vehicle_attitude", [f"q[{i}]" for i in range(4)]))
    target = euler_degrees(paired("vehicle_attitude_setpoint", [f"q_d[{i}]" for i in range(4)]))
    rate_target = np.degrees(paired("vehicle_rates_setpoint", ["roll", "pitch", "yaw"]))
    rates = np.degrees(np.array([gyro[f"xyz[{i}]"] for i in range(3)]))
    torque = paired("vehicle_torque_setpoint", [f"xyz[{i}]" for i in range(3)])
    motors = paired("actuator_motors", [f"control[{i}]" for i in range(4)])
    columns = {"time_s": stamps / 1e6 + offset}
    for i, axis in enumerate(("roll", "pitch", "yaw")):
        columns.update({axis + "_target_deg": target[i], axis + "_actual_deg": actual[i],
                        axis + "_rate_target_deg_s": rate_target[i],
                        axis + "_rate_actual_deg_s": rates[i], axis + "_torque": torque[i]})
    for i in range(4):
        columns[f"motor_{i}"] = motors[i]
    for values in columns.values():
        valid &= np.isfinite(values)
    columns = {name: values[valid] for name, values in columns.items()}
    if not valid.any():
        raise ValueError("no fresh, finite control pairs")
    summary = {"ulog": str(path), "time_offset_s": offset,
               "dropouts": len(log.dropouts), "windows": {}}
    for name, lo, hi in windows:
        keep = (columns["time_s"] >= lo) & (columns["time_s"] < hi)
        if not keep.any():
            raise ValueError(f"no samples in {name}")
        result = {"window_s": [lo, hi], "samples": int(keep.sum())}
        for axis in ("roll", "pitch", "yaw"):
            error = wrapped_error(columns[axis + "_actual_deg"][keep],
                                  columns[axis + "_target_deg"][keep])
            demand = columns[axis + "_rate_target_deg_s"][keep]
            achieved = columns[axis + "_rate_actual_deg_s"][keep]
            result[axis] = {
                "angle_error_rms_deg": float(np.sqrt(np.mean(error**2))),
                "angle_error_max_abs_deg": float(np.max(np.abs(error))),
                "rate_error_rms_deg_s": float(np.sqrt(np.mean((achieved-demand)**2))),
                "rate_demand_mean_deg_s": float(demand.mean()),
                "rate_actual_mean_deg_s": float(achieved.mean()),
                "torque_mean": float(columns[axis + "_torque"][keep].mean())}
        motor = np.array([columns[f"motor_{i}"][keep] for i in range(4)])
        result["motors"] = {"mean": motor.mean(axis=1).tolist(), "peak": float(motor.max()),
                            "fraction_above_095": float(np.mean(motor > .95))}
        summary["windows"][name] = result
    return columns, summary


def plot(columns, windows, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(13, 8), sharex=True, layout="constrained")
    lo, hi = min(x[1] for x in windows), max(x[2] for x in windows)
    keep = (columns["time_s"] >= lo) & (columns["time_s"] <= hi)
    t = columns["time_s"][keep]
    for row, axis in enumerate(("roll", "pitch", "yaw")):
        for col, suffix in enumerate(("_deg", "_deg_s")):
            root = axis if col == 0 else axis + "_rate"
            ax = axes[row, col]
            ax.plot(t, columns[root + "_target" + suffix][keep], label="PX4 target", lw=1)
            ax.plot(t, columns[root + "_actual" + suffix][keep], label="Actual", lw=1)
            ax.set_ylabel(axis.capitalize() + (" (deg)" if col == 0 else " rate (deg/s)"))
            ax.grid(alpha=.2)
            ax.legend(fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("Bag-aligned seconds")
    fig.suptitle(f"{output.stem}: attitude and angular-rate tracking")
    fig.savefig(output, dpi=150)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--time-offset", type=float)
    parser.add_argument("--window", action="append", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    windows = [(name, float(lo), float(hi)) for name, lo, hi in
               (text.split(":") for text in args.window)]
    columns, summary = analyse(args.ulog, args.time_offset, windows)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.with_suffix(".csv").open("w") as file:
        writer = csv.writer(file)
        writer.writerow(columns)
        writer.writerows(zip(*columns.values()))
    args.out.with_suffix(".json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    if args.plot:
        plot(columns, windows, args.out.with_suffix(".png"))
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
