#!/usr/bin/env python3
"""Compare PX4's internal demand with its response, using one onboard clock.

Run with .venv-mavlink/bin/python3 (pyulog and numpy). Example:
  scripts/analyze_control_response.py flight.ulg --out reports/control

Writes CSV and JSON. Times default to seconds since ULog start; --time-offset
adds seconds to PX4 boot time instead, for an explicitly measured bag alignment.
Tilt-equivalent acceleration assumes near-hover vertical thrust. It is neither
the measured acceleration nor proof of an actuator fault. No live connections.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def recent_indices(source, target, max_age_us):
    """Causal matching: return indices and a mask; never extend a stale sample."""
    indices = np.searchsorted(source, target, side="right") - 1
    clipped = np.maximum(indices, 0)
    valid = (indices >= 0) & ((target.astype(float) - source[clipped]) <= max_age_us)
    return clipped, valid


def analyse(path, offset=None, start=None, end=None):
    log = ULog(str(path))
    data = {item.name: item.data for item in log.data_list if item.multi_id == 0}
    lp = data["vehicle_local_position"]
    sp = data["vehicle_local_position_setpoint"]
    attitude = data["vehicle_attitude"]
    stamp = lp["timestamp"]
    time = stamp / 1e6 + (offset if offset is not None else -log.start_timestamp / 1e6)
    si, valid_s = recent_indices(sp["timestamp"], stamp, 200_000)
    ai, valid_a = recent_indices(attitude["timestamp"], stamp, 50_000)
    keep = valid_s & valid_a & lp["xy_valid"].astype(bool) & lp["v_xy_valid"].astype(bool)
    if start is not None:
        keep &= time >= start
    if end is not None:
        keep &= time <= end
    q = np.array([attitude[f"q[{i}]"][ai] for i in range(4)])
    norm = np.linalg.norm(q, axis=0)
    keep &= np.isfinite(norm) & (norm > 1e-6)
    q /= np.maximum(norm, 1e-6)
    w, x, y, z = q
    bz = 1 - 2 * (x*x + y*y)
    keep &= bz > 0.5  # Near-hover approximation excludes large tilts/inversion.
    safe_bz = np.maximum(bz, 0.5)
    columns = {
        "time_s": time,
        "vx_demand": sp["vx"][si], "vy_demand": sp["vy"][si],
        "vx_actual": lp["vx"], "vy_actual": lp["vy"],
        "ax_demand": sp["acceleration[0]"][si],
        "ay_demand": sp["acceleration[1]"][si],
        "ax_tilt_equivalent": -9.81 * 2 * (x*z + w*y) / safe_bz,
        "ay_tilt_equivalent": -9.81 * 2 * (y*z - w*x) / safe_bz,
        "ax_estimated": lp["ax"], "ay_estimated": lp["ay"],
        "heading_deg": np.degrees(lp["heading"]),
        "demand_age_s": (stamp.astype(float) - sp["timestamp"][si]) / 1e6,
    }
    for values in columns.values():
        keep &= np.isfinite(values)
    columns = {name: values[keep] for name, values in columns.items()}
    if not keep.any():
        raise ValueError("no valid overlapping samples in the requested window")
    demand = np.hypot(columns["vx_demand"], columns["vy_demand"])
    actual = np.hypot(columns["vx_actual"], columns["vy_actual"])
    error = np.hypot(columns["vx_demand"] - columns["vx_actual"],
                     columns["vy_demand"] - columns["vy_actual"])
    peak = int(np.argmax(actual))
    summary = {
        "ulog": str(path), "samples": int(keep.sum()),
        "time_offset_s": offset, "window_s": [float(columns["time_s"][0]),
                                                 float(columns["time_s"][-1])],
        "peak_speed_time_s": float(columns["time_s"][peak]),
        "peak_actual_speed_m_s": float(actual[peak]),
        "demand_speed_at_actual_peak_m_s": float(demand[peak]),
        "peak_demand_speed_m_s": float(demand.max()),
        "p90_velocity_error_m_s": float(np.percentile(error, 90)),
        "ulog_dropouts": len(log.dropouts),
        "parameters": {k: v for k, v in log.initial_parameters.items()
                       if k.startswith(("MPC_XY", "MC_ROLL", "MC_PITCH", "EKF2_EV_DELAY"))},
        "missing_control_topics": sorted({"vehicle_attitude_setpoint",
            "vehicle_rates_setpoint", "vehicle_angular_velocity"} - data.keys()),
    }
    return columns, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--time-offset", type=float)
    parser.add_argument("--start", type=float)
    parser.add_argument("--end", type=float)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    columns, summary = analyse(args.ulog, args.time_offset, args.start, args.end)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.with_suffix(".csv").open("w") as file:
        writer = csv.writer(file)
        writer.writerow(columns)
        writer.writerows(zip(*columns.values()))
    args.out.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
