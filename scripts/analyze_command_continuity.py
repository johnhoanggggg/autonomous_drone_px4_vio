#!/usr/bin/env python3
"""Audit published velocity continuity in a planner bag, without publishing ROS.

Run after sourcing ROS/workspace. Position differences include the adapter's
map-to-PX4 rebasing; they are not measurements of vehicle speed. Acceleration
below is the velocity-reference difference over ROS receipt time, not the
separately clipped acceleration feedforward or measured aircraft acceleration.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from analyze_yaw_tracking import TOPICS, read


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument("--end", type=float, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    topic = "/fmu/in/trajectory_setpoint"
    data, _ = read(args.bag, list(dict.fromkeys([*TOPICS, topic])))
    t0 = min(data[n][0][0] for n in TOPICS if data[n])
    rows = []
    entries = data[topic]
    for (ta, a), (tb, b) in zip(entries, entries[1:]):
        time, dt = (tb - t0) / 1e9, (tb - ta) / 1e9
        if not args.start <= time < args.end or not 0.035 <= dt <= 0.070:
            continue
        if not all(math.isfinite(v) for v in [*a.velocity[:2], *b.velocity[:2]]):
            continue  # position-only holds have no velocity FF
        dv = [float(b.velocity[i] - a.velocity[i]) for i in range(2)]
        rows.append(dict(time_s=time, dt_s=dt,
                         old_vx=float(a.velocity[0]), old_vy=float(a.velocity[1]),
                         vx=float(b.velocity[0]), vy=float(b.velocity[1]),
                         velocity_step_m_s=math.hypot(*dv),
                         reference_acceleration_m_s2=math.hypot(*dv) / dt))
    if not rows:
        raise ValueError("no consecutive finite route commands in the window")
    summary = dict(bag=args.bag, window_s=[args.start, args.end], samples=len(rows),
                   acceleration_percentiles_m_s2=dict(zip(
                       ["p50", "p90", "p99", "max"],
                       np.percentile([r["reference_acceleration_m_s2"] for r in rows],
                                     [50, 90, 99, 100]).tolist())),
                   steps_above_04=[r for r in rows if r["reference_acceleration_m_s2"] > .4])
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.with_suffix(".csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    args.out.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
