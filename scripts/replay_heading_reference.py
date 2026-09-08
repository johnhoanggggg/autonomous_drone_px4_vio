#!/usr/bin/env python3
"""Compare heading policies on recorded command geometry; never publishes ROS.

Uses the ROS environment and the production C++ replay binary. This is a
recorded-state shadow, NOT closed-loop flight or full adapter replay. Poses,
commands and route acceptance are fixed historical inputs. Receipt ordering
and projection approximate unrecorded internal progress/join state. Recorded
YAW_ALIGN/DEGRADED status freezes route replacement, as in the flown adapter.
Use a window without fault/clearance rejection; those are explicitly rejected.
"""
import argparse
from bisect import bisect_right
import csv
import json
import math
from pathlib import Path
import re
import subprocess

from analyze_yaw_tracking import TOPICS, read, yaw_of
from replay_command_heading import CONFIG

PATH = "/planner/path"
POSE = "/rtabmap/body_pose"
CORRECTION = "/rtabmap/odom_correction"
LOCAL = "/fmu/out/vehicle_local_position_v1"
SETPOINT = "/fmu/in/trajectory_setpoint"
STATUS = "/planner/flight/status"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--start", type=float, default=12.8)
    parser.add_argument("--end", type=float, default=14.65)
    root = Path(__file__).resolve().parents[1]
    parser.add_argument("--binary", type=Path,
                        default=root / "ros2_ws/build/px4_vio_bridge/planner_flight_replay")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    names = tuple(dict.fromkeys((*TOPICS, PATH, CORRECTION,
                                "/planner/flight/command_speed", "/planner/flight/joining")))
    data, _ = read(args.bag, names)
    t0 = min(data[n][0][0] for n in TOPICS if data[n])
    stamps = {n: [t for t, _ in data[n]] for n in names}

    def latest(name, stamp, max_age=1.0):
        i = bisect_right(stamps[name], stamp) - 1
        if i < 0 or stamp - stamps[name][i] > max_age * 1e9:
            raise ValueError(f"missing/stale {name} at {(stamp-t0)/1e9:.3f}")
        return data[name][i]

    accepted, records, steps = None, [], []
    initial = None
    for stamp, sp in data[SETPOINT]:
        time = (stamp - t0) / 1e9
        if not args.start <= time <= args.end:
            continue
        _, status = latest(STATUS, stamp)
        phase = status.data.split()[0]
        if phase not in ("ROUTE", "YAW_ALIGN", "DEGRADED", "CORNER_HOLD"):
            raise ValueError(f"unsupported phase {phase} in this focused shadow window")
        target_match = re.search(r"yaw_target=([-\d.]+)deg", status.data)
        if not target_match:
            raise ValueError("recorded status has no heading target")
        recorded_target = float(target_match[1])
        if initial is None:
            initial = math.radians(recorded_target)
        path_stamp, path = latest(PATH, stamp, 3.0)
        if accepted is None or phase not in ("YAW_ALIGN", "DEGRADED"):
            accepted = (path_stamp, path)
        pose_stamp, pose = latest(POSE, stamp, 0.3)
        local_stamp, local = latest(LOCAL, stamp, 0.1)
        _, correction = latest(CORRECTION, stamp)
        correction_yaw = yaw_of(correction)
        # Adapter: NED command = PX4 position + R(map->vio)*map displacement,
        # with ENU/NED x/y swapped. Invert it using recent recorded inputs.
        vx, vy = sp.position[1] - local.y, sp.position[0] - local.x
        c, s = math.cos(correction_yaw), math.sin(correction_yaw)
        command = [float(pose.pose.position.x + c*vx - s*vy),
                   float(pose.pose.position.y + s*vx + c*vy)]
        _, speed = latest("/planner/flight/command_speed", stamp, 0.1)
        _, joining = latest("/planner/flight/joining", stamp, 0.1)
        steps.append(dict(op="heading_sample", command=command,
                          points=[[p.pose.position.x, p.pose.position.y]
                                  for p in accepted[1].poses],
                          measured_heading=float(local.heading), correction_yaw=correction_yaw,
                          command_speed=float(speed.data), dt=0.05))
        records.append(dict(time_s=time, phase=phase, recorded_target_deg=recorded_target,
                            published_yaw_deg=math.degrees(sp.yaw),
                            measured_yaw_deg=math.degrees(local.heading),
                            command_speed=speed.data, joining=joining.data,
                            incoming_path_s=(path_stamp-t0)/1e9,
                            accepted_path_s=(accepted[0]-t0)/1e9,
                            pose_age_s=(stamp-pose_stamp)/1e9,
                            local_age_s=(stamp-local_stamp)/1e9))
    if not steps:
        raise ValueError("no samples in requested window")
    scenario = dict(limiter=CONFIG, initial_yaw=initial, steps=steps)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.with_suffix(".inputs.json").write_text(json.dumps(scenario, indent=2) + "\n")
    for label, lookahead, rate in (("old", 0, 0), ("new", 0.35, math.radians(20))):
        scenario.update(heading_lookahead=lookahead, heading_rate=rate)
        run = subprocess.run([str(args.binary)], input=json.dumps(scenario),
                             text=True, capture_output=True, check=True, timeout=60)
        results = json.loads(run.stdout)
        for row, result in zip(records, results, strict=True):
            if not result["ok"]:
                raise ValueError(result)
            for key in ("preview", "target", "error"):
                row[f"{label}_{key}_deg"] = math.degrees(result[key])
            for key in ("yaw_holding", "yaw_speed_cap", "progress", "projection_error"):
                row[f"{label}_{key}"] = result[key]
    with args.out.with_suffix(".csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    summary = dict(scope=__doc__, bag=args.bag, window=[args.start, args.end], samples=len(records))
    for label in ("old", "new"):
        targets = [r[f"{label}_target_deg"] for r in records]
        jumps = [abs((b-a+180) % 360-180) for a, b in zip(targets, targets[1:])]
        summary[label] = dict(max_target_step_deg=max(jumps),
                              target_range_deg=[min(targets), max(targets)],
                              shadow_yaw_hold_samples=sum(r[f"{label}_yaw_holding"] for r in records))
    summary["max_pose_age_s"] = max(r["pose_age_s"] for r in records)
    summary["max_local_age_s"] = max(r["local_age_s"] for r in records)
    args.out.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
