"""Separate physical/VIO track shape, SLAM correction motion and planner vertices."""

from bisect import bisect_right
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).parent
FLIGHTS = ("111754", "112101")


def stamp(message):
    value = message["header"]["stamp"]
    return value["sec"] * 1_000_000_000 + value["nanosec"]


def point(message):
    pose = message["pose"]
    return np.array([pose["position"]["x"], pose["position"]["y"]], dtype=float)


def yaw(message):
    q = message["pose"]["orientation"]
    return math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]),
                      1 - 2 * (q["y"] ** 2 + q["z"] ** 2))


def transform(raw, correction):
    angle = yaw(correction)
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    return rotation @ raw + point(correction)


def path_length(points):
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def turn_angles(points):
    segments = np.diff(points, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    keep = lengths > 0.002
    segments = segments[keep]
    if len(segments) < 2:
        return np.array([])
    headings = np.unwrap(np.arctan2(segments[:, 1], segments[:, 0]))
    return np.abs(np.diff(headings))


def phase(text):
    for name in ("GOAL_REACHED", "COMMAND_HOLD", "YAW_ALIGN", "DEGRADED", "HOLD", "ROUTE"):
        if text.startswith(name):
            return name
    return None


COLORS = {
    "ROUTE": "#159447",
    "DEGRADED": "#e19b22",
    "HOLD": "#b13b91",
    "YAW_ALIGN": "#326dc0",
    "COMMAND_HOLD": "#d33333",
    "GOAL_REACHED": "#53b873",
}


summary = {}
fig, axes = plt.subplots(2, 2, figsize=(14, 11), layout="constrained")
for row_index, suffix in enumerate(FLIGHTS):
    data = json.loads((HERE / f"{suffix}.json").read_text(encoding="utf-8"))
    logs = [(time, message) for time, message in data["/rosout"]
            if message.get("name") == "cpp_flight_adapter"]
    route_start = next(time for time, message in logs if message.get("msg") == "-> ROUTE")
    land = next(time for time, message in logs if message.get("msg") == "-> LAND")

    corrected = {stamp(message): (time, message) for time, message in data["/rtabmap/body_pose"]}
    raw = {stamp(message): (time, message) for time, message in data["/rtabmap/body_vio_pose"]}
    corrections = {stamp(message): (time, message) for time, message in data["/rtabmap/odom_correction"]}
    common = sorted(set(corrected) & set(raw) & set(corrections))
    common = [key for key in common if route_start <= corrected[key][0] <= land]
    times = np.array([corrected[key][0] for key in common])
    corrected_points = np.array([point(corrected[key][1]) for key in common])
    raw_points = np.array([point(raw[key][1]) for key in common])
    final_correction = corrections[common[-1]][1]
    fixed_points = np.array([transform(value, final_correction) for value in raw_points])
    moving_correction = corrected_points - fixed_points
    correction_steps = np.linalg.norm(np.diff(moving_correction, axis=0), axis=1)

    statuses = [(route_start, "ROUTE")]
    for time, message in data["/planner/flight/status"]:
        if route_start <= time <= land:
            name = phase(message["data"])
            if name and name != statuses[-1][1]:
                statuses.append((time, name))
    intervals = [(start, statuses[index + 1][0] if index + 1 < len(statuses) else land, name)
                 for index, (start, name) in enumerate(statuses)]

    paths = [(time, message) for time, message in data["/planner/path"]
             if route_start <= time <= land and len(message["poses"]) >= 2]
    unique_paths = {}
    path_stats = []
    for time, message in paths:
        points = np.array([[pose["pose"]["position"]["x"], pose["pose"]["position"]["y"]]
                           for pose in message["poses"]])
        fingerprint = tuple(np.round(points.flatten(), 4))
        if fingerprint in unique_paths:
            continue
        unique_paths[fingerprint] = (time, points)
        turns = turn_angles(points)
        path_stats.append({
            "time": time,
            "points": len(points),
            "length": path_length(points),
            "turn_count": len(turns),
            "maximum_turn_deg": float(np.degrees(turns).max()) if len(turns) else 0.0,
            "turns_over_20_deg": int(np.sum(np.degrees(turns) > 20.0)),
        })

    corrected_turns = turn_angles(corrected_points)
    fixed_turns = turn_angles(fixed_points)
    summary[suffix] = {
        "route_window": [route_start, land],
        "paired_body_samples": len(common),
        "corrected_track_length": path_length(corrected_points),
        "fixed_correction_raw_vio_track_length": path_length(fixed_points),
        "extra_corrected_track_length": path_length(corrected_points) - path_length(fixed_points),
        "correction_increment_p95": float(np.percentile(correction_steps, 95)),
        "correction_increment_max": float(correction_steps.max()),
        "corrected_step_turn_p95_deg": float(np.degrees(np.percentile(corrected_turns, 95))),
        "fixed_raw_step_turn_p95_deg": float(np.degrees(np.percentile(fixed_turns, 95))),
        "unique_planner_paths": len(unique_paths),
        "path_statistics": path_stats,
        "phase_intervals": intervals,
    }

    ax = axes[row_index, 0]
    for start, end, name in intervals:
        keep = (times >= start) & (times <= end)
        if keep.any():
            ax.plot(corrected_points[keep, 0], corrected_points[keep, 1],
                    color=COLORS[name], lw=2.2, label=name)
    ax.plot(fixed_points[:, 0], fixed_points[:, 1], color="#653a9c", lw=1.1,
            alpha=.9, label="Raw VIO through final fixed correction")
    if unique_paths:
        _, last_path = list(unique_paths.values())[-1]
        ax.plot(last_path[:, 0], last_path[:, 1], "--", color="#2674d8", lw=1.4,
                label="Last planner polyline")
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), fontsize=7)
    ax.set_aspect("equal")
    ax.grid(alpha=.2)
    ax.set_title(f"{suffix}: corrected track vs fixed-frame raw VIO")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")

    ax = axes[row_index, 1]
    ax.plot(times, np.linalg.norm(moving_correction, axis=1), label="Correction-relative displacement")
    ax.plot(times[1:], correction_steps, label="Correction contribution per pose sample")
    for start, end, name in intervals:
        if name != "ROUTE":
            ax.axvspan(start, end, color=COLORS[name], alpha=.10)
    ax.set_title(f"{suffix}: map-correction contribution")
    ax.set_xlabel("bag seconds")
    ax.set_ylabel("metres")
    ax.grid(alpha=.2)
    ax.legend(fontsize=7)

fig.suptitle("Spatial jaggedness audit: state recovery versus SLAM correction")
fig.savefig(HERE / "spatial_jaggedness.png", dpi=170)
(HERE / "spatial_jaggedness.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
print(json.dumps(summary, indent=2))
