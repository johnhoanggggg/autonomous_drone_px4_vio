"""Compare the new opt-in damping flights with the proven global baseline."""

from bisect import bisect_right
from collections import Counter
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).parent
ROOT = HERE.parents[1]
BASELINE = ROOT / "reports/migration_tunnel_20260921_0726/072654.json"
FLIGHTS = {
    "072654 baseline OFF": BASELINE,
    "111754 damping ON": HERE / "111754.json",
    "112101 damping ON": HERE / "112101.json",
}

LP = "/fmu/out/vehicle_local_position_v1"
SP = "/fmu/in/trajectory_setpoint"


def norm2(values):
    return math.hypot(float(values[0]), float(values[1]))


def phase(text):
    for name in ("GOAL_REACHED", "COMMAND_HOLD", "YAW_ALIGN", "DEGRADED", "HOLD", "ROUTE"):
        if text.startswith(name):
            return name
    return None


class Flight:
    def __init__(self, label, path):
        self.label = label
        self.data = json.loads(path.read_text(encoding="utf-8"))
        self.times = {topic: [row[0] for row in rows] for topic, rows in self.data.items()}
        logs = [row for row in self.data.get("/rosout", []) if row[1].get("name") == "cpp_flight_adapter"]
        self.route_start = next(t for t, msg in logs if msg.get("msg") == "-> ROUTE")
        self.land = next((t for t, msg in logs if msg.get("msg") == "-> LAND"), None)
        self.goal_start = next(
            (t for t, msg in self.data.get("/planner/flight/status", [])
             if msg["data"].startswith("GOAL_REACHED")), None)
        self.end = self.goal_start or self.land or self.data[LP][-1][0]
        self.status_intervals = self._status_intervals()

    def at(self, topic, time):
        rows = self.data.get(topic, [])
        index = bisect_right(self.times.get(topic, []), time) - 1
        return None if index < 0 else rows[index][1]

    def phase_at(self, time):
        for start, end, name in self.status_intervals:
            if start <= time < end:
                return name
        return "OUTSIDE"

    def _status_intervals(self):
        events = [(self.route_start, "ROUTE")]
        final = self.land or self.data[LP][-1][0]
        for time, msg in self.data.get("/planner/flight/status", []):
            if time < self.route_start or time > final:
                continue
            name = phase(msg["data"])
            if name and name != events[-1][1]:
                events.append((time, name))
        return [
            (start, events[index + 1][0] if index + 1 < len(events) else final, name)
            for index, (start, name) in enumerate(events)
        ]

    def local_samples(self, start=None, end=None, phase_name=None):
        start = self.route_start if start is None else start
        end = self.end if end is None else end
        return [
            (time, msg) for time, msg in self.data[LP]
            if start <= time <= end and (phase_name is None or self.phase_at(time) == phase_name)
        ]

    def position_errors(self, phase_name="ROUTE"):
        values = []
        for time, msg in self.data[SP]:
            if not self.route_start <= time <= self.end or self.phase_at(time) != phase_name:
                continue
            position = msg["position"]
            if not math.isfinite(position[0]) or not math.isfinite(position[1]):
                continue
            local = self.at(LP, time)
            values.append(norm2((position[0] - local["x"], position[1] - local["y"])))
        return values

    def cross_track(self):
        return [
            abs(msg["data"]) for time, msg in self.data.get("/planner/follower/cross_track", [])
            if self.route_start <= time <= self.end and self.phase_at(time) == "ROUTE"
        ]

    def phase_summary(self):
        durations = Counter()
        entries = Counter()
        last = None
        final = self.goal_start or self.land or self.data[LP][-1][0]
        for start, end, name in self.status_intervals:
            if start >= final:
                break
            durations[name] += max(0.0, min(end, final) - start)
            if name != last:
                entries[name] += 1
            last = name
        return dict(durations), dict(entries)

    def interval_motion(self, start, end):
        rows = self.local_samples(start, end)
        if len(rows) < 2:
            return None
        points = np.array([[msg["x"], msg["y"]] for _, msg in rows])
        speeds = np.array([math.hypot(msg["vx"], msg["vy"]) for _, msg in rows])
        return {
            "duration": end - start,
            "net_displacement": float(np.linalg.norm(points[-1] - points[0])),
            "path_length": float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
            "max_radius_from_start": float(np.linalg.norm(points - points[0], axis=1).max()),
            "speed_peak": float(speeds.max()),
            "speed_rms": float(np.sqrt(np.mean(speeds ** 2))),
        }

    def map_interval_motion(self, start, end):
        rows = [(time, msg) for time, msg in self.data.get("/rtabmap/body_pose", [])
                if start <= time <= end]
        if len(rows) < 2:
            return None
        points = np.array([[msg["pose"]["position"]["x"], msg["pose"]["position"]["y"]]
                           for _, msg in rows])
        return {
            "net_displacement": float(np.linalg.norm(points[-1] - points[0])),
            "path_length": float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
            "max_radius_from_start": float(np.linalg.norm(points - points[0], axis=1).max()),
        }

    def damping_summary(self):
        rows = []
        for time, msg in self.data.get("/planner/flight/damping_debug", []):
            if self.route_start <= time <= (self.land or self.data[LP][-1][0]):
                rows.append((time, json.loads(msg["data"])))
        if not rows:
            return None
        reasons = Counter(row["reason"] for _, row in rows)
        active = [(time, row) for time, row in rows if row["reason"] in ("braking_or_hold", "phase_release")]
        braking = [(time, row) for time, row in rows if row["reason"] == "braking_or_hold"]
        applied = np.array([norm2(row["applied_correction_ned"]) for _, row in active])
        filtered = np.array([norm2(row["filtered_correction_ned"]) for _, row in active])
        velocity_errors = np.array([
            norm2(np.asarray(row["reference_velocity_ned"]) - np.asarray(row["measured_velocity_ned"]))
            for _, row in braking if row["measured_velocity_ned"] is not None
        ])
        opposing = []
        for _, row in braking:
            if row["measured_velocity_ned"] is None:
                continue
            opposing.append(
                np.dot(row["applied_correction_ned"], row["measured_velocity_ned"]) < 0.0
            )
        gaps = np.diff([time for time, _ in rows])
        tick = float(np.median(gaps[gaps < .12])) if len(gaps) else .05
        return {
            "reasons": dict(reasons),
            "active_seconds_estimate": len(active) * tick,
            "braking_seconds_estimate": len(braking) * tick,
            "applied_correction_peak": float(applied.max()),
            "applied_correction_p95": float(np.percentile(applied, 95)),
            "filtered_correction_peak": float(filtered.max()),
            "filtered_at_0p10_cap_fraction": float(np.mean(filtered >= .099)),
            "braking_velocity_error_p95": float(np.percentile(velocity_errors, 95)),
            "braking_correction_opposed_motion_fraction": float(np.mean(opposing)),
        }

    def metrics(self):
        route_rows = self.local_samples(phase_name="ROUTE")
        speeds = np.array([math.hypot(msg["vx"], msg["vy"]) for _, msg in route_rows])
        points = np.array([[msg["x"], msg["y"]] for _, msg in self.local_samples()])
        errors = np.array(self.position_errors())
        cross = self.cross_track()
        durations, entries = self.phase_summary()
        config_rows = self.data.get("/planner/flight/config", [])
        config = json.loads(config_rows[-1][1]["data"]) if config_rows else {}
        goal_hold = self.interval_motion(self.goal_start, self.land) if self.goal_start and self.land else None
        goal_hold_map = self.map_interval_motion(self.goal_start, self.land) if self.goal_start and self.land else None
        hold_motions = [
            self.interval_motion(start, end)
            for start, end, name in self.status_intervals
            if name == "HOLD" and start < self.end
        ]
        hold_motions = [motion for motion in hold_motions if motion]
        return {
            "config": {
                "brake_damping": config.get("brake_damping", False),
                "damping_gain": config.get("damping_gain"),
                "command_speed": config.get("command_speed"),
                "command_acceleration": config.get("command_acceleration"),
            },
            "route_start": self.route_start,
            "goal_start": self.goal_start,
            "land": self.land,
            "completed_goal": self.goal_start is not None,
            "route_to_goal_or_land_seconds": self.end - self.route_start,
            "route_measured_speed_peak": float(speeds.max()),
            "route_measured_speed_p95": float(np.percentile(speeds, 95)),
            "route_position_error_p95": float(np.percentile(errors, 95)),
            "route_position_error_max": float(errors.max()),
            "route_cross_track_max": max(cross) if cross else None,
            "window_net_displacement": float(np.linalg.norm(points[-1] - points[0])),
            "window_path_length": float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
            "phase_seconds": durations,
            "phase_entries": entries,
            "fault_hold_motion": hold_motions,
            "goal_hold_motion": goal_hold,
            "goal_hold_map_motion": goal_hold_map,
            "damping": self.damping_summary(),
            "throttled_values": sorted({msg["data"] for _, msg in self.data.get("/perf/throttled", [])}),
        }


flights = [Flight(label, path) for label, path in FLIGHTS.items()]
metrics = {flight.label: flight.metrics() for flight in flights}
metrics["mapped_clearance_min"] = {
    "072654 baseline OFF": 0.21407695373656802,
    "111754 damping ON": 0.266,
    "112101 damping ON": 0.210,
}
(HERE / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")


fig, axes = plt.subplots(4, 1, figsize=(12, 13), layout="constrained", sharex=False)
colours = ("#555555", "#16834a", "#c84737")
for flight, colour in zip(flights, colours):
    local = flight.local_samples(flight.route_start, flight.land or flight.data[LP][-1][0])
    axes[0].plot(
        [time - flight.route_start for time, _ in local],
        [math.hypot(msg["vx"], msg["vy"]) for _, msg in local],
        color=colour, label=flight.label,
    )
    error_times, errors = [], []
    for time, msg in flight.data[SP]:
        if not flight.route_start <= time <= flight.end or not math.isfinite(msg["position"][0]):
            continue
        local_msg = flight.at(LP, time)
        error_times.append(time - flight.route_start)
        errors.append(norm2((msg["position"][0] - local_msg["x"], msg["position"][1] - local_msg["y"])))
    axes[1].plot(error_times, errors, color=colour, label=flight.label)
    cross = [(time - flight.route_start, abs(msg["data"]))
             for time, msg in flight.data.get("/planner/follower/cross_track", [])
             if flight.route_start <= time <= flight.end]
    if cross:
        axes[2].plot([row[0] for row in cross], [row[1] for row in cross], color=colour, label=flight.label)
    damping = []
    for time, msg in flight.data.get("/planner/flight/damping_debug", []):
        if flight.route_start <= time <= (flight.land or flight.data[LP][-1][0]):
            row = json.loads(msg["data"])
            damping.append((time - flight.route_start, norm2(row["applied_correction_ned"])))
    if damping:
        axes[3].plot([row[0] for row in damping], [row[1] for row in damping], color=colour, label=flight.label)
for flight, colour in zip(flights, colours):
    if flight.goal_start:
        for axis in axes:
            axis.axvline(flight.goal_start - flight.route_start, color=colour, linestyle="--", alpha=.55)
axes[0].set_ylabel("Measured XY speed (m/s)")
axes[1].set_ylabel("Position-target error (m)")
axes[2].set_ylabel("Follower cross-track (m)")
axes[2].axhline(.08, color="black", linestyle=":", label="0.08 m latch")
axes[3].set_ylabel("Applied damping (m/s²)")
axes[3].set_xlabel("Seconds from ROUTE entry")
for axis in axes:
    axis.grid(alpha=.25)
    axis.legend(fontsize=8, loc="upper right")
fig.suptitle("Global-flight response: proven baseline versus opt-in damping")
fig.savefig(HERE / "flight_response_comparison.png", dpi=170)


map_sources = (
    (ROOT / "reports/migration_tunnel_20260921_0726/flight_maps.png", "07:26 baseline OFF — goal reached"),
    (HERE / "renders/111754/overview.png", "11:17 damping ON — goal reached + 3 s hold"),
    (HERE / "renders/112101/overview.png", "11:21 damping ON — no goal; VIO reset landing"),
)
map_fig, map_axes = plt.subplots(1, 3, figsize=(18, 7), layout="constrained")
for axis, (path, title) in zip(map_axes, map_sources):
    axis.imshow(plt.imread(path))
    axis.set_title(title)
    axis.axis("off")
map_fig.suptitle("Rendered global flights (map scale and orientation differ by run)")
map_fig.savefig(HERE / "flight_maps_comparison.png", dpi=150)

for label, row in metrics.items():
    if label == "mapped_clearance_min":
        continue
    print(label, json.dumps({
        "completed": row["completed_goal"],
        "duration": row["route_to_goal_or_land_seconds"],
        "speed_peak": row["route_measured_speed_peak"],
        "position_error_p95": row["route_position_error_p95"],
        "position_error_max": row["route_position_error_max"],
        "cross_track_max": row["route_cross_track_max"],
        "phases": row["phase_seconds"],
        "goal_hold": row["goal_hold_motion"],
        "damping": row["damping"],
    }, indent=2))
