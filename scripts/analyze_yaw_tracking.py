#!/usr/bin/env python3
"""Does the nose point where the vehicle is actually going?

Usage (needs the ROS 2 environment, NOT .venv-mavlink):
    source /opt/ros/jazzy/setup.bash
    source /home/john/ros2_ws/install/setup.bash
    python3 scripts/analyze_yaw_tracking.py ros2_ws/flight_logs/offboard_global_<stamp> [...]

The adapter turns the vehicle onto each leg so the camera looks along the route
(`yaw_track`, on by default). A mismatch can make the drone travel sideways.
Whether it also harms tracking depends on what the camera sees; heading and
feature counts alone cannot establish a camera-to-wall viewing angle.

Four angles get compared, and which pair disagrees says which stage is at fault:

  course        atan2(vy, vx) from PX4's own local position -- where the
                vehicle is actually travelling, in NED
  heading       PX4's own heading -- where the nose actually points, NED
  yaw_target    what the adapter asked for, scraped from /planner/flight/status
  body VIO yaw  the yaw of /rtabmap/body_vio_pose, converted to NED

  heading - course      nose vs travel. This is the number the pilot sees. A
                        bias can reflect command geometry, drift or a frame
                        error; scatter during turns can include slew lag.
  yaw_target - course   whether the adapter ASKED for the wrong direction.
  heading - yaw_target  whether PX4 simply has not got there yet (slew lag).
  body VIO yaw (NED) - heading   compares the continuous VIO frame with PX4.
                        ENU yaw becomes pi/2-yaw in NED. Map yaw additionally
                        includes loop-closure rotation and cannot be directly
                        subtracted from PX4 heading.

Only samples above `--min-speed` in ROUTE/YAW_ALIGN/CORNER_HOLD are used.
Pairs use recent past receipt times, not interpolation across missing data.
Position error and feedforward both affect travel, so neither alone predicts
the actual course. No live ROS publishers are created.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import glob
import json
import math
import os
import re
import sys

import numpy as np

TOPICS = (
    "/fmu/out/vehicle_local_position_v1",
    "/planner/flight/status",
    "/rtabmap/body_pose",
    "/rtabmap/vio_feature_count",
    "/planner/follower/vio_displacement",
    "/rtabmap/body_vio_pose",
    "/vio/yaw_offset/pose",
    "/fmu/in/trajectory_setpoint",
    "/fmu/out/vehicle_status_v1",
)


def read(bag: str, topics=TOPICS):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    mcap = bag
    if os.path.isdir(bag):
        found = glob.glob(os.path.join(bag, "*.mcap"))
        if len(found) != 1:
            raise ValueError(f"expected one MCAP in {bag}; found {len(found)}")
        mcap = found[0]
    if os.path.getsize(mcap) == 0:
        raise ValueError("MCAP is empty; no recoverable flight samples")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=mcap, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {name: [] for name in topics}
    decode_errors = Counter()
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in out:
            continue
        try:
            out[topic].append((stamp, deserialize_message(data, get_message(types[topic]))))
        except Exception:
            decode_errors[topic] += 1
    for name in out:
        out[name].sort(key=lambda item: item[0])
    stamps = [v[0][0] for v in out.values() if v]
    if not stamps:
        raise ValueError(f"{bag}: none of the required topics are present")
    out["_decode_errors"] = dict(decode_errors)
    return out, min(stamps)


def wrap(angle):
    return (np.asarray(angle) + np.pi) % (2 * np.pi) - np.pi


def circular_stats(values):
    """Mean and spread of an angle set, done on the circle rather than the line."""
    s, c = np.sin(values).mean(), np.cos(values).mean()
    mean = math.atan2(s, c)
    resultant = min(1.0, math.hypot(s, c))
    # Circular standard deviation; 0 means every sample agrees.
    spread = math.sqrt(-2.0 * math.log(resultant)) if resultant > 1e-12 else math.inf
    return mean, spread


def describe(label, values, note=""):
    values = np.asarray(values)
    values = wrap(values[np.isfinite(values)])
    if len(values) < 5:
        print(f"    {label:<22} too few samples")
        return
    mean, spread = circular_stats(values)
    print(f"    {label:<22} mean {math.degrees(mean):+7.1f}  sd {math.degrees(spread):6.1f}"
          f"  p50|{np.degrees(np.median(np.abs(values))):5.1f}|"
          f"  p90|{np.degrees(np.percentile(np.abs(values), 90)):5.1f}|  {note}")


def yaw_of(pose_msg):
    q = pose_msg.pose.orientation
    values = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(values).all() or norm < 1e-6:
        return math.nan
    x, y, z, w = values / norm
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class RecentSamples:
    """Latest received sample, bounded in age; never uses future data."""
    def __init__(self, samples):
        self.samples = samples
        self.times = [stamp for stamp, _ in samples]

    def at(self, stamp, max_age=0.30):
        index = bisect_right(self.times, stamp) - 1
        if index < 0 or (stamp - self.times[index]) / 1e9 > max_age:
            return None
        return self.samples[index][1]


def enu_yaw_to_ned(yaw):
    return float(wrap(math.pi / 2 - yaw))


def angle_stats(values):
    values = np.asarray(values, dtype=float)
    values = wrap(values[np.isfinite(values)])
    if not len(values):
        return {"samples": 0}
    mean, spread = circular_stats(values)
    return {
        "samples": len(values),
        "mean_deg": round(math.degrees(mean), 2),
        "sd_deg": round(math.degrees(spread), 2) if math.isfinite(spread) else None,
        "median_abs_deg": round(float(np.degrees(np.median(np.abs(values)))), 2),
        "p90_abs_deg": round(float(np.degrees(np.percentile(np.abs(values), 90))), 2),
    }


def analyse_data(data, t0, min_speed=0.10, max_age=0.30):
    streams = {name: RecentSamples(data.get(name, [])) for name in TOPICS}
    metrics = defaultdict(list)
    phases = defaultdict(lambda: defaultdict(list))
    counts = Counter()
    features = {"crab_ge_30deg": [], "crab_lt_30deg": []}
    seen_features = set()
    timeline = []
    last_second = None

    for stamp, pose in data.get("/fmu/out/vehicle_local_position_v1", []):
        counts["local_position"] += 1
        if (not pose.v_xy_valid or
                not all(math.isfinite(x) for x in (pose.vx, pose.vy, pose.heading))):
            continue
        speed = math.hypot(pose.vx, pose.vy)
        if speed < min_speed:
            continue
        status = streams["/planner/flight/status"].at(stamp, max_age)
        phase = status.data.split()[0] if status and status.data.split() else ""
        if phase not in {"ROUTE", "YAW_ALIGN", "CORNER_HOLD"}:
            continue
        if data.get("/fmu/out/vehicle_status_v1"):
            vehicle = streams["/fmu/out/vehicle_status_v1"].at(stamp, 1.0)
            if (vehicle is None or
                    vehicle.arming_state != vehicle.ARMING_STATE_ARMED or
                    vehicle.nav_state != vehicle.NAVIGATION_STATE_OFFBOARD):
                continue
        counts["moving_route"] += 1
        counts[phase] += 1
        course = math.atan2(pose.vy, pose.vx)
        angles = {"heading_minus_course": pose.heading - course}
        match = re.search(r"yaw_target=(-?\d+(?:\.\d+)?)deg", status.data)
        if match:
            target = math.radians(float(match.group(1)))
            angles["heading_minus_target"] = pose.heading - target
            angles["target_minus_course"] = target - course

        follower = streams["/planner/follower/vio_displacement"].at(stamp, max_age)
        follow = math.nan
        if follower and math.hypot(follower.vector.x, follower.vector.y) >= 0.15:
            follow = math.atan2(follower.vector.x, follower.vector.y)
            angles["follower_minus_course"] = follow - course

        sp = streams["/fmu/in/trajectory_setpoint"].at(stamp, max_age)
        wire_yaw = ff_heading = error_heading = math.nan
        if sp:
            wire_yaw = sp.yaw
            angles["heading_minus_wire_yaw"] = pose.heading - wire_yaw
            angles["wire_yaw_minus_course"] = wire_yaw - course
            vx, vy = sp.velocity[:2]
            if math.hypot(vx, vy) >= 0.03:
                ff_heading = math.atan2(vy, vx)
                angles["follower_minus_feedforward"] = follow - ff_heading
                angles["wire_yaw_minus_feedforward"] = wire_yaw - ff_heading
            dx, dy = sp.position[0] - pose.x, sp.position[1] - pose.y
            if pose.xy_valid and math.hypot(dx, dy) >= 0.05:
                error_heading = math.atan2(dy, dx)
                angles["wire_yaw_minus_position_error"] = wire_yaw - error_heading

        for label, topic in (("body_vio", "/rtabmap/body_vio_pose"),
                             ("bridge_input", "/vio/yaw_offset/pose")):
            sample = streams[topic].at(stamp, max_age)
            if sample:
                angles[label + "_ned_minus_heading"] = enu_yaw_to_ned(yaw_of(sample)) - pose.heading
        for name, angle in angles.items():
            if math.isfinite(angle):
                metrics[name].append(angle)
                phases[phase][name].append(angle)

        feature = streams["/rtabmap/vio_feature_count"].at(stamp, max_age)
        if feature is not None and id(feature) not in seen_features:
            seen_features.add(id(feature))
            crab = abs(math.degrees(float(wrap(pose.heading - course))))
            features["crab_ge_30deg" if crab >= 30 else "crab_lt_30deg"].append(feature.data)

        second = int((stamp - t0) / 1e9)
        if second != last_second:
            last_second = second
            def deg(value):
                return round(math.degrees(float(wrap(value))), 1) if math.isfinite(value) else None
            timeline.append({
                "at_s": round((stamp - t0) / 1e9, 2), "phase": phase,
                "speed_mps": round(speed, 3), "heading_deg": deg(pose.heading),
                "course_deg": deg(course), "wire_yaw_deg": deg(wire_yaw),
                "follower_deg": deg(follow), "feedforward_deg": deg(ff_heading),
                "position_error_direction_deg": deg(error_heading),
            })
    return {
        "min_speed_mps": min_speed, "max_pair_age_s": max_age,
        "decode_errors": data.get("_decode_errors", {}),
        "vehicle_status_filter_available": bool(data.get("/fmu/out/vehicle_status_v1")),
        "sample_counts": dict(counts),
        "angles": {name: angle_stats(v) for name, v in metrics.items()},
        "by_phase": {phase: {name: angle_stats(v) for name, v in group.items()}
                     for phase, group in phases.items()},
        "feature_counts": {name: {"samples": len(v),
                                  "mean": round(float(np.mean(v)), 1) if v else None}
                           for name, v in features.items()},
        "timeline": timeline,
        "limitations": [
            "Receipt-time alignment includes sensor/transport latency.",
            "When vehicle status is unavailable, route text is the only flight-phase filter.",
            "Feedforward and position error both influence motion; neither alone predicts course.",
            "Feature-count association cannot prove the camera faced a wall or caused tracking loss.",
        ],
    }


def analyse(bag, min_speed, max_age=0.30):
    data, t0 = read(bag)
    result = {"bag": bag, **analyse_data(data, t0, min_speed, max_age)}
    print(os.path.basename(os.path.normpath(bag)))
    print(f"  moving route samples: {result['sample_counts'].get('moving_route', 0)}")
    if result["decode_errors"]:
        print(f"  excluded undecodable messages: {result['decode_errors']}")
    if not result["vehicle_status_filter_available"]:
        print("  vehicle status unavailable: phase filtered using planner status only")
    for name, stats in result["angles"].items():
        print(f"  {name:36} n={stats['samples']:4} mean={stats['mean_deg']:+6.1f}deg "
              f"median_abs={stats['median_abs_deg']:5.1f}deg p90_abs={stats['p90_abs_deg']:5.1f}deg")
    print(f"  feature counts: {result['feature_counts']}")
    return result


def main():
    parser = argparse.ArgumentParser(
        description="Compare commanded yaw, actual heading and actual course.")
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--min-speed", type=float, default=0.10,
                        help="ignore samples slower than this (m/s)")
    parser.add_argument("--max-age", type=float, default=0.30)
    parser.add_argument("--out", help="Optional JSON report with one-second timeline")
    args = parser.parse_args()
    if not all(math.isfinite(x) and x > 0 for x in (args.min_speed, args.max_age)):
        parser.error("min-speed and max-age must be finite and positive")
    results = []
    for bag in args.bags:
        try:
            results.append(analyse(bag, args.min_speed, args.max_age))
        except (ValueError, RuntimeError, OSError) as error:
            results.append({"bag": bag, "error": str(error)})
            print(f"{bag}: unavailable: {error}")
    if args.out:
        with open(args.out, "w") as output:
            json.dump(results, output, indent=2, allow_nan=False)
            output.write("\n")
        print(f"written to {args.out}")
    return int(any("error" in r for r in results))


if __name__ == "__main__":
    sys.exit(main())
