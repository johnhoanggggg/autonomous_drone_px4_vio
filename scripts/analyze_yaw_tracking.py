#!/usr/bin/env python3
"""Does the nose point where the vehicle is actually going?

Usage (needs the ROS 2 environment, NOT .venv-mavlink):
    source /opt/ros/jazzy/setup.bash
    source /home/john/ros2_ws/install/setup.bash
    python3 scripts/analyze_yaw_tracking.py ros2_ws/flight_logs/offboard_global_<stamp> [...]

The adapter turns the vehicle onto each leg so the camera looks along the route
(`yaw_track`, on by default). When that goes wrong the drone flies sideways or
crabs into a wall, and the OAK-D then stares at a blank surface, which starves
the feature tracker exactly where the geometry is tightest.

Four angles get compared, and which pair disagrees says which stage is at fault:

  course        atan2(vy, vx) from PX4's own local position -- where the
                vehicle is actually travelling, in NED
  heading       PX4's own heading -- where the nose actually points, NED
  yaw_target    what the adapter asked for, scraped from /planner/flight/status
  map yaw       the yaw of /rtabmap/body_pose, i.e. the same nose expressed in
                the SLAM map frame

  heading - course      nose vs travel. This is the number the pilot sees. A
                        steady bias is a frame or sign error; scatter that
                        tracks turns is ordinary slew lag.
  yaw_target - course   whether the adapter ASKED for the wrong direction.
  heading - yaw_target  whether PX4 simply has not got there yet (slew lag).
  map yaw - heading     whether the SLAM frame and PX4 agree about which way
                        the vehicle faces at all. A constant offset here means
                        a yaw target computed from map-frame displacement is
                        rotated by that constant before it reaches the nose.

Only samples above `--min-speed` are used: course is meaningless at a hover.
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import re
import sys

import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = (
    "/fmu/out/vehicle_local_position_v1",
    "/planner/flight/status",
    "/rtabmap/body_pose",
    "/rtabmap/vio_feature_count",
    "/planner/follower/vio_displacement",
)


def read(bag: str):
    mcap = bag
    if os.path.isdir(bag):
        found = glob.glob(os.path.join(bag, "*.mcap"))
        if not found:
            raise SystemExit(f"no .mcap inside {bag}")
        mcap = found[0]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=mcap, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {name: [] for name in TOPICS}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in out:
            continue
        try:
            out[topic].append((stamp, deserialize_message(data, get_message(types[topic]))))
        except Exception:
            continue
    for name in out:
        out[name].sort(key=lambda item: item[0])
    stamps = [v[0][0] for v in out.values() if v]
    if not stamps:
        raise SystemExit(f"{bag}: none of the required topics are present")
    return out, min(stamps)


def wrap(angle):
    return (np.asarray(angle) + np.pi) % (2 * np.pi) - np.pi


def circular_stats(values):
    """Mean and spread of an angle set, done on the circle rather than the line."""
    s, c = np.sin(values).mean(), np.cos(values).mean()
    mean = math.atan2(s, c)
    resultant = math.hypot(s, c)
    # Circular standard deviation; 0 means every sample agrees.
    spread = math.sqrt(-2.0 * math.log(resultant)) if resultant > 1e-12 else math.inf
    return mean, spread


def describe(label, values, note=""):
    if len(values) < 5:
        print(f"    {label:<22} too few samples")
        return
    mean, spread = circular_stats(values)
    print(f"    {label:<22} mean {math.degrees(mean):+7.1f}  sd {math.degrees(spread):6.1f}"
          f"  p50|{np.degrees(np.median(np.abs(values))):5.1f}|"
          f"  p90|{np.degrees(np.percentile(np.abs(values), 90)):5.1f}|  {note}")


def yaw_of(pose_msg):
    q = pose_msg.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def analyse(bag, min_speed):
    data, t0 = read(bag)
    seconds = lambda s: (s - t0) / 1e9  # noqa: E731
    print(f"########## {os.path.basename(os.path.normpath(bag))}")

    lp = data["/fmu/out/vehicle_local_position_v1"]
    if not lp:
        print("  no PX4 local position in this bag\n")
        return
    lt = np.array([seconds(s) for s, _ in lp])
    vx = np.array([m.vx for _, m in lp])
    vy = np.array([m.vy for _, m in lp])
    heading = np.array([m.heading for _, m in lp])
    speed = np.hypot(vx, vy)
    moving = speed >= min_speed
    print(f"  {moving.sum()} of {len(lp)} samples above {min_speed:.2f} m/s"
          f"  (max {speed.max():.2f} m/s)")
    if moving.sum() < 5:
        print("  never moved enough to have a meaningful course\n")
        return

    course = np.arctan2(vy[moving], vx[moving])
    head_m = heading[moving]
    print("\n  ANGLES (degrees, + is clockwise from north)")
    describe("heading - course", wrap(head_m - course), "<- nose vs travel")

    # What the adapter asked for.
    flight = [(seconds(s), m.data) for s, m in data["/planner/flight/status"]]
    targets = [(t, float(v.group(1))) for t, s in flight
               for v in [re.search(r"yaw_target=(-?\d+)deg", s)] if v]
    if targets:
        tt = np.array([t for t, _ in targets])
        tv = np.unwrap(np.radians([v for _, v in targets]))
        at_move = np.interp(lt[moving], tt, tv)
        describe("yaw_target - course", wrap(at_move - course), "<- was the ASK wrong?")
        describe("heading - yaw_target", wrap(head_m - at_move), "<- slew lag")

    # Do the SLAM frame and PX4 agree about which way the vehicle faces?
    body = data["/rtabmap/body_pose"]
    if body:
        bt = np.array([seconds(s) for s, _ in body])
        by = np.unwrap([yaw_of(m) for _, m in body])
        at_move = np.interp(lt[moving], bt, by)
        offset = wrap(at_move - head_m)
        describe("map yaw - PX4 heading", offset, "<- frame agreement")
        mean, spread = circular_stats(offset)
        if math.degrees(spread) < 20.0 and abs(math.degrees(mean)) > 10.0:
            print(f"      => a CONSTANT {math.degrees(mean):+.0f} deg offset between the two"
                  f" frames (sd {math.degrees(spread):.0f} deg).")
            print(f"         A yaw target derived from map-frame displacement is rotated by"
                  f" this before it reaches the nose.")

    # Where the camera was pointing when the tracker starved.
    feats = data["/rtabmap/vio_feature_count"]
    if feats and targets:
        ft = np.array([seconds(s) for s, _ in feats])
        fv = np.array([m.data for _, m in feats])
        crab = np.abs(np.degrees(wrap(np.interp(ft, lt[moving], np.unwrap(head_m - course)))))
        lean = crab >= 30.0
        if lean.sum() >= 5 and (~lean).sum() >= 5:
            print(f"\n  VIO FEATURES vs how far the nose is off the direction of travel")
            print(f"    crabbing >= 30 deg: mean {fv[lean].mean():5.0f} features"
                  f"  (n={lean.sum()})")
            print(f"    crabbing <  30 deg: mean {fv[~lean].mean():5.0f} features"
                  f"  (n={(~lean).sum()})")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Compare commanded yaw, actual heading and actual course.")
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--min-speed", type=float, default=0.05,
                        help="ignore samples slower than this (m/s)")
    args = parser.parse_args()
    for bag in args.bags:
        analyse(bag, args.min_speed)
    return 0


if __name__ == "__main__":
    sys.exit(main())
