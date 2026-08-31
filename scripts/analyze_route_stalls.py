#!/usr/bin/env python3
"""Answer one question about a global-planner bag: why is the route stop-start?

Usage (needs the ROS 2 environment, NOT .venv-mavlink):
    source /opt/ros/jazzy/setup.bash
    source /home/john/ros2_ws/install/setup.bash
    python3 scripts/analyze_route_stalls.py ros2_ws/flight_logs/offboard_global_<stamp> [...]

`scripts/render_flight_map.py` shows where the vehicle was when a *fault* fired.
This shows the opposite and more common case: the adapter is in ROUTE, the
follower is FOLLOWING and valid, nothing is faulted, and the vehicle is still
barely moving. On the three flights of 2026-08-31T064x, 24 of 27 stop onsets
looked exactly like that.

The measurement that explains it is the pairing of two numbers that live in
different nodes:

  * how often the follower accepts a new path generation, and
  * what `PathCommandLimiter::set_path` does to command speed when it does.

A replacement path whose geometry ahead is not identical to the installed one
takes the join branch, which sets `speed_ = 0.0` and re-accelerates the command
point from rest at `command_acceleration`. Reaching `command_speed` 0.20 m/s at
0.30 m/s^2 takes 0.67 s. If paths arrive faster than that, the commanded speed
is a sawtooth that never reaches cruise, and mean commanded speed lands near
half of maximum however healthy the flight is.

So the headline pair is REBUILD RATE against RAMP TIME. Everything else printed
here is there to rule out the alternatives: faults, yaw pauses, corner holds,
and a carrot that is not actually asking for full lookahead.

Definitions used throughout:
  stop        commanded speed below `--stop-speed` (default 0.05 m/s)
  rebuild     a change in /planner/follower/path_generation
  attributed  the adapter and follower states at the instant a stop began
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import sys
from collections import defaultdict

import numpy as np
from scipy import ndimage

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = (
    "/planner/flight/status",
    "/planner/follower/status",
    "/planner/follower/path_generation",
    "/planner/follower/remaining",
    "/planner/follower/cross_track",
    "/planner/status",
    "/planner/path",
    "/planner/config",
    "/planner/follower/config",
    "/rtabmap/body_pose",
    "/rtabmap/grid",
)

# Config keys worth echoing: the ones that set the ramp and the rebuild rate.
CONFIG_KEYS = (
    "robot_radius", "safety_margin", "max_cross_track", "cross_track_resume",
    "cross_track_recovery_time", "cross_track_filter_time_constant",
    "path_retain_tolerance", "switch_improvement", "lookahead",
    "max_carrot_speed", "max_carrot_acceleration",
)


def read(bag: str) -> tuple[dict, float]:
    """Load the topics of interest. Accepts a bag directory or the .mcap."""
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
    wanted = set(TOPICS)
    out = {name: [] for name in wanted}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in wanted:
            continue
        try:
            out[topic].append((stamp, deserialize_message(data, get_message(types[topic]))))
        except Exception:
            # A torn final record in a killed bag, or a message type this
            # workspace no longer builds. Neither is worth aborting a report.
            continue
    for name in out:
        out[name].sort(key=lambda item: item[0])
    stamps = [v[0][0] for v in out.values() if v]
    if not stamps:
        raise SystemExit(f"{bag}: none of the planner topics are present")
    return out, min(stamps)


def field(status: str, pattern: str) -> float | None:
    found = re.search(pattern, status)
    return float(found.group(1)) if found else None


def analyse(bag: str, stop_speed: float) -> None:
    data, t0 = read(bag)
    seconds = lambda stamp: (stamp - t0) / 1e9  # noqa: E731

    print(f"########## {os.path.basename(os.path.normpath(bag))}")

    for topic in ("/planner/config", "/planner/follower/config"):
        if data[topic]:
            config = json.loads(data[topic][0][1].data)
            shown = "  ".join(
                f"{k}={config[k]}" for k in CONFIG_KEYS if k in config)
            print(f"  {topic}\n    {shown}")

    flight = [(seconds(s), m.data) for s, m in data["/planner/flight/status"]]
    if not flight:
        print("  no /planner/flight/status -- not a global-planner flight bag\n")
        return
    span = flight[-1][0] - flight[0][0]
    kind = lambda text: text.split()[0]  # noqa: E731

    # --- adapter occupancy ---------------------------------------------------
    occupancy, entries, previous = defaultdict(float), defaultdict(int), None
    for i in range(len(flight) - 1):
        state = kind(flight[i][1])
        occupancy[state] += flight[i + 1][0] - flight[i][0]
        if state != previous:
            entries[state] += 1
            previous = state
    print(f"\n  ADAPTER STATE over {span:.1f}s")
    for state, held in sorted(occupancy.items(), key=lambda kv: -kv[1]):
        print(f"    {held:6.1f}s {100 * held / span:5.1f}%  x{entries[state]:<3d} {state}")

    # --- the headline pair ---------------------------------------------------
    speeds = [(t, field(s, r"command_speed=([\d.]+)")) for t, s in flight]
    speeds = [(t, v) for t, v in speeds if v is not None]
    generations = [(seconds(s), m.data) for s, m in data["/planner/follower/path_generation"]]

    if not speeds:
        print("  no ROUTE samples with a command_speed\n")
        return
    times = np.array([t for t, _ in speeds])
    values = np.array([v for _, v in speeds])
    route_span = times[-1] - times[0]

    rebuilds = np.array([])
    if generations:
        gt = np.array([t for t, _ in generations])
        gv = np.array([v for _, v in generations])
        rebuilds = gt[1:][np.diff(gv) != 0]

    config = json.loads(data["/planner/follower/config"][0][1].data) \
        if data["/planner/follower/config"] else {}
    top = config.get("max_carrot_speed", 0.20)
    accel = config.get("max_carrot_acceleration", 0.30)
    ramp = top / accel if accel > 0 else math.inf

    print("\n  WHY IT IS STOP-START")
    rate = len(rebuilds) / route_span if route_span > 0 else 0.0
    print(f"    path rebuilds        {len(rebuilds)} in {route_span:.1f}s = {rate:.2f}/s"
          f"  (one every {1 / rate:.2f}s)" if rate else "    path rebuilds        none")
    print(f"    ramp to full speed   {ramp:.2f}s  ({top:.2f} m/s at {accel:.2f} m/s^2)")
    if rate > 0:
        margin = 1 / rate - ramp
        print(f"    => {margin:+.2f}s of margin between arrivals and the ramp; whether that"
              f" matters is settled by the profile below, not by this number")

    print(f"\n    commanded speed      mean {values.mean():.3f}  p50 {np.median(values):.3f}"
          f"  max {values.max():.3f} m/s  (ceiling {top:.2f})")
    print(f"    fraction of maximum  {values.mean() / top:.0%}")

    moving = values > stop_speed
    falling = np.where(np.diff(moving.astype(int)) == -1)[0]
    rising = np.where(np.diff(moving.astype(int)) == 1)[0]
    durations = []
    for edge in falling:
        after = rising[rising > edge]
        end = times[after[0] + 1] if len(after) else times[-1]
        durations.append(end - times[edge + 1])
    if durations:
        print(f"    below {stop_speed:.2f} m/s        {100 * (1 - moving.mean()):.1f}% of ROUTE samples")
        print(f"    discrete stops       {len(durations)}, total {sum(durations):.1f}s, "
              f"median {np.median(durations):.2f}s, longest {max(durations):.2f}s")
        print(f"                         one every {span / len(durations):.1f}s of flight")

    # --- is anything actually wrong when it stops? ---------------------------
    follower = [(seconds(s), m.data.split()[0]) for s, m in data["/planner/follower/status"]]

    def state_at(when, series):
        seen = None
        for t, value in series:
            if t <= when:
                seen = value
            else:
                break
        return seen or "?"

    if len(falling):
        attributed, near_rebuild = defaultdict(int), 0
        for edge in falling:
            when = times[edge + 1]
            adapter = state_at(when, [(t, kind(s)) for t, s in flight])
            attributed[f"{adapter} / {state_at(when, follower)}"] += 1
            if len(rebuilds) and np.abs(rebuilds - when).min() <= 0.60:
                near_rebuild += 1
        print("\n    stop onset, adapter / follower state:")
        for label, count in sorted(attributed.items(), key=lambda kv: -kv[1]):
            healthy = " <- nothing is faulted" if label == "ROUTE / FOLLOWING" else ""
            print(f"      x{count:<3d} {label}{healthy}")
        print(f"    stop onsets within 0.60s of a rebuild: {near_rebuild}/{len(falling)}")

    # How far is the standing command point from the route that just arrived?
    # This sizes the only cheap fix. `PathCommandLimiter::set_path` sends any
    # replacement with a non-zero offset through the join manoeuvre, which
    # decelerates to a standstill by construction, and only an offset at or
    # below the snap tolerance keeps its speed. If most replans land within a
    # grid cell, most of the stopping is being bought for nothing.
    offsets = [(t, field(s, r"path_offset=([\d.]+)m")) for t, s in flight]
    offsets = [(t, v) for t, v in offsets if v is not None]
    if offsets and len(rebuilds):
        ot = np.array([t for t, _ in offsets]); ov = np.array([v for _, v in offsets])
        at_rebuild = []
        for when in rebuilds:
            window = (ot >= when) & (ot <= when + 0.35)
            if window.sum():
                at_rebuild.append(ov[window].max())
        if at_rebuild:
            arr = np.array(at_rebuild)
            print(f"    command-point offset when a new path arrives: p50 {np.median(arr):.3f}"
                  f"  p90 {np.percentile(arr, 90):.3f}  max {arr.max():.3f} m")
            for tol in (0.005, 0.010, 0.020, 0.030):
                print(f"      within {tol * 1000:.0f} mm of the new path: "
                      f"{100 * (arr <= tol).mean():.0f}% of rebuilds")

    if len(rebuilds):
        # The decisive plot, flattened to text: commanded speed as a function of
        # how long ago the last path arrived. If a rebuild resets the command
        # point's speed, this rises from near zero with the slope of
        # `max_carrot_acceleration`. If it is flat, rebuilds are not the brake
        # and the leash is -- the carrot can only ever sit `lookahead` ahead of
        # a vehicle the position controller is dragging along behind it.
        since = np.full(len(times), np.inf)
        for i, when in enumerate(times):
            earlier = rebuilds[rebuilds <= when]
            if len(earlier):
                since[i] = when - earlier[-1]
        bins = [(0.0, 0.2), (0.2, 0.4), (0.4, 0.6), (0.6, 0.9), (0.9, 1.5), (1.5, np.inf)]
        print("    commanded speed vs time since the last path rebuild:")
        for lo, hi in bins:
            window = (since >= lo) & (since < hi)
            if window.sum() < 3:
                continue
            label = f"{lo:.1f}-{hi:.1f}s" if np.isfinite(hi) else f"{lo:.1f}s+"
            print(f"      {label:>10}  n={window.sum():<4d} mean {values[window].mean():.3f}"
                  f"  p90 {np.percentile(values[window], 90):.3f} m/s")

    # --- rule out the alternatives ------------------------------------------
    print("\n  RULING OUT THE ALTERNATIVES")
    displacement = [field(s, r"displacement=([\d.]+)m") for _, s in flight]
    displacement = [v for v in displacement if v is not None]
    if displacement:
        want = config.get("lookahead", 0.25)
        print(f"    carrot distance      p50 {np.median(displacement):.3f} m against "
              f"lookahead {want:.2f} -- the follower {'is' if abs(np.median(displacement) - want) < 0.02 else 'is NOT'} "
              f"asking for full lookahead")
    corner = occupancy.get("CORNER_HOLD", 0.0)
    yaw = occupancy.get("YAW_ALIGN", 0.0)
    print(f"    corner holds         {corner:.1f}s ({100 * corner / span:.1f}%)")
    print(f"    yaw-align pauses     {yaw:.1f}s ({100 * yaw / span:.1f}%)")
    targets = [field(s, r"yaw_target=(-?\d+)deg") for _, s in flight]
    targets = [v for v in targets if v is not None]
    if targets:
        unwrapped = np.unwrap(np.radians(targets))
        variation = np.degrees(np.abs(np.diff(unwrapped))).sum()
        extent = max(np.degrees(unwrapped.max() - unwrapped.min()), 1.0)
        print(f"    yaw_target travel    {variation:.0f} deg over a {extent:.0f} deg span"
              f" = {variation / extent:.1f}x more turning than the route needs")
    held = sum(v for k, v in occupancy.items() if k in ("HOLD", "COMMAND_HOLD"))
    print(f"    faulted / held       {held:.1f}s ({100 * held / span:.1f}%)")

    # --- what it cost --------------------------------------------------------
    remaining = [(seconds(s), m.data) for s, m in data["/planner/follower/remaining"]]
    print("\n  WHAT IT COST")
    if remaining:
        elapsed = max(remaining[-1][0] - remaining[0][0], 1e-9)
        closure = (remaining[0][1] - remaining[-1][1]) / elapsed
        print(f"    goal closure         {closure:.3f} m/s against {top:.2f} commanded"
              f"  ({closure / top:.0%})")
        print(f"    remaining            {remaining[0][1]:.2f} -> {remaining[-1][1]:.2f} m"
              f" in {elapsed:.1f}s")
    poses = [(seconds(s), (m.pose.position.x, m.pose.position.y))
             for s, m in data["/rtabmap/body_pose"]]
    if len(poses) > 1:
        xy = np.array([p for _, p in poses])
        travelled = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
        net = float(np.linalg.norm(xy[-1] - xy[0]))
        print(f"    travelled            {travelled:.2f} m for {net:.2f} m net"
              f"  (wander {travelled / max(net, 1e-6):.1f}x)")

    # --- does the corridor still admit the configured clearance? -------------
    #
    # A stop is not always a control problem. `lethal_radius` is
    # robot_radius + safety_margin, and raising the margin shrinks the set of
    # cells the planner may route through by a lot more than it sounds: on the
    # 20260831T064x maps every 0.05 m of margin costs roughly a quarter of the
    # navigable area. CLEARANCE_ESCAPING and CLEARANCE_BLOCKED at a pinch that
    # measures just under the requirement is that trade showing up as a stall,
    # not as a planning failure -- so price it before touching the follower.
    if data["/rtabmap/grid"] and len(poses) > 1:
        grid = data["/rtabmap/grid"][-1][1]
        cells = np.array(grid.data, dtype=np.int16).reshape(
            grid.info.height, grid.info.width)
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        occupied = cells >= config.get("occupied_threshold", 65)
        unknown = cells < 0
        free = ~occupied & ~unknown
        # Unknown is blocked for the planner, so it bounds clearance too.
        distance = ndimage.distance_transform_edt(~(occupied | unknown)) * res
        lethal = config.get("robot_radius", 0.25) + config.get("safety_margin", 0.0)
        print("\n  CORRIDOR")
        print(f"    lethal_radius {lethal:.2f} m -> minimum plannable gap "
              f"{2 * lethal:.2f} m")
        for probe in (lethal - 0.05, lethal, lethal + 0.05):
            if probe <= 0:
                continue
            area = float((free & (distance >= probe)).sum()) * res * res
            mark = "  <- as configured" if abs(probe - lethal) < 1e-9 else ""
            print(f"      navigable at {probe:.2f} m: {area:5.2f} m2{mark}")
        ix = ((xy[:, 0] - ox) / res).astype(int)
        iy = ((xy[:, 1] - oy) / res).astype(int)
        inside = (ix >= 0) & (iy >= 0) & (ix < cells.shape[1]) & (iy < cells.shape[0])
        along = distance[iy[inside], ix[inside]]
        if len(along):
            print(f"    clearance along the flown track: min {along.min():.3f}"
                  f"  p10 {np.percentile(along, 10):.3f}  p50 {np.median(along):.3f} m")
            print(f"      below the {lethal:.2f} m requirement for "
                  f"{100 * (along < lethal).mean():.1f}% of the track")
    print()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Explain stop-start motion in a global-planner flight bag.")
    parser.add_argument("bags", nargs="+", help="bag directories or .mcap files")
    parser.add_argument(
        "--stop-speed", type=float, default=0.05,
        help="commanded speed below which the route counts as stopped (m/s)")
    args = parser.parse_args()
    for bag in args.bags:
        analyse(bag, args.stop_speed)
    return 0


if __name__ == "__main__":
    sys.exit(main())
