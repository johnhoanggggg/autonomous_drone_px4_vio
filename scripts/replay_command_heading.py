#!/usr/bin/env python3
"""Exercise heading previews with recorded geometry; never publishes ROS data.

This is NOT a flight simulation or full adapter replay. Recorded poses and hold
decisions are fixed inputs, clearance is stubbed true, and defaults describe the
current launch (not an assertion about historical flight configuration).

Checks preview non-mutation, and agreement with the next advancing limiter
velocity while the command point is ON the path. Agreement is deliberately not
required during a join: there the tick velocity carries a lateral lane-change
term the heading must ignore, which is what stalled 20260906T113239Z. The angle
between the two is reported per bag instead, as `joining_direction_degrees`.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import subprocess

from analyze_yaw_tracking import RecentSamples, read, yaw_of

POSE = "/rtabmap/body_pose"
PATH = "/planner/path"
CORRECTION = "/rtabmap/odom_correction"
DISPLACEMENT = "/planner/follower/vio_displacement"
STATUS = "/planner/flight/status"
CONFIG = dict(max_speed=0.2, max_acceleration=0.3, max_projection_error=0.05,
              corner_tolerance=0.05, max_entry_error=0.3, max_connector_error=0.2,
              suffix_tolerance=0.03, corner_blending=True, junction_deviation=0.05)


def spans(first, middle, last):
    """Whether `middle` lies in the angular wedge from `first` to `last`."""
    angle = lambda a, b: abs(math.atan2(  # noqa: E731
        a[0]*b[1] - a[1]*b[0], a[0]*b[0] + a[1]*b[1]))
    return abs(angle(first, middle) + angle(middle, last) - angle(first, last)) < 1e-9


def replay(bag, binary, config):
    data, start = read(bag, (POSE, PATH, CORRECTION, DISPLACEMENT, STATUS))
    streams = {name: RecentSamples(data[name]) for name in
               (POSE, PATH, CORRECTION, DISPLACEMENT, STATUS)}
    if any(not data[name] for name in streams):
        raise ValueError("missing required replay topics")
    end = max(data[name][-1][0] for name in streams)
    steps, checks = [], []
    in_route = False
    for stamp in range(start, end + 1, 50_000_000):
        status = streams[STATUS].at(stamp, 0.5)
        phase = status.data.split()[0] if status and status.data else ""
        pose = streams[POSE].at(stamp, 0.3)
        path = streams[PATH].at(stamp, 3.0)
        correction = streams[CORRECTION].at(stamp, 1.0)
        displacement = streams[DISPLACEMENT].at(stamp, 0.5)
        active = phase in ("ROUTE", "YAW_ALIGN", "CORNER_HOLD")
        if not active or any(x is None for x in (pose, path, correction, displacement)):
            if in_route:
                steps.append({"op": "clear"})
            in_route = False
            continue
        yaw = yaw_of(correction)
        if not math.isfinite(yaw):
            continue
        reference = [pose.pose.position.x, pose.pose.position.y]
        c, s = math.cos(yaw), math.sin(yaw)
        x, y = displacement.vector.x, displacement.vector.y
        desired = [reference[0] + c*x - s*y, reference[1] + s*x + c*y]
        if not in_route or phase != "YAW_ALIGN":
            steps.append({"op": "set_path", "reference": reference,
                          "points": [[p.pose.position.x, p.pose.position.y]
                                     for p in path.poses]})
        in_route = True
        checks.append(len(steps))
        args = dict(desired=desired, reference=reference, dt=0.05)
        steps.append(dict(op="preview_direction", **args))
        steps.append(dict(op="update", advance=phase != "YAW_ALIGN", **args))
    output = subprocess.run([str(binary)], input=json.dumps({"limiter": config,
                            "steps": steps}), text=True, capture_output=True,
                            check=True, timeout=60)
    results = json.loads(output.stdout)
    counts = Counter()
    errors = Counter()
    maximum_error = 0.0
    for step, result in zip(steps, results):
        if not result["ok"]:
            errors[step["op"] + ": " + result["error"]] += 1
    joining_degrees, joining_angles = 0.0, []
    previous_index, previous_direction = None, None
    for index in checks:
        preview, advanced = results[index:index + 2]
        if index and preview["path"] != results[index - 1]["path"]:
            raise AssertionError("preview mutated limiter state")
        counts["non_mutating_previews"] += 1
        if not preview["ok"] or not advanced["ok"]:
            continue
        counts["successful_previews"] += 1
        vx, vy = advanced["path"]["velocity"]
        speed = math.hypot(vx, vy)
        if not steps[index + 1]["advance"] or speed <= 1e-6:
            continue
        direction = preview["direction"]
        if direction is None:
            # A rejoin whose along-path rate is zero still moves, on the lateral
            # term alone. That is deliberately headingless: there is no route
            # travel to face, so the adapter keeps the yaw it has.
            if preview["path"]["joining"] or advanced["path"]["joining"]:
                counts["headingless_lateral_only_rejoins"] += 1
                continue
            raise AssertionError("advancing velocity has no preview direction")
        error = math.hypot(direction[0] - vx/speed, direction[1] - vy/speed)
        if preview["path"]["joining"] or advanced["path"]["joining"]:
            counts["joining_directions_reported"] += 1
            offset = math.degrees(abs(math.atan2(
                direction[0]*vy - direction[1]*vx, direction[0]*vx + direction[1]*vy)))
            joining_angles.append(offset)
            joining_degrees = max(joining_degrees, offset)
        elif error <= 1e-10:
            maximum_error = max(maximum_error, error)
            counts["advancing_directions_checked"] += 1
        elif (previous_direction is not None and index - previous_index <= 3 and
              spans(previous_direction, [vx/speed, vy/speed], direction)):
            # One tick straddles a direction change -- a blended corner, or the
            # first tick on a replacement leg. Its chord velocity is then part
            # old leg and part new, and the preview is already on the new one,
            # which is where the nose has to go. The index gap is 2 for a plain
            # tick and 3 when a set_path preceded this preview; anything larger
            # means samples were dropped and the pair is not consecutive.
            counts["chords_spanning_a_direction_change"] += 1
        else:
            raise AssertionError("preview differs from next advancing velocity")
        previous_index, previous_direction = index, direction
    if not counts["advancing_directions_checked"]:
        raise ValueError("no advancing directions exercised")
    return dict(bag=bag, checks=dict(counts), max_unit_vector_error=maximum_error,
                joining_direction_degrees=joining_degrees,
                joining_direction_p90=(sorted(joining_angles)[int(0.9*len(joining_angles))]
                                       if joining_angles else 0.0),
                joining_over_20_degrees=sum(a > 20.0 for a in joining_angles),
                rejected_operations=dict(errors), decode_errors=data["_decode_errors"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--binary", type=Path, default=Path(__file__).resolve().parents[1] /
                        "ros2_ws/build/px4_vio_bridge/planner_flight_replay")
    parser.add_argument("--config", type=Path, help="JSON limiter configuration override")
    args = parser.parse_args()
    config = CONFIG.copy()
    if args.config:
        config.update(json.loads(args.config.read_text()))
    print(json.dumps(dict(scope="geometry-only replay; no closed-loop or clearance validation",
                          config=config, results=[replay(b, args.binary, config)
                                                  for b in args.bags]), indent=2))


if __name__ == "__main__":
    main()
