#!/usr/bin/env python3
"""Render a flight's occupancy grid with the trajectory and planner state on top.

Offline only: reads an MCAP bag, writes PNGs. It publishes nothing and touches
no flight code.

The point of it is to make clearance failures visible. `POSE_INSIDE_CLEARANCE`,
`CLEARANCE_ESCAPING` and the adapter's `COMMAND_HOLD ... insufficient
clearance` are all statements about where the vehicle sat relative to an
obstacle, and reading them out of a status string one line at a time is how a
whole flight goes by before the geometry is obvious.

Two different clearance numbers appear here, and they are not interchangeable:

* the printed per-event figures are EXACT -- distance from the pose to the full
  axis-aligned square of every occupied cell in a bounded window, the same
  measure `segment_minimum_clearance()` uses in flight;
* the shaded band on the image is an approximation from a distance transform on
  a supersampled mask, accurate to about `resolution / supersample`. It is there
  to show shape, not to be read off.

Usage:
    render_flight_map.py BAG [-o OUT_DIR] [--clearance 0.25] [--at 31.4 ...]
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import re
import sys

import numpy as np
from PIL import Image, ImageDraw

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# Follower/adapter states worth colouring differently along the track.
STATE_COLOURS = {
    "FOLLOWING": (60, 200, 90),
    "GOAL_REACHED": (40, 240, 140),
    "CLEARANCE_ESCAPING": (255, 170, 0),
    "CLEARANCE_BLOCKED": (255, 40, 40),
    "CROSS_TRACK_EXCEEDED": (255, 0, 140),
    "CROSS_TRACK_HOLD": (210, 60, 160),
    "CROSS_TRACK_HOLD_WAITING_FOR_PATH": (210, 60, 160),
    "CROSS_TRACK_RECOVERING": (170, 90, 200),
    "CORRECTION_SETTLING": (90, 160, 255),
    "WAITING_FOR_POST_CORRECTION_PATH": (90, 160, 255),
}
OTHER_COLOUR = (150, 150, 150)

TOPICS = (
    "/rtabmap/grid",
    "/rtabmap/pose",
    "/planner/path",
    "/planner/follower/status",
    "/planner/flight/status",
    "/planner/status",
    "/waypoint/clicked",
    "/planner/effective_goal",
)


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {topic: [] for topic in TOPICS}
    start = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if start is None:
            start = stamp
        if topic in out:
            out[topic].append(
                ((stamp - start) / 1e9, deserialize_message(data, get_message(types[topic])))
            )
    return out


class Grid:
    """An occupancy grid plus the exact clearance query used in flight."""

    def __init__(self, msg):
        self.width = int(msg.info.width)
        self.height = int(msg.info.height)
        self.resolution = float(msg.info.resolution)
        self.origin_x = float(msg.info.origin.position.x)
        self.origin_y = float(msg.info.origin.position.y)
        self.data = np.asarray(msg.data, dtype=np.int16).reshape(self.height, self.width)

    def world_to_cell(self, x, y):
        return (
            int(math.floor((x - self.origin_x) / self.resolution)),
            int(math.floor((y - self.origin_y) / self.resolution)),
        )

    def point_clearance(self, x, y, occupied_threshold=65, window=1.0):
        """Exact distance to the nearest occupied cell square, or None.

        None means the point is in unknown or outside-map space, which is
        blocked rather than clear -- the same answer the flight code gives.
        """
        cx, cy = self.world_to_cell(x, y)
        if not (0 <= cx < self.width and 0 <= cy < self.height):
            return None
        if self.data[cy, cx] < 0:
            return None
        span = int(math.ceil(window / self.resolution))
        x0, x1 = max(0, cx - span), min(self.width - 1, cx + span)
        y0, y1 = max(0, cy - span), min(self.height - 1, cy + span)
        patch = self.data[y0:y1 + 1, x0:x1 + 1]
        ys, xs = np.nonzero(patch >= occupied_threshold)
        if len(xs) == 0:
            return math.inf
        # Distance from the point to each occupied cell's full square.
        left = self.origin_x + (xs + x0) * self.resolution
        bottom = self.origin_y + (ys + y0) * self.resolution
        dx = np.maximum(np.maximum(left - x, x - (left + self.resolution)), 0.0)
        dy = np.maximum(np.maximum(bottom - y, y - (bottom + self.resolution)), 0.0)
        return float(np.min(np.hypot(dx, dy)))

    def clearance_field(self, occupied_threshold=65, supersample=3):
        """Approximate distance-to-obstacle over the whole grid, in metres.

        Exact Euclidean distance transform on a mask supersampled by
        `supersample`, so each occupied cell contributes its full square rather
        than its centre. Accurate to about resolution / supersample.
        """
        mask = np.repeat(
            np.repeat(self.data >= occupied_threshold, supersample, axis=0),
            supersample, axis=1,
        )
        if not mask.any():
            return np.full(self.data.shape, math.inf)
        big = _edt(mask) * (self.resolution / supersample)
        return big[supersample // 2::supersample, supersample // 2::supersample]


def _edt_1d(f):
    """Felzenszwalb & Huttenlocher 1-D squared distance transform."""
    n = len(f)
    d = np.empty(n)
    v = np.zeros(n, dtype=np.int64)
    z = np.empty(n + 1)
    k = 0
    z[0], z[1] = -np.inf, np.inf
    for q in range(1, n):
        s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
        while s <= z[k]:
            k -= 1
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = np.inf
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        d[q] = (q - v[k]) ** 2 + f[v[k]]
    return d


def _edt(mask):
    """Exact Euclidean distance (in pixels) from every pixel to a True pixel."""
    large = 1e12
    f = np.where(mask, 0.0, large)
    for row in range(f.shape[0]):
        f[row] = _edt_1d(f[row])
    for col in range(f.shape[1]):
        f[:, col] = _edt_1d(f[:, col])
    return np.sqrt(f)


class Canvas:
    def __init__(self, grid, scale, clearance=None, supersample=3, threshold=65):
        self.grid = grid
        self.scale = scale
        rgb = np.zeros((grid.height, grid.width, 3), dtype=np.uint8)
        rgb[...] = (238, 238, 234)                       # known free
        rgb[grid.data < 0] = (52, 58, 66)                # unknown: blocked
        if clearance:
            field = grid.clearance_field(threshold, supersample)
            band = (field < clearance) & (grid.data >= 0) & (grid.data < threshold)
            rgb[band] = (250, 214, 165)                  # inside the hard envelope
        rgb[grid.data >= threshold] = (20, 20, 20)       # occupied
        # Row 0 is the grid's minimum y, so flip for an image with +y upwards.
        self.image = Image.fromarray(rgb[::-1]).resize(
            (grid.width * scale, grid.height * scale), Image.NEAREST
        )
        self.draw = ImageDraw.Draw(self.image)

    def px(self, x, y):
        cx = (x - self.grid.origin_x) / self.grid.resolution
        cy = (y - self.grid.origin_y) / self.grid.resolution
        return (cx * self.scale, (self.grid.height - cy) * self.scale)

    def line(self, points, colour, width=2):
        if len(points) >= 2:
            self.draw.line([self.px(*p) for p in points], fill=colour, width=width)

    def dot(self, point, colour, radius=4, outline=None):
        x, y = self.px(*point)
        self.draw.ellipse(
            [x - radius, y - radius, x + radius, y + radius], fill=colour, outline=outline
        )

    def cross(self, point, colour, radius=7, width=3):
        x, y = self.px(*point)
        self.draw.line([x - radius, y - radius, x + radius, y + radius], fill=colour, width=width)
        self.draw.line([x - radius, y + radius, x + radius, y - radius], fill=colour, width=width)

    def text(self, xy, lines, colour=(20, 20, 20)):
        self.draw.multiline_text(xy, "\n".join(lines), fill=colour, spacing=3)

    def save(self, path):
        self.image.save(path)


def state_of(status):
    """The leading state token of a follower status line."""
    if status.startswith("CLEARANCE_ESCAPING"):
        return "CLEARANCE_ESCAPING"
    if status.startswith("CORRECTION_SETTLING"):
        return "CORRECTION_SETTLING"
    return status.split(" ")[0]


def at_or_before(series, when):
    chosen = None
    for stamp, value in series:
        if stamp <= when:
            chosen = value
        else:
            break
    return chosen


def path_points(msg):
    return [(p.pose.position.x, p.pose.position.y) for p in msg.poses]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", type=Path)
    parser.add_argument("-o", "--out", type=Path, default=None,
                        help="output directory (default: <bag>/render)")
    parser.add_argument("--clearance", type=float, default=0.25,
                        help="hard clearance to shade, metres (0 disables)")
    parser.add_argument("--occupied-threshold", type=int, default=65)
    parser.add_argument("--supersample", type=int, default=3,
                        help="clearance-band accuracy: resolution/supersample")
    parser.add_argument("--scale", type=int, default=0,
                        help="pixels per cell (0 = fit --width)")
    parser.add_argument("--width", type=int, default=1100, help="target image width")
    parser.add_argument("--at", type=float, nargs="*", default=[],
                        help="also render a frame at these bag times")
    parser.add_argument("--events", action="store_true",
                        help="render a frame at every fault the bag records")
    args = parser.parse_args(argv)

    data = read_bag(args.bag)
    grids = data["/rtabmap/grid"]
    if not grids:
        print("no /rtabmap/grid in this bag; nothing to render", file=sys.stderr)
        return 1
    poses = [(t, (m.pose.position.x, m.pose.position.y)) for t, m in data["/rtabmap/pose"]]
    follower = [(t, m.data) for t, m in data["/planner/follower/status"]]
    flight = [(t, m.data) for t, m in data["/planner/flight/status"]]
    if not poses:
        print("no /rtabmap/pose in this bag; nothing to draw", file=sys.stderr)
        return 1

    out = args.out or (args.bag / "render")
    out.mkdir(parents=True, exist_ok=True)

    # Every clearance question is answered against the grid that was current at
    # the time asked. Measuring a mid-flight event against the final map is a
    # different question and gives a different -- and flattering -- answer:
    # 20260829T085734Z reported start=0.227m at t=31.42s, which the final grid
    # scores 0.298m because later observations cleared the cell responsible.
    cache = {}

    def grid_at(when):
        index = 0
        for position, (stamp, _) in enumerate(grids):
            if stamp <= when:
                index = position
            else:
                break
        if index not in cache:
            cache[index] = Grid(grids[index][1])
        return cache[index]

    grid = Grid(grids[-1][1])
    cache[len(grids) - 1] = grid
    scale = args.scale or max(1, round(args.width / grid.width))
    clearance = args.clearance if args.clearance > 0 else None

    # --- events: everything that stopped or degraded the flight -------------
    events = []
    previous = None
    for t, status in follower:
        state = state_of(status)
        if state != previous and state not in ("FOLLOWING", "GOAL_REACHED"):
            events.append((t, "follower", status))
        previous = state
    previous = None
    for t, status in flight:
        head = status.split(";")[0]
        if head != previous and not head.startswith("ROUTE valid"):
            events.append((t, "adapter", status))
        previous = head
    events.sort()

    print(f"{args.bag.name}: {len(poses)} poses, {len(grids)} grids, "
          f"final grid {grid.width}x{grid.height} @ {grid.resolution:.3f}m")
    print(f"clearance band = {clearance}m (approx, +/-{grid.resolution / args.supersample:.3f}m); "
          f"printed clearances are exact")
    print(f"\n{'time':>7}  {'source':8} {'pose clearance':>14}  status")
    for t, source, status in events:
        pose = at_or_before(poses, t)
        measured = (
            grid_at(t).point_clearance(*pose, args.occupied_threshold) if pose else None)
        if measured is None:
            shown = "unknown"
        elif math.isinf(measured):
            shown = "no obstacle"
        else:
            shown = f"{measured:.3f} m"
        print(f"{t:7.2f}  {source:8} {shown:>14}  {status[:96]}")

    # --- exact clearance along the whole track ------------------------------
    measured = [
        (t, grid_at(t).point_clearance(x, y, args.occupied_threshold)) for t, (x, y) in poses]
    real = [c for _, c in measured if c is not None and math.isfinite(c)]
    unknown = sum(1 for _, c in measured if c is None)
    if real:
        below = sum(1 for c in real if clearance and c < clearance)
        print(f"\npose clearance against the grid current at each sample: "
              f"min={min(real):.3f}m median={sorted(real)[len(real) // 2]:.3f}m  "
              f"below {clearance}m: {below}/{len(real)}  in unknown space: {unknown}")

    def render(when, name):
        frame = grid_at(when)
        canvas = Canvas(frame, scale, clearance, args.supersample, args.occupied_threshold)
        # Accepted path as it stood at this instant.
        installed = at_or_before(data["/planner/path"], when)
        if installed is not None:
            canvas.line(path_points(installed), (0, 110, 220), width=max(2, scale // 3))
        # Track, coloured by follower state.
        for (t0, p0), (t1, p1) in zip(poses, poses[1:]):
            if t1 > when:
                break
            status = at_or_before(follower, t1) or ""
            colour = STATE_COLOURS.get(state_of(status), OTHER_COLOUR)
            canvas.line([p0, p1], colour, width=max(2, scale // 2))
        goal = at_or_before(data["/waypoint/clicked"], when)
        if goal is not None:
            canvas.dot((goal.point.x, goal.point.y), (255, 255, 255), radius=scale,
                       outline=(0, 0, 0))
        effective = at_or_before(data["/planner/effective_goal"], when)
        if effective is not None:
            canvas.dot((effective.point.x, effective.point.y), (255, 140, 0), radius=scale - 1,
                       outline=(0, 0, 0))
        here = at_or_before(poses, when)
        if here is not None:
            canvas.cross(here, (200, 0, 0), radius=scale + 3)
        status = at_or_before(follower, when) or ""
        adapter = at_or_before(flight, when) or ""
        exact = frame.point_clearance(*here, args.occupied_threshold) if here else None
        canvas.text((8, 8), [
            f"{args.bag.name}   t={when:.2f}s",
            f"pose clearance {('unknown' if exact is None else format(exact, '.3f') + ' m')}"
            f"   band={clearance} m",
            f"follower: {status[:110]}",
            f"adapter:  {adapter[:110]}",
        ])
        target = out / name
        canvas.save(target)
        return target

    end = poses[-1][0]
    print()
    print("wrote", render(end, "overview.png"))
    moments = list(args.at)
    if args.events:
        moments += [t for t, _, _ in events]
    for when in sorted(set(round(m, 2) for m in moments)):
        print("wrote", render(when, f"t{when:07.2f}.png".replace(".", "_", 1)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
