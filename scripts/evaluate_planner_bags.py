#!/usr/bin/env python3
"""Replay global-planner geometry from MCAP bags and compare safety variants.

This is an offline evaluator: it neither publishes ROS messages nor changes the
flight stack.  It reconstructs A* plans from the recorded occupancy grids,
poses and goal, then compares:

* the current traversability-only path simplifier;
* a cost-aware simplifier that may not enter a higher-cost cell than the A*
  subpath it replaces;
* an absolute cost-band simplifier; and
* continuous segment-to-occupied-cell clearance validation.

It also re-evaluates the recorded follower samples with a clearance-adaptive
lookahead which backs the carrot toward the vehicle until the direct command
chord satisfies the requested clearance.
"""

from __future__ import annotations

import argparse
import bisect
from dataclasses import dataclass
import glob
import json
import math
from pathlib import Path
import statistics
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


REPO = Path(__file__).resolve().parents[1]
PACKAGE = REPO / "ros2_ws" / "src" / "px4_vio_bridge"
sys.path.insert(0, str(PACKAGE))

from px4_vio_bridge.grid_planner import (  # noqa: E402
    GridMap,
    LETHAL,
    astar,
    closest_reachable_goal,
    grid_lethal_radius,
    inflate_occupancy,
    path_length,
    recover_start,
    simplify_path,
    segment_has_clearance,
    traversable,
)
from px4_vio_bridge.path_follower import Polyline  # noqa: E402


TOPICS = (
    "/planner/candidate_path",
    "/planner/config",
    "/planner/follower/config",
    "/planner/follower/lookahead",
    "/planner/path",
    "/rtabmap/grid",
    "/rtabmap/pose",
    "/waypoint/clicked",
)


@dataclass(frozen=True)
class ObstacleMap:
    grid: GridMap
    boxes: tuple[tuple[float, float, float, float], ...]


@dataclass(frozen=True)
class PlanResult:
    baseline: tuple
    preserve: tuple
    band: tuple | None
    continuous: tuple | None
    raw: tuple
    recorded_match_error: float
    baseline_exact: bool
    band_exact: bool
    continuous_exact: bool


def resolve_bags(inputs):
    paths = []
    for value in inputs:
        matches = glob.glob(value)
        for match in matches or [value]:
            path = Path(match)
            if path.is_dir():
                paths.extend(sorted(path.glob("*.mcap")))
            elif path.suffix == ".mcap":
                paths.append(path)
    return sorted(dict.fromkeys(path.resolve() for path in paths))


def load_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = [topic for topic in TOPICS if topic in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    messages = {topic: [] for topic in wanted}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        try:
            message = deserialize_message(data, get_message(types[topic]))
        except Exception:
            continue
        messages[topic].append((timestamp, message))
    for entries in messages.values():
        entries.sort(key=lambda item: item[0])
    return messages


def latest(entries, timestamp):
    if not entries:
        return None
    index = bisect.bisect_right([item[0] for item in entries], timestamp) - 1
    return entries[index] if index >= 0 else None


def make_grid(message):
    info = message.info
    return GridMap(
        int(info.width),
        int(info.height),
        float(info.resolution),
        float(info.origin.position.x),
        float(info.origin.position.y),
        tuple(int(value) for value in message.data),
    )


def make_obstacles(message, occupied_threshold):
    grid = make_grid(message)
    resolution = grid.resolution
    boxes = []
    for index, value in enumerate(grid.data):
        if value < occupied_threshold:
            continue
        x = index % grid.width
        y = index // grid.width
        x0 = grid.origin_x + x * resolution
        y0 = grid.origin_y + y * resolution
        boxes.append((x0, x0 + resolution, y0, y0 + resolution))
    return ObstacleMap(grid, tuple(boxes))


def point_segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1.0e-18:
        return math.dist(point, start)
    fraction = max(
        0.0,
        min(
            1.0,
            ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
            / length_sq,
        ),
    )
    projection = (start[0] + fraction * dx, start[1] + fraction * dy)
    return math.dist(point, projection)


def orientation(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def on_segment(a, b, point, tolerance=1.0e-12):
    return (
        min(a[0], b[0]) - tolerance <= point[0] <= max(a[0], b[0]) + tolerance
        and min(a[1], b[1]) - tolerance <= point[1] <= max(a[1], b[1]) + tolerance
        and abs(orientation(a, b, point)) <= tolerance
    )


def segments_intersect(a, b, c, d):
    oa, ob = orientation(a, b, c), orientation(a, b, d)
    oc, od = orientation(c, d, a), orientation(c, d, b)
    if ((oa > 0 > ob) or (oa < 0 < ob)) and ((oc > 0 > od) or (oc < 0 < od)):
        return True
    return (
        (abs(oa) <= 1.0e-12 and on_segment(a, b, c))
        or (abs(ob) <= 1.0e-12 and on_segment(a, b, d))
        or (abs(oc) <= 1.0e-12 and on_segment(c, d, a))
        or (abs(od) <= 1.0e-12 and on_segment(c, d, b))
    )


def segment_segment_distance(a, b, c, d):
    if segments_intersect(a, b, c, d):
        return 0.0
    return min(
        point_segment_distance(a, c, d),
        point_segment_distance(b, c, d),
        point_segment_distance(c, a, b),
        point_segment_distance(d, a, b),
    )


def segment_box_distance(start, end, box):
    x0, x1, y0, y1 = box
    if (
        x0 <= start[0] <= x1
        and y0 <= start[1] <= y1
        or x0 <= end[0] <= x1
        and y0 <= end[1] <= y1
    ):
        return 0.0
    corners = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
    return min(
        segment_segment_distance(start, end, corners[index], corners[(index + 1) % 4])
        for index in range(4)
    )


def segment_clearance(obstacles, start, end, stop_below=-1.0):
    best = math.inf
    # Bounding-box rejection is substantial on the recorded 0.05 m maps.
    margin = best if math.isfinite(best) else 0.0
    seg_x0, seg_x1 = sorted((start[0], end[0]))
    seg_y0, seg_y1 = sorted((start[1], end[1]))
    for box in obstacles.boxes:
        x0, x1, y0, y1 = box
        if math.isfinite(best):
            margin = best
            if x1 < seg_x0 - margin or x0 > seg_x1 + margin:
                continue
            if y1 < seg_y0 - margin or y0 > seg_y1 + margin:
                continue
        distance = segment_box_distance(start, end, box)
        if distance < best:
            best = distance
            if best < stop_below:
                return best
    return best


def polyline_clearance(obstacles, points):
    if not points:
        return math.nan
    if len(points) == 1:
        return segment_clearance(obstacles, points[0], points[0])
    return min(
        segment_clearance(obstacles, start, end)
        for start, end in zip(points, points[1:])
    )


def sampled_line_cells(grid, start, end):
    """The cells and corner cells checked by the production line validator."""
    x0, y0 = start
    x1, y1 = end
    dx, dy = x1 - x0, y1 - y0
    steps = max(abs(dx), abs(dy)) * 2 + 1
    cells = []
    previous = start
    for index in range(steps + 1):
        fraction = index / max(1, steps)
        cell = (int(round(x0 + dx * fraction)), int(round(y0 + dy * fraction)))
        if cell not in cells:
            cells.append(cell)
        sx, sy = cell[0] - previous[0], cell[1] - previous[1]
        if sx and sy:
            for corner in ((previous[0] + sx, previous[1]), (previous[0], previous[1] + sy)):
                if corner not in cells:
                    cells.append(corner)
        previous = cell
    return tuple(cells)


def line_max_cost(grid, start, end):
    cells = sampled_line_cells(grid, start, end)
    if any(not traversable(grid, cell) for cell in cells):
        return LETHAL
    return max(grid.value(cell) for cell in cells)


def simplify_with_checks(
    inflated,
    cells,
    *,
    source_obstacles=None,
    required_clearance=None,
    absolute_cost_ceiling=None,
    preserve_subpath_cost=False,
):
    if len(cells) <= 2:
        result = tuple(cells)
        if absolute_cost_ceiling is not None and any(
            inflated.value(cell) > absolute_cost_ceiling for cell in result
        ):
            return None
        if source_obstacles is not None and required_clearance is not None:
            points = tuple(inflated.cell_center(cell) for cell in result)
            if polyline_clearance(source_obstacles, points) + 1.0e-9 < required_clearance:
                return None
        return result

    simplified = [cells[0]]
    anchor = 0
    while anchor < len(cells) - 1:
        candidate = len(cells) - 1
        accepted = None
        while candidate > anchor:
            shortcut_cost = line_max_cost(inflated, cells[anchor], cells[candidate])
            valid = shortcut_cost < LETHAL
            if valid and absolute_cost_ceiling is not None:
                valid = shortcut_cost <= absolute_cost_ceiling
            if valid and preserve_subpath_cost:
                # Include the side cells checked for each diagonal transition,
                # not just the A* path cells. Otherwise even an adjacent valid
                # A* step can appear to increase cost and no result is possible.
                subpath_max = max(
                    line_max_cost(inflated, cells[index], cells[index + 1])
                    for index in range(anchor, candidate)
                )
                valid = shortcut_cost <= subpath_max
            if valid and source_obstacles is not None and required_clearance is not None:
                start = inflated.cell_center(cells[anchor])
                end = inflated.cell_center(cells[candidate])
                valid = (
                    segment_clearance(
                        source_obstacles, start, end, stop_below=required_clearance
                    )
                    + 1.0e-9
                    >= required_clearance
                )
            if valid:
                accepted = candidate
                break
            candidate -= 1
        if accepted is None:
            return None
        simplified.append(cells[accepted])
        anchor = accepted
    return tuple(simplified)


def cost_ceiling(config, clearance):
    lethal = float(config["lethal_radius"])
    inflation = float(config["inflation_radius"])
    scaling = float(config["inflation_cost_scaling"])
    if clearance <= lethal:
        return LETHAL - 1
    if clearance >= inflation:
        return 0
    span = max(1.0e-9, inflation - lethal)
    decay = math.exp(-scaling * (clearance - lethal) / span)
    return max(0, min(LETHAL - 1, int(math.floor((LETHAL - 1) * decay))))


def points_from_path(message):
    return tuple((float(p.pose.position.x), float(p.pose.position.y)) for p in message.poses)


def path_match_error(first, second):
    if not first or not second or len(first) != len(second):
        return math.inf
    return max(math.dist(a, b) for a, b in zip(first, second))


def quantiles(values):
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return (math.nan, math.nan, math.nan)
    return (
        values[0],
        values[len(values) // 20],
        statistics.median(values),
    )


def evaluate_plan_events(messages, config, band_clearance, continuous_clearance):
    plans = []
    map_cache = {}
    plan_cache = {}
    ceiling = cost_ceiling(config, band_clearance)
    safe_map_cache = {}
    for timestamp, recorded_message in messages.get("/planner/candidate_path", []):
        if len(recorded_message.poses) < 2:
            continue
        map_item = latest(messages.get("/rtabmap/grid", []), timestamp)
        pose_item = latest(messages.get("/rtabmap/pose", []), timestamp)
        goal_item = latest(messages.get("/waypoint/clicked", []), timestamp)
        if map_item is None or pose_item is None or goal_item is None:
            continue
        map_timestamp, map_message = map_item
        pose = pose_item[1].pose.position
        goal = goal_item[1].point
        obstacles = map_cache.get(map_timestamp)
        if obstacles is None:
            obstacles = make_obstacles(
                map_message, int(config["occupied_threshold"])
            )
            map_cache[map_timestamp] = obstacles
        source = obstacles.grid
        inflated = inflate_occupancy(
            source,
            occupied_threshold=int(config["occupied_threshold"]),
            lethal_radius=float(config["lethal_radius"]),
            inflation_radius=float(config["inflation_radius"]),
            cost_scaling=float(config["inflation_cost_scaling"]),
        )
        start = inflated.world_to_cell((pose.x, pose.y))
        if start is None:
            continue
        if not traversable(inflated, start):
            recovered = recover_start(
                source,
                inflated,
                start,
                occupied_threshold=int(config["occupied_threshold"]),
                max_radius=float(config["start_recovery_radius"]),
            )
            if recovered is None:
                continue
            start = recovered.cell
        selection = closest_reachable_goal(inflated, start, (goal.x, goal.y))
        if selection is None:
            continue
        key = ("baseline", map_timestamp, start, selection.cell)
        raw = plan_cache.get(key)
        if raw is None:
            search = astar(
                inflated,
                start,
                selection.cell,
                heuristic_weight=float(config["heuristic_weight"]),
                cost_weight=float(config["cost_weight"]),
                timeout_ms=0.0,
            )
            raw = search.cells
            plan_cache[key] = raw
        if len(raw) < 2:
            continue
        baseline_cells = simplify_path(inflated, raw)
        preserve_cells = simplify_with_checks(
            inflated, raw, preserve_subpath_cost=True
        )
        baseline_requested = inflated.world_to_cell((goal.x, goal.y))
        baseline_exact = baseline_requested == selection.cell

        def clearance_plan(required):
            # Production inflation measures obstacle-cell centre to path-cell
            # centre. Continuous validation measures from the occupied square's
            # edge, so add the cell half-diagonal before searching. Without this
            # correction a nominal 0.40 m grid envelope is only about 0.365 m
            # at the corner of a 0.05 m occupied cell.
            adjusted_lethal = grid_lethal_radius(required, source.resolution)
            extra = max(
                0.0,
                float(config["inflation_radius"]) - float(config["lethal_radius"]),
            )
            safe_key = (map_timestamp, round(required, 6))
            safe_inflated = safe_map_cache.get(safe_key)
            if safe_inflated is None:
                safe_inflated = inflate_occupancy(
                    source,
                    occupied_threshold=int(config["occupied_threshold"]),
                    lethal_radius=adjusted_lethal,
                    inflation_radius=adjusted_lethal + extra,
                    cost_scaling=float(config["inflation_cost_scaling"]),
                )
                safe_map_cache[safe_key] = safe_inflated
            safe_start = safe_inflated.world_to_cell((pose.x, pose.y))
            if safe_start is None:
                return None, False
            if not traversable(safe_inflated, safe_start):
                recovered = recover_start(
                    source,
                    safe_inflated,
                    safe_start,
                    occupied_threshold=int(config["occupied_threshold"]),
                    max_radius=float(config["start_recovery_radius"]),
                )
                if recovered is None:
                    return None, False
                safe_start = recovered.cell
            safe_selection = closest_reachable_goal(
                safe_inflated, safe_start, (goal.x, goal.y)
            )
            if safe_selection is None:
                return None, False
            safe_requested = safe_inflated.world_to_cell((goal.x, goal.y))
            safe_exact = safe_requested == safe_selection.cell
            safe_plan_key = (
                "clearance",
                round(required, 6),
                map_timestamp,
                safe_start,
                safe_selection.cell,
            )
            safe_raw = plan_cache.get(safe_plan_key)
            if safe_raw is None:
                search = astar(
                    safe_inflated,
                    safe_start,
                    safe_selection.cell,
                    heuristic_weight=float(config["heuristic_weight"]),
                    cost_weight=float(config["cost_weight"]),
                    timeout_ms=0.0,
                )
                safe_raw = search.cells
                plan_cache[safe_plan_key] = safe_raw
            if len(safe_raw) < 2:
                return None, safe_exact
            cells = simplify_path(
                safe_inflated,
                safe_raw,
                preserve_cost=True,
                source_grid=source,
                required_clearance=required,
                occupied_threshold=int(config["occupied_threshold"]),
            )
            if not cells:
                return None, safe_exact
            return (
                tuple(safe_inflated.cell_center(cell) for cell in cells),
                safe_exact,
            )

        # The band variant makes the desired 0.50 m band a hard constraint and
        # replans around it. The continuous variant uses the current 0.40 m
        # requirement but corrects centre-vs-cell-edge geometry.
        band_points, band_exact = clearance_plan(band_clearance)
        continuous_points, continuous_exact = clearance_plan(continuous_clearance)

        def world(cells):
            return None if cells is None else tuple(inflated.cell_center(cell) for cell in cells)

        baseline = world(baseline_cells)
        recorded = points_from_path(recorded_message)
        plans.append((
            PlanResult(
                baseline=baseline,
                preserve=world(preserve_cells),
                band=band_points,
                continuous=continuous_points,
                raw=world(raw),
                recorded_match_error=path_match_error(baseline, recorded),
                baseline_exact=baseline_exact,
                band_exact=band_exact,
                continuous_exact=continuous_exact,
            ),
            obstacles,
        ))
    return plans, ceiling


def evaluate_lookahead(messages, follower_config, required_clearance):
    base = float(follower_config["lookahead"])
    samples = []
    map_cache = {}
    for timestamp, lookahead_message in messages.get("/planner/follower/lookahead", []):
        path_item = latest(messages.get("/planner/path", []), timestamp)
        pose_item = latest(messages.get("/rtabmap/pose", []), timestamp)
        map_item = latest(messages.get("/rtabmap/grid", []), timestamp)
        if path_item is None or pose_item is None or map_item is None:
            continue
        points = points_from_path(path_item[1])
        if len(points) < 2:
            continue
        pose_message = pose_item[1].pose.position
        pose = (float(pose_message.x), float(pose_message.y))
        obstacles = map_cache.get(map_item[0])
        if obstacles is None:
            obstacles = make_obstacles(map_item[1], 65)
            map_cache[map_item[0]] = obstacles
        recorded_target = (
            float(lookahead_message.pose.position.x),
            float(lookahead_message.pose.position.y),
        )
        recorded_clearance = segment_clearance(obstacles, pose, recorded_target)
        pose_clearance = segment_clearance(obstacles, pose, pose)
        polyline = Polyline(points)
        projection = polyline.project(pose)
        chosen = None
        chosen_clearance = -math.inf
        trial = base
        while trial >= -1.0e-12:
            target = polyline.point_at(projection.along + max(0.0, trial))
            if segment_has_clearance(
                obstacles.grid,
                pose,
                target,
                required_clearance,
            ):
                chosen = max(0.0, trial)
                chosen_clearance = segment_clearance(obstacles, pose, target)
                break
            trial -= 0.05
        samples.append((recorded_clearance, chosen, chosen_clearance, pose_clearance))
    return samples


def evaluate_nominal_lookahead(plans, lookahead, required_clearance):
    """Test corner cutting assuming the vehicle lies exactly on each safe path."""
    samples = []
    for result, obstacles in plans:
        if result.continuous is None or len(result.continuous) < 2:
            continue
        polyline = Polyline(result.continuous)
        distance = 0.0
        while distance <= polyline.length + 1.0e-9:
            pose = polyline.point_at(distance)
            target = polyline.point_at(distance + lookahead)
            baseline_clearance = segment_clearance(obstacles, pose, target)
            chosen = lookahead
            while chosen >= -1.0e-12:
                candidate = polyline.point_at(distance + max(0.0, chosen))
                if segment_has_clearance(
                    obstacles.grid,
                    pose,
                    candidate,
                    required_clearance,
                ):
                    break
                chosen -= 0.05
            samples.append((baseline_clearance, max(0.0, chosen)))
            distance += 0.05
    return samples


def summarize_bag(path, messages, band_clearance, continuous_clearance):
    if not messages.get("/planner/config") or not messages.get("/waypoint/clicked"):
        print(f"{path.name}: skipped (no autonomous goal/config)")
        return None
    config = json.loads(messages["/planner/config"][-1][1].data)
    follower_config = json.loads(messages["/planner/follower/config"][-1][1].data)
    plans, ceiling = evaluate_plan_events(
        messages, config, band_clearance, continuous_clearance
    )
    lookahead = evaluate_lookahead(messages, follower_config, continuous_clearance)
    if not plans:
        print(f"{path.name}: skipped (no reconstructable paths)")
        return None

    variants = {
        "baseline": [(result.baseline, obstacles) for result, obstacles in plans],
        "cost-preserving": [(result.preserve, obstacles) for result, obstacles in plans],
        f"cost-band-{band_clearance:.2f}": [(result.band, obstacles) for result, obstacles in plans],
        f"continuous-{continuous_clearance:.2f}": [(result.continuous, obstacles) for result, obstacles in plans],
    }
    rows = {}
    for name, entries in variants.items():
        valid = [(points, obstacles) for points, obstacles in entries if points is not None]
        clearances = [polyline_clearance(obstacles, points) for points, obstacles in valid]
        lengths = [path_length(points) for points, _ in valid]
        points_count = [len(points) for points, _ in valid]
        rows[name] = {
            "valid": len(valid),
            "clearance": quantiles(clearances),
            "length": statistics.mean(lengths) if lengths else math.nan,
            "points": statistics.mean(points_count) if points_count else math.nan,
        }
    rows["baseline"]["exact"] = sum(result.baseline_exact for result, _ in plans)
    rows["cost-preserving"]["exact"] = rows["baseline"]["exact"]
    rows[f"cost-band-{band_clearance:.2f}"]["exact"] = sum(
        result.band is not None and result.band_exact for result, _ in plans
    )
    rows[f"continuous-{continuous_clearance:.2f}"]["exact"] = sum(
        result.continuous is not None and result.continuous_exact for result, _ in plans
    )

    match = [result.recorded_match_error for result, _ in plans]
    look_recorded = [item[0] for item in lookahead]
    look_valid = [item for item in lookahead if item[1] is not None]
    look_reduced = [item for item in look_valid if item[1] < float(follower_config["lookahead"]) - 1.0e-9]
    chosen_values = [item[1] for item in look_valid]
    pose_unsafe = sum(
        item[3] + 1.0e-9 < continuous_clearance for item in lookahead
    )
    nominal = evaluate_nominal_lookahead(
        plans, float(follower_config["lookahead"]), continuous_clearance
    )
    nominal_violations = sum(
        clearance + 1.0e-9 < continuous_clearance for clearance, _ in nominal
    )
    nominal_reduced = sum(
        chosen < float(follower_config["lookahead"]) - 1.0e-9
        for _, chosen in nominal
    )

    print(f"\n{path.parent.name}")
    print(
        f"  reconstructed plans={len(plans)}  production-match<=1cm="
        f"{sum(value <= 0.01 for value in match)}/{len(match)}  "
        f"cost ceiling={ceiling}"
    )
    for name, values in rows.items():
        minimum, p05, median = values["clearance"]
        print(
            f"  {name:17s} valid={values['valid']:3d}/{len(plans):3d} "
            f"exact={values['exact']:3d}/{len(plans):3d} "
            f"clearance min/p05/med={minimum:.3f}/{p05:.3f}/{median:.3f}m "
            f"mean length={values['length']:.3f}m points={values['points']:.1f}"
        )
    look_min, look_p05, look_med = quantiles(look_recorded)
    print(
        f"  recorded lookahead chords: n={len(lookahead)} "
        f"clearance min/p05/med={look_min:.3f}/{look_p05:.3f}/{look_med:.3f}m"
    )
    print(
        f"  adaptive lookahead: safe={len(look_valid)}/{len(lookahead)} "
        f"reduced={len(look_reduced)} holds={len(lookahead)-len(look_valid)} "
        f"pose-already-unsafe={pose_unsafe} "
        f"chosen median/min="
        f"{statistics.median(chosen_values) if chosen_values else math.nan:.2f}/"
        f"{min(chosen_values) if chosen_values else math.nan:.2f}m"
    )
    print(
        f"  on corrected paths: fixed-lookahead violations={nominal_violations}/"
        f"{len(nominal)} adaptive reductions={nominal_reduced}"
    )
    return {"plans": len(plans), "rows": rows, "lookahead": lookahead}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", help="MCAP files, bag directories, or globs")
    parser.add_argument(
        "--cost-band-clearance",
        type=float,
        default=0.50,
        help="grid-cost clearance band the simplifier may not enter (default: 0.50 m)",
    )
    parser.add_argument(
        "--continuous-clearance",
        type=float,
        default=0.40,
        help="exact segment-to-occupied-cell clearance (default: 0.40 m)",
    )
    args = parser.parse_args()
    bags = resolve_bags(args.bags)
    if not bags:
        parser.error("no MCAP bags found")
    print(
        f"Offline planner evaluation: cost band={args.cost_band_clearance:.2f}m, "
        f"continuous clearance={args.continuous_clearance:.2f}m"
    )
    summaries = []
    for path in bags:
        messages = load_bag(path)
        result = summarize_bag(
            path, messages, args.cost_band_clearance, args.continuous_clearance
        )
        if result is not None:
            summaries.append(result)
    if not summaries:
        raise SystemExit("no usable autonomous bags")


if __name__ == "__main__":
    main()
