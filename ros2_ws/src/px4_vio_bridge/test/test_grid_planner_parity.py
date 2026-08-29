"""Cross-language parity: the C++ grid planner against the Python one.

The C++ planner is only eligible to replace the Python one in flight if it
produces the same costmap, the same start recovery, the same goal selection and
the same A* cells from the same map. This drives both implementations over
identical randomized occupancy grids -- open rooms, cluttered rooms, unknown
frontiers, and starts buried inside their own lethal inflation -- and requires
exact agreement on every cell it emits.

The A* timeout is disabled on both sides: a wall-clock cutoff would make the
comparison machine-dependent, and this test is about the search, not its clock.

Skipped when the replay binary has not been built.
"""
import json
import os
import random
import shutil
import subprocess

import pytest

from px4_vio_bridge.grid_planner import (
    GridMap,
    astar,
    classify_goal,
    closest_reachable_goal,
    grid_lethal_radius,
    inflate_occupancy,
    inflation_display_data,
    recover_start,
    simplify_path,
)

COST_TOLERANCE = 1.0e-9


def replay_binary():
    """Path to the built C++ replay harness, or None when it is absent."""
    found = shutil.which("grid_planner_replay")
    if found:
        return found
    # ament installs executables under lib/<package>, which is not on PATH.
    for root in filter(None, os.environ.get("AMENT_PREFIX_PATH", "").split(":")):
        path = os.path.join(root, "lib", "px4_vio_bridge", "grid_planner_replay")
        if os.path.exists(path):
            return path
    build = os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "build", "px4_vio_bridge",
        "grid_planner_replay",
    )
    return build if os.path.exists(build) else None


BINARY = replay_binary()
requires_binary = pytest.mark.skipif(
    BINARY is None, reason="grid_planner_replay is not built"
)


def random_scenario(seed):
    """A small map with obstacles, an unknown frontier, and a start and goal.

    Kept small (<= 40x40) so a full BFS/A* pair runs in milliseconds, but shaped
    like the flight maps: mostly unknown, a known free pocket, scattered
    obstacles, and a start that may well sit inside its own inflation.
    """
    rng = random.Random(seed)
    width = rng.randint(12, 40)
    height = rng.randint(12, 40)
    resolution = rng.choice((0.03, 0.05, 0.10))
    data = [-1] * (width * height)
    # Carve a known-free pocket, then drop obstacles into it.
    x0, y0 = rng.randint(0, width // 3), rng.randint(0, height // 3)
    x1 = rng.randint(x0 + max(4, width // 3), width)
    y1 = rng.randint(y0 + max(4, height // 3), height)
    for y in range(y0, min(y1, height)):
        for x in range(x0, min(x1, width)):
            data[y * width + x] = 0
    for _ in range(rng.randint(0, (x1 - x0) * (y1 - y0) // 8)):
        x = rng.randrange(x0, min(x1, width))
        y = rng.randrange(y0, min(y1, height))
        data[y * width + x] = rng.choice((70, 100))
    start = (rng.randrange(x0, min(x1, width)), rng.randrange(y0, min(y1, height)))
    origin_x = rng.uniform(-2.0, 2.0)
    origin_y = rng.uniform(-2.0, 2.0)
    goal_world = (
        origin_x + rng.uniform(0.0, width * resolution),
        origin_y + rng.uniform(0.0, height * resolution),
    )
    return {
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin_x": origin_x,
        "origin_y": origin_y,
        "data": data,
        "occupied_threshold": 65,
        "lethal_radius": rng.choice((0.10, 0.20, 0.30)),
        "inflation_radius": rng.choice((0.40, 0.50)),
        "cost_scaling": 3.0,
        "heuristic_weight": rng.choice((1.0, 1.2)),
        "cost_weight": rng.choice((0.0, 2.0)),
        "start_recovery_radius": rng.choice((0.0, 0.30, 0.50)),
        "start": list(start),
        "goal_world": list(goal_world),
    }


def python_result(scenario):
    """Run the Python pipeline the C++ replay harness mirrors."""
    grid = GridMap(
        scenario["width"], scenario["height"], scenario["resolution"],
        scenario["origin_x"], scenario["origin_y"], tuple(scenario["data"]),
    )
    threshold = scenario["occupied_threshold"]
    inflated = inflate_occupancy(
        grid,
        occupied_threshold=threshold,
        lethal_radius=grid_lethal_radius(scenario["lethal_radius"], grid.resolution),
        inflation_radius=grid_lethal_radius(scenario["inflation_radius"], grid.resolution),
        cost_scaling=scenario["cost_scaling"],
    )
    out = {
        "inflated": list(inflated.data),
        "display": list(inflation_display_data(
            grid, inflated, occupied_threshold=threshold)),
    }
    start = tuple(scenario["start"])
    recovery = recover_start(
        grid, inflated, start,
        occupied_threshold=threshold,
        max_radius=scenario["start_recovery_radius"],
    )
    if recovery is None:
        out.update(recovered=None, recovery_distance=None,
                   selection=None, result=None, simplified=None)
        return out
    out["recovered"] = list(recovery.cell)
    out["recovery_distance"] = recovery.distance
    selection = closest_reachable_goal(
        inflated, recovery.cell, tuple(scenario["goal_world"]))
    if selection is None:
        out.update(selection=None, result=None, simplified=None)
        return out
    out["selection"] = {
        "cell": list(selection.cell),
        "distance": selection.distance,
        "reachable_cells": selection.reachable_cells,
    }
    exact, terminal = classify_goal(
        grid, inflated, tuple(scenario["goal_world"]), selection.cell)
    out["goal_exact"] = exact
    out["goal_terminal"] = terminal
    result = astar(
        inflated, recovery.cell, selection.cell,
        heuristic_weight=scenario["heuristic_weight"],
        cost_weight=scenario["cost_weight"],
        timeout_ms=0.0,
    )
    out["result"] = {
        "cells": [list(cell) for cell in result.cells],
        "cost": result.cost if result.found else None,
        "expanded": result.expanded,
        "reason": result.reason,
    }
    if not result.found:
        out["simplified"] = None
        return out
    out["simplified"] = [
        list(cell)
        for cell in simplify_path(
            inflated, result.cells,
            preserve_cost=True,
            source_grid=grid,
            required_clearance=scenario["lethal_radius"],
            occupied_threshold=threshold,
        )
    ]
    return out


def cpp_result(scenario):
    completed = subprocess.run(
        [BINARY], input=json.dumps(scenario), capture_output=True,
        text=True, check=True, timeout=120,
    )
    return json.loads(completed.stdout)


@requires_binary
@pytest.mark.parametrize("seed", range(80))
def test_pipeline_matches_python(seed):
    scenario = random_scenario(seed)
    expected = python_result(scenario)
    actual = cpp_result(scenario)

    assert actual["inflated"] == expected["inflated"], "costmap differs"
    assert actual["display"] == expected["display"], "inflation display differs"
    assert actual["recovered"] == expected["recovered"], "start recovery differs"
    if expected["recovery_distance"] is None:
        assert actual["recovery_distance"] is None
    else:
        assert actual["recovery_distance"] == pytest.approx(
            expected["recovery_distance"], abs=COST_TOLERANCE)
    if expected["selection"] is None:
        assert actual["selection"] is None
        return
    assert actual["selection"]["cell"] == expected["selection"]["cell"]
    assert actual["selection"]["reachable_cells"] == expected["selection"]["reachable_cells"]
    assert actual["selection"]["distance"] == pytest.approx(
        expected["selection"]["distance"], abs=COST_TOLERANCE)
    assert actual["goal_exact"] == expected["goal_exact"]
    assert actual["goal_terminal"] == expected["goal_terminal"]
    assert actual["result"]["reason"] == expected["result"]["reason"]
    assert actual["result"]["cells"] == expected["result"]["cells"], "A* path differs"
    assert actual["result"]["expanded"] == expected["result"]["expanded"], "expansions differ"
    if expected["result"]["cost"] is not None:
        assert actual["result"]["cost"] == pytest.approx(
            expected["result"]["cost"], abs=COST_TOLERANCE)
    assert actual["simplified"] == expected["simplified"], "simplified path differs"
