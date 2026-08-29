"""Cross-language parity: the C++ command limiters against the Python ones.

The C++ flight adapter is only eligible to replace the Python one if it
produces the same commands from the same inputs. This drives both
implementations over identical randomized step sequences -- route entry,
replanning, corner stops, corner blending, yaw-alignment pauses and rejected
rejoins -- and requires the published command point to agree to 1e-9.

Skipped when the replay binary has not been built.
"""
import json
import math
import os
import random
import shutil
import subprocess

import pytest

from px4_vio_bridge.planner_flight import HorizontalCommandLimiter, PathCommandLimiter

TOLERANCE = 1.0e-9


def replay_binary():
    """Path to the built C++ replay harness, or None when it is absent."""
    for candidate in (
        os.path.join(os.environ.get("COLCON_PREFIX_PATH", ""), ""),
    ):
        del candidate
    found = shutil.which("planner_flight_replay")
    if found:
        return found
    # ament installs executables under lib/<package>, which is not on PATH.
    for root in filter(None, os.environ.get("AMENT_PREFIX_PATH", "").split(":")):
        path = os.path.join(root, "lib", "px4_vio_bridge", "planner_flight_replay")
        if os.path.exists(path):
            return path
    build = os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "build", "px4_vio_bridge",
        "planner_flight_replay",
    )
    return build if os.path.exists(build) else None


BINARY = replay_binary()
requires_binary = pytest.mark.skipif(
    BINARY is None, reason="planner_flight_replay is not built"
)

DEFAULT_CONFIG = {
    "max_speed": 0.20,
    "max_acceleration": 0.30,
    "max_projection_error": 0.05,
    "corner_tolerance": 0.05,
    "max_entry_error": 0.30,
    "max_connector_error": 0.20,
    "suffix_tolerance": 0.01,
    "corner_blending": False,
    "junction_deviation": 0.05,
}


def run_cpp(scenario):
    output = subprocess.run(
        [BINARY],
        input=json.dumps(scenario),
        capture_output=True,
        text=True,
        check=True,
        timeout=120,
    )
    return json.loads(output.stdout)


def run_python(scenario):
    config = scenario["limiter"]
    path_limiter = PathCommandLimiter(
        max_speed=config["max_speed"],
        max_acceleration=config["max_acceleration"],
        max_projection_error=config["max_projection_error"],
        corner_tolerance=config["corner_tolerance"],
        max_entry_error=config["max_entry_error"],
        max_connector_error=config["max_connector_error"],
        suffix_tolerance=config["suffix_tolerance"],
        corner_blending=config["corner_blending"],
        junction_deviation=config["junction_deviation"],
    )
    horizontal = HorizontalCommandLimiter(
        max_speed=config["max_speed"],
        max_acceleration=config["max_acceleration"],
    )
    clearance = scenario.get("clearance", True)
    results = []
    for step in scenario["steps"]:
        op = step["op"]
        result = {"op": op}
        try:
            if op == "set_path":
                result["changed"] = path_limiter.set_path(
                    [tuple(point) for point in step["points"]],
                    tuple(step["reference"]),
                    clearance_check=lambda start, end: clearance,
                )
            elif op == "update":
                reference = step.get("reference")
                path_limiter.update(
                    tuple(step["desired"]),
                    step["dt"],
                    advance=step.get("advance", True),
                    reference_point=tuple(reference) if reference else None,
                )
            elif op == "clear":
                path_limiter.clear()
            elif op == "horizontal_reset":
                horizontal.reset(tuple(step["position"]))
            elif op == "horizontal_update":
                horizontal.update(tuple(step["target"]), step["dt"])
            elif op == "horizontal_adopt":
                horizontal.adopt(tuple(step["position"]), tuple(step["velocity"]))
            else:
                raise ValueError(f"unknown op {op}")
            result["ok"] = True
        except (RuntimeError, ValueError) as exc:
            result["ok"] = False
            result["error"] = str(exc)
        result["path"] = {
            "position": list(path_limiter.position)
            if path_limiter.position is not None else None,
            "velocity": list(path_limiter.velocity),
            "waiting_vertex": path_limiter.waiting_vertex,
            "has_path": path_limiter.path is not None,
        }
        result["horizontal"] = {
            "position": list(horizontal.position)
            if horizontal.position is not None else None,
            "velocity": list(horizontal.velocity),
        }
        results.append(result)
    return results


def assert_same(expected, actual, label):
    assert len(expected) == len(actual), f"{label}: step count differs"
    for index, (want, got) in enumerate(zip(expected, actual)):
        where = f"{label} step {index} ({want['op']})"
        assert want["ok"] == got["ok"], (
            f"{where}: acceptance differs "
            f"(python={want.get('error', 'ok')} cpp={got.get('error', 'ok')})"
        )
        assert want.get("changed") == got.get("changed"), f"{where}: changed differs"
        for section in ("path", "horizontal"):
            wanted, given = want[section], got[section]
            if wanted["position"] is None or given["position"] is None:
                assert wanted["position"] == given["position"], f"{where}: {section} position"
            else:
                for axis in range(2):
                    assert abs(wanted["position"][axis] - given["position"][axis]) < TOLERANCE, (
                        f"{where}: {section} position[{axis}] "
                        f"{wanted['position'][axis]!r} != {given['position'][axis]!r}"
                    )
                    assert abs(wanted["velocity"][axis] - given["velocity"][axis]) < TOLERANCE, (
                        f"{where}: {section} velocity[{axis}] "
                        f"{wanted['velocity'][axis]!r} != {given['velocity'][axis]!r}"
                    )
        if want["path"]["waiting_vertex"] is None or got["path"]["waiting_vertex"] is None:
            assert want["path"]["waiting_vertex"] == got["path"]["waiting_vertex"], (
                f"{where}: waiting_vertex"
            )
        else:
            assert abs(
                want["path"]["waiting_vertex"] - got["path"]["waiting_vertex"]
            ) < TOLERANCE, f"{where}: waiting_vertex"
        assert want["path"]["has_path"] == got["path"]["has_path"], f"{where}: has_path"


def check(scenario, label):
    assert_same(run_python(scenario), run_cpp(scenario), label)


def straight_route(corner_blending=False):
    config = dict(DEFAULT_CONFIG, corner_blending=corner_blending)
    steps = [{"op": "set_path", "points": [[0.0, 0.0], [2.0, 0.0]],
              "reference": [0.0, 0.0]}]
    steps += [
        {"op": "update", "desired": [2.0, 0.0], "dt": 0.05,
         "advance": True, "reference": [0.0, 0.0]}
        for _ in range(120)
    ]
    return {"limiter": config, "steps": steps}


@requires_binary
def test_straight_route_matches():
    check(straight_route(), "straight route")


@requires_binary
@pytest.mark.parametrize("corner_blending", [False, True])
def test_corner_handling_matches(corner_blending):
    """The stop-at-every-vertex path and the junction-deviation blend."""
    config = dict(DEFAULT_CONFIG, corner_blending=corner_blending)
    steps = [{"op": "set_path",
              "points": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [2.0, 1.5]],
              "reference": [0.0, 0.0]}]
    # The reference walks the route, so corner stops are released the way the
    # vehicle releases them in flight.
    for index in range(300):
        travelled = min(2.0, index * 0.004)
        reference = [min(1.0, travelled), max(0.0, travelled - 1.0)]
        steps.append({"op": "update", "desired": [2.0, 1.5], "dt": 0.05,
                      "advance": True, "reference": reference})
    check({"limiter": config, "steps": steps}, f"corners blending={corner_blending}")


@requires_binary
def test_yaw_alignment_pause_matches():
    """advance=False must brake forward identically in both implementations."""
    steps = [{"op": "set_path", "points": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
              "reference": [0.0, 0.0]}]
    for index in range(200):
        steps.append({"op": "update", "desired": [1.0, 1.0], "dt": 0.05,
                      "advance": index % 40 >= 20, "reference": [0.5, 0.0]})
    check({"limiter": DEFAULT_CONFIG, "steps": steps}, "yaw alignment pause")


@requires_binary
@pytest.mark.parametrize("clearance", [True, False])
def test_replanning_matches(clearance):
    """Shared-suffix carry, in-band rejoin and the connector-clearance gate."""
    steps = [{"op": "set_path", "points": [[0.0, 0.0], [2.0, 0.0]],
              "reference": [0.0, 0.0]}]
    for index in range(60):
        steps.append({"op": "update", "desired": [2.0, 0.0], "dt": 0.05,
                      "advance": True, "reference": [0.2, 0.0]})
        if index % 10 == 9:
            # Rewrite the head only: the tail the command sits on survives.
            steps.append({"op": "set_path",
                          "points": [[-0.5, 0.0], [0.0, 0.0], [2.0, 0.0]],
                          "reference": [0.2, 0.0]})
        if index % 17 == 16:
            # An offset replacement: inside the connector band but outside the
            # projection band, so the clearance verdict decides it.
            steps.append({"op": "set_path",
                          "points": [[0.0, 0.12], [2.0, 0.12]],
                          "reference": [0.2, 0.0]})
    check({"limiter": DEFAULT_CONFIG, "steps": steps, "clearance": clearance},
          f"replanning clearance={clearance}")


@requires_binary
def test_route_entry_rejection_matches():
    """Entry beyond max_entry_error must be refused by both."""
    steps = [
        {"op": "set_path", "points": [[0.0, 0.0], [1.0, 0.0]], "reference": [0.0, 5.0]},
        {"op": "set_path", "points": [[0.0, 0.0], [1.0, 0.0]], "reference": [0.0, 0.1]},
    ]
    check({"limiter": DEFAULT_CONFIG, "steps": steps}, "route entry rejection")


@requires_binary
def test_horizontal_limiter_matches():
    steps = [{"op": "horizontal_reset", "position": [0.0, 0.0]}]
    for index in range(200):
        angle = index * 0.05
        steps.append({"op": "horizontal_update",
                      "target": [math.cos(angle), math.sin(angle)], "dt": 0.05})
    steps.append({"op": "horizontal_adopt", "position": [1.0, 2.0],
                  "velocity": [0.1, 0.05]})
    check({"limiter": DEFAULT_CONFIG, "steps": steps}, "horizontal limiter")


@requires_binary
def test_randomized_sequences_match():
    """Fuzz the whole step vocabulary against the Python implementation."""
    rng = random.Random(20260828)
    for trial in range(25):
        blending = rng.random() < 0.5
        config = dict(
            DEFAULT_CONFIG,
            corner_blending=blending,
            junction_deviation=round(rng.uniform(0.02, 0.10), 4),
            max_speed=round(rng.uniform(0.10, 0.40), 4),
            max_acceleration=round(rng.uniform(0.20, 0.60), 4),
        )

        def route():
            points = [[0.0, 0.0]]
            for _ in range(rng.randint(1, 5)):
                points.append([
                    round(points[-1][0] + rng.uniform(0.3, 1.2), 4),
                    round(points[-1][1] + rng.uniform(-0.8, 0.8), 4),
                ])
            return points

        steps = [{"op": "set_path", "points": route(), "reference": [0.0, 0.0]}]
        reference = [0.0, 0.0]
        for _ in range(rng.randint(50, 200)):
            choice = rng.random()
            if choice < 0.08:
                steps.append({"op": "set_path", "points": route(),
                              "reference": [round(v, 4) for v in reference]})
            elif choice < 0.10:
                steps.append({"op": "clear"})
            else:
                reference = [round(reference[0] + rng.uniform(-0.02, 0.06), 4),
                             round(reference[1] + rng.uniform(-0.03, 0.03), 4)]
                steps.append({
                    "op": "update",
                    "desired": [round(rng.uniform(-1.0, 4.0), 4),
                                round(rng.uniform(-2.0, 2.0), 4)],
                    "dt": round(rng.choice([0.02, 0.05, 0.1]), 4),
                    "advance": rng.random() < 0.8,
                    "reference": reference,
                })
        check({"limiter": config, "steps": steps,
               "clearance": rng.random() < 0.5}, f"random trial {trial}")
