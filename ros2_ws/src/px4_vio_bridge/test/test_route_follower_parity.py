"""Cross-language parity for the route-follower safety state machine."""

import json
import math
import os
import random
import shutil
import subprocess

import pytest

from px4_vio_bridge.path_follower import PositionRouteFollower


TOLERANCE = 1.0e-9
DEFAULT_CONFIG = {
    "lookahead": 0.60,
    "max_carrot_speed": 0.20,
    "max_carrot_acceleration": 0.30,
    "max_cross_track": 0.20,
    "cross_track_resume": 0.05,
    "cross_track_recovery_time": 0.30,
    "arrival_tolerance": 0.12,
    "arrival_release_tolerance": 0.20,
}


def replay_binary():
    found = shutil.which("route_follower_replay")
    if found:
        return found
    for root in filter(None, os.environ.get("AMENT_PREFIX_PATH", "").split(":")):
        candidate = os.path.join(
            root, "lib", "px4_vio_bridge", "route_follower_replay"
        )
        if os.path.exists(candidate):
            return candidate
    candidate = os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "build",
        "px4_vio_bridge", "route_follower_replay",
    )
    return candidate if os.path.exists(candidate) else None


BINARY = replay_binary()
requires_binary = pytest.mark.skipif(
    BINARY is None, reason="route_follower_replay is not built"
)


def result_dict(result):
    return {
        "status": result.status,
        "valid": result.valid,
        "desired_carrot": list(result.desired_carrot),
        "commanded_carrot": list(result.commanded_carrot),
        "commanded_displacement": list(result.commanded_displacement),
        "path_progress": result.path_progress,
        "progress": result.progress,
        "remaining": result.remaining,
        "cross_track": result.cross_track,
        "generation": result.generation,
    }


def state_dict(follower):
    return {
        "has_path": follower.path is not None,
        "generation": follower.generation,
        "progress": follower.progress,
        "path_progress": follower.path_progress,
        "commanded_displacement": list(follower.commanded_displacement),
        "command_velocity": list(follower.command_velocity),
        "cross_track_latched": follower.cross_track_latched,
        "at_goal": follower.at_goal,
    }


def run_python(scenario):
    follower = PositionRouteFollower(**scenario["follower"])
    output = []
    for step in scenario["steps"]:
        op = step["op"]
        item = {"op": op}
        try:
            if op == "set_path":
                item["changed"] = follower.set_path(
                    [tuple(point) for point in step["points"]], tuple(step["pose"])
                )
            elif op == "update":
                verdict = step.get("validator", True)
                item["result"] = result_dict(follower.update(
                    tuple(step["pose"]), step["dt"],
                    lookahead=step.get("lookahead"),
                    command_validator=lambda _point, value=verdict: value,
                ))
            elif op == "clear_path":
                follower.clear_path()
            elif op == "reset_route_progress":
                follower.reset_route_progress()
            elif op == "interrupt_cross_track_recovery":
                follower.interrupt_cross_track_recovery()
            elif op == "hold_command":
                follower.hold_command()
            else:
                raise ValueError(f"unknown op {op}")
            item["ok"] = True
        except (RuntimeError, ValueError) as error:
            item["ok"] = False
            item["error"] = str(error)
        item["state"] = state_dict(follower)
        output.append(item)
    return output


def run_cpp(scenario):
    completed = subprocess.run(
        [BINARY], input=json.dumps(scenario), capture_output=True, text=True,
        check=True, timeout=120,
    )
    return json.loads(completed.stdout)


def assert_value_same(wanted, actual, where="root"):
    if isinstance(wanted, bool) or wanted is None or isinstance(wanted, str):
        assert actual == wanted, where
    elif isinstance(wanted, (int, float)):
        assert actual == pytest.approx(wanted, abs=TOLERANCE), where
    elif isinstance(wanted, list):
        assert len(actual) == len(wanted), where
        for index, (left, right) in enumerate(zip(wanted, actual)):
            assert_value_same(left, right, f"{where}[{index}]")
    elif isinstance(wanted, dict):
        assert actual.keys() == wanted.keys(), where
        for key in wanted:
            assert_value_same(wanted[key], actual[key], f"{where}.{key}")
    else:
        raise TypeError(type(wanted))


def check(scenario):
    assert_value_same(run_python(scenario), run_cpp(scenario))


@requires_binary
def test_fault_recovery_replan_and_arrival_match():
    steps = [
        {"op": "set_path", "points": [[0.0, 0.0], [3.0, 0.0]], "pose": [0.0, 0.0]},
        {"op": "update", "pose": [0.5, 0.0], "dt": 0.1},
        {"op": "update", "pose": [1.0, 0.21], "dt": 0.1},
        {"op": "update", "pose": [1.0, 0.04], "dt": 0.1},
        {"op": "set_path", "points": [[1.0, 0.04], [3.0, 0.0]], "pose": [1.0, 0.04]},
        {"op": "update", "pose": [1.0, 0.04], "dt": 0.1},
        {"op": "interrupt_cross_track_recovery"},
        {"op": "update", "pose": [1.0, 0.04], "dt": 0.1},
        {"op": "update", "pose": [1.0, 0.04], "dt": 0.1},
        {"op": "update", "pose": [1.0, 0.04], "dt": 0.1},
        {"op": "set_path", "points": [[1.0, 0.04], [1.1, 0.04]], "pose": [1.0, 0.04]},
        {"op": "update", "pose": [1.0, 0.04], "dt": 0.1},
        {"op": "set_path", "points": [[1.0, 0.04], [1.19, 0.04]], "pose": [1.0, 0.04]},
        {"op": "update", "pose": [1.0, 0.04], "dt": 0.1},
    ]
    check({"follower": DEFAULT_CONFIG, "steps": steps})


@requires_binary
@pytest.mark.parametrize("seed", range(40))
def test_randomized_routes_match(seed):
    rng = random.Random(seed)
    steps = [{
        "op": "set_path",
        "points": [[0.0, 0.0], [1.0, 0.0], [2.0, 0.5]],
        "pose": [0.0, 0.0],
    }]
    pose = [0.0, 0.0]
    for index in range(80):
        if index and index % 17 == 0:
            points = [
                [pose[0], pose[1]],
                [pose[0] + rng.uniform(0.2, 0.8), pose[1] + rng.uniform(-0.2, 0.2)],
                [2.2, 0.5],
            ]
            steps.append({"op": "set_path", "points": points, "pose": list(pose)})
        pose[0] += rng.uniform(0.0, 0.04)
        pose[1] += rng.uniform(-0.025, 0.025)
        steps.append({
            "op": "update", "pose": list(pose), "dt": rng.uniform(0.06, 0.14),
            "lookahead": rng.choice((None, 0.2, 0.6)),
            "validator": rng.random() > 0.04,
        })
    check({"follower": DEFAULT_CONFIG, "steps": steps})
