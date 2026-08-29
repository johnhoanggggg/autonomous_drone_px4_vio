"""Generation-aware replay checks for 3D paths and follower chords."""

import json
import os
import random
import shutil
import subprocess

import pytest


def replay_binary():
    found = shutil.which("planner_3d_replay")
    if found:
        return found
    for root in filter(None, os.environ.get("AMENT_PREFIX_PATH", "").split(":")):
        path = os.path.join(root, "lib", "px4_vio_bridge", "planner_3d_replay")
        if os.path.exists(path):
            return path
    build = os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "build",
        "px4_vio_bridge", "planner_3d_replay",
    )
    return build if os.path.exists(build) else None


BINARY = replay_binary()
requires_binary = pytest.mark.skipif(BINARY is None, reason="planner_3d_replay is not built")


def scenario(seed=0):
    width = height = depth = 14
    data = [0] * (width * height * depth)
    rng = random.Random(seed)
    # Deterministic clutter outside the protected straight corridor. This
    # exercises inflation/search ordering without making the expected route
    # depend on wall-clock timing.
    candidates = [
        (x, y, z)
        for z in range(1, depth - 1)
        for y in range(1, height - 1)
        for x in range(1, width - 1)
        if abs(y - 7) >= 4 or abs(z - 7) >= 4
    ]
    for x, y, z in rng.sample(candidates, 80):
        data[(z * height + y) * width + x] = 100
    return {
        "generation": 17,
        "width": width,
        "height": height,
        "depth": depth,
        "resolution": 0.1,
        "origin": [0.0, 0.0, 0.0],
        "data": data,
        "start": [0.25, 0.75, 0.75],
        "goal": [1.15, 0.75, 0.75],
        "config": {
            "lethal_radius": 0.04,
            "inflation_radius": 0.14,
            "timeout_ms": 1000.0,
            "start_recovery_radius": 0.2,
        },
    }


def run_replay(record):
    completed = subprocess.run(
        [BINARY], input=json.dumps(record), capture_output=True, text=True, timeout=30,
    )
    return completed.returncode, json.loads(completed.stdout)


@requires_binary
@pytest.mark.parametrize("seed", range(16))
def test_randomized_replay_is_deterministic_and_clear(seed):
    record = scenario(seed)
    first_code, first = run_replay(record)
    second_code, second = run_replay(record)
    assert first_code == second_code == 0
    assert first == second
    assert first["valid"]
    assert first["found"]
    assert first["reason"] == "PATH_VALID"
    assert first["checked_path_segments"] >= 1
    assert first["violations"] == []


@requires_binary
def test_accepts_recorded_path_and_chord_from_exact_generation():
    record = scenario()
    record["path_map_generation"] = 17
    record["accepted_path"] = [record["start"], record["goal"]]
    record["follower_chords"] = [{
        "map_generation": 17,
        "start": record["start"],
        "end": [0.35, 0.75, 0.75],
    }]
    code, result = run_replay(record)
    assert code == 0
    assert result["valid"]
    assert result["checked_follower_chords"] == 1


@requires_binary
@pytest.mark.parametrize(
    "field,expected",
    [
        ("path", "PATH_GENERATION_MISMATCH"),
        ("chord", "CHORD_GENERATION_MISMATCH"),
    ],
)
def test_rejects_generation_mismatch(field, expected):
    record = scenario()
    if field == "path":
        record["path_map_generation"] = 16
        record["accepted_path"] = [record["start"], record["goal"]]
    else:
        record["follower_chords"] = [{
            "map_generation": 16,
            "start": record["start"],
            "end": record["goal"],
        }]
    code, result = run_replay(record)
    assert code == 1
    assert not result["valid"]
    assert expected in {item["kind"] for item in result["violations"]}


@requires_binary
def test_rejects_unknown_voxel_in_recorded_follower_chord():
    record = scenario()
    x, y, z = 6, 7, 7
    record["data"][(z * record["height"] + y) * record["width"] + x] = -1
    record["follower_chords"] = [{
        "map_generation": 17,
        "start": record["start"],
        "end": record["goal"],
    }]
    code, result = run_replay(record)
    assert code == 1
    assert not result["valid"]
    assert "FOLLOWER_CHORD_CLEARANCE" in {
        item["kind"] for item in result["violations"]
    }
