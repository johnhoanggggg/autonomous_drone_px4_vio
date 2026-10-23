"""Decode the September 21 global damping comparison bags without replaying ROS."""

import json
from pathlib import Path
import sys

from rosidl_runtime_py.convert import message_to_ordereddict


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from analyze_yaw_tracking import read  # noqa: E402


TOPICS = (
    "/planner/flight/config",
    "/planner/flight/damping_debug",
    "/planner/flight/status",
    "/planner/flight/heading_debug",
    "/planner/follower/config",
    "/planner/follower/status",
    "/planner/follower/goal_reached",
    "/planner/follower/cross_track",
    "/planner/follower/remaining",
    "/planner/follower/valid",
    "/planner/follower/path_generation",
    "/planner/path",
    "/planner/candidate_path",
    "/planner/map_generation",
    "/planner/path_map_generation",
    "/planner/correction_epoch",
    "/planner/flight/path_offset",
    "/planner/effective_goal",
    "/waypoint/clicked",
    "/rtabmap/body_pose",
    "/rtabmap/body_vio_pose",
    "/rtabmap/odom_correction",
    "/rtabmap/grid",
    "/fmu/in/trajectory_setpoint",
    "/fmu/in/vehicle_command",
    "/fmu/out/vehicle_local_position_v1",
    "/fmu/out/vehicle_status_v1",
    "/fmu/out/vehicle_command_ack",
    "/perf/throttled",
    "/rosout",
)

FLIGHTS = ("111652", "111754", "111943", "112101")


for suffix in FLIGHTS:
    bag_dir = ROOT / "ros2_ws" / "flight_logs" / f"offboard_global_20260921T{suffix}Z"
    mcap = next(bag_dir.glob("*.mcap"))
    data, t0 = read(str(mcap), TOPICS)
    decoded = {
        topic: [
            [(stamp - t0) * 1e-9, message_to_ordereddict(message)]
            for stamp, message in rows
        ]
        for topic, rows in data.items()
        if not topic.startswith("_")
    }
    (Path(__file__).parent / f"{suffix}.json").write_text(
        json.dumps(decoded), encoding="utf-8"
    )
    (Path(__file__).parent / f"{suffix}_decode_errors.json").write_text(
        json.dumps(data["_decode_errors"], indent=2), encoding="utf-8"
    )
    duration = max((rows[-1][0] for rows in decoded.values() if rows), default=0.0)
    print(suffix, f"duration={duration:.3f}s", "decode_errors=", data["_decode_errors"])
