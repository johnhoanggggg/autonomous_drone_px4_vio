#!/usr/bin/env python3
"""Extract the two 2026-09-13 strict-limiter MCAPs for offline review.

The second recording has empty rosbag metadata and a truncated MCAP footer.
Reading the MCAP directly with SequentialReader recovers its complete records;
all series are sorted by receipt timestamp before they are written.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from analyze_yaw_tracking import read  # noqa: E402


TOPICS = [
    "/fmu/out/vehicle_local_position_v1",
    "/fmu/in/trajectory_setpoint",
    "/rtabmap/body_pose",
    "/rtabmap/body_vio_pose",
    "/rtabmap/odom_correction",
    "/planner/flight/status",
    "/planner/follower/status",
    "/planner/flight/config",
    "/planner/follower/config",
    "/planner/status",
    "/planner/map_generation",
    "/planner/path_map_generation",
    "/planner/path",
    "/planner/effective_goal",
    "/waypoint/clicked",
    "/rtabmap/grid",
]


def pose_row(stamp: int, message, t0: int) -> dict:
    pose = message.pose
    return {
        "t": (stamp - t0) / 1e9,
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "z": float(pose.position.z),
        "qx": float(pose.orientation.x),
        "qy": float(pose.orientation.y),
        "qz": float(pose.orientation.z),
        "qw": float(pose.orientation.w),
    }


def extract(bag_dir: Path) -> Path:
    mcaps = sorted(bag_dir.glob("*.mcap"))
    if len(mcaps) != 1:
        raise ValueError(f"expected one MCAP in {bag_dir}, found {len(mcaps)}")
    mcap = mcaps[0]
    data, _ = read(str(mcap), TOPICS)
    t0 = min(rows[0][0] for rows in data.values() if isinstance(rows, list) and rows)
    out = {
        "flight": bag_dir.name,
        "mcap": mcap.name,
        "mcap_bytes": mcap.stat().st_size,
        "mcap_sha256": hashlib.sha256(mcap.read_bytes()).hexdigest(),
        "t0_receipt_ns": t0,
        "decode_errors": data.get("_decode_errors", {}),
    }

    topic = "/fmu/out/vehicle_local_position_v1"
    out[topic] = []
    for stamp, message in data[topic]:
        out[topic].append({
            "t": (stamp - t0) / 1e9,
            "receipt_ns": stamp,
            "timestamp": int(message.timestamp),
            **{name: float(getattr(message, name)) for name in
               ("x", "y", "z", "vx", "vy", "ax", "ay", "heading")},
            "xy_reset_counter": int(message.xy_reset_counter),
            "vxy_reset_counter": int(message.vxy_reset_counter),
        })

    topic = "/fmu/in/trajectory_setpoint"
    out[topic] = []
    for stamp, message in data[topic]:
        out[topic].append({
            "t": (stamp - t0) / 1e9,
            "receipt_ns": stamp,
            "timestamp": int(message.timestamp),
            "position": list(map(float, message.position)),
            "velocity": list(map(float, message.velocity)),
            "acceleration": list(map(float, message.acceleration)),
            "yaw": float(message.yaw),
            "yawspeed": float(message.yawspeed),
        })

    for topic in ("/rtabmap/body_pose", "/rtabmap/body_vio_pose",
                  "/rtabmap/odom_correction"):
        out[topic] = [pose_row(stamp, message, t0) for stamp, message in data[topic]]

    for topic in ("/planner/flight/status", "/planner/follower/status",
                  "/planner/flight/config", "/planner/follower/config",
                  "/planner/status"):
        out[topic] = [
            {"t": (stamp - t0) / 1e9, "text": message.data}
            for stamp, message in data[topic]
        ]

    for topic in ("/planner/map_generation", "/planner/path_map_generation"):
        out[topic] = [
            {"t": (stamp - t0) / 1e9, "value": int(message.data)}
            for stamp, message in data[topic]
        ]

    out["/planner/path"] = [
        {
            "t": (stamp - t0) / 1e9,
            "frame_id": message.header.frame_id,
            "xy": [[float(p.pose.position.x), float(p.pose.position.y)] for p in message.poses],
        }
        for stamp, message in data["/planner/path"]
    ]
    for topic in ("/planner/effective_goal", "/waypoint/clicked"):
        out[topic] = [
            {"t": (stamp - t0) / 1e9,
             "x": float(message.point.x), "y": float(message.point.y),
             "z": float(message.point.z)}
            for stamp, message in data[topic]
        ]

    grids = data["/rtabmap/grid"]
    if grids:
        stamp, grid = grids[-1]
        out["final_grid"] = {
            "t": (stamp - t0) / 1e9,
            "data": list(grid.data),
            "width": int(grid.info.width),
            "height": int(grid.info.height),
            "resolution": float(grid.info.resolution),
            "origin_x": float(grid.info.origin.position.x),
            "origin_y": float(grid.info.origin.position.y),
        }
    out["topic_counts"] = {
        topic: len(rows) for topic, rows in data.items() if isinstance(rows, list)
    }
    target = bag_dir / "render" / "bag_data.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(out, indent=2, allow_nan=True) + "\n")
    return target


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+", type=Path)
    args = parser.parse_args()
    for bag in args.bags:
        print(extract(bag))


if __name__ == "__main__":
    main()
