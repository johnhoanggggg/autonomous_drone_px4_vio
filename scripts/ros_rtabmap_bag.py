#!/usr/bin/env python3
"""Record OAK parity inputs and baseline outputs, or replay only sensor inputs.

Source the v3 message and project workspaces first. For replay, start:
  ros2 launch px4_vio_bridge ros_rtabmap_oak_v3_tracked.launch.py \
    start_camera:=false use_sim_time:=true foxglove:=false
Then run this script's play command. Restart the estimator between replays.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from types import SimpleNamespace

import yaml


INPUT_TOPICS = (
    "/oak/left/image_rect",
    "/oak/stereo/image_raw",
    "/oak/left/camera_info",
    "/oak/left_rect_feature_tracker/tracked_features",
    "/oak/imu/data",
    "/tf_static",
)
BASELINE_TOPICS = (
    "/tf", "/rtabmap/vio_odometry", "/rtabmap/vio_odom_info",
    "/rtabmap/info", "/rtabmap/oak_feature_count", "/diagnostics",
)
REPO = Path(__file__).resolve().parents[1]
QOS_PATH = REPO / "ros2_ws/src/px4_vio_bridge/config/oak_parity_bag_qos.yaml"


def record_command(bag):
    return [
        "ros2", "bag", "record", "--output", str(bag), "--storage", "mcap",
        "--disable-keyboard-controls", "--qos-profile-overrides-path", str(QOS_PATH),
        "--topics", *INPUT_TOPICS, *BASELINE_TOPICS,
    ]


def play_command(bag, rate):
    # Explicit allowlist: replaying old odometry or /tf would mix old estimator
    # results with new ones. /tf_static contains the OAK's calibrated geometry.
    return [
        "ros2", "bag", "play", str(bag), "--rate", str(rate),
        "--clock", "200", "--delay", "3", "--disable-keyboard-controls",
        "--qos-profile-overrides-path", str(QOS_PATH),
        "--topics", *INPUT_TOPICS,
    ]


def inspect_bag(bag):
    with (Path(bag) / "metadata.yaml").open() as stream:
        metadata = yaml.safe_load(stream)["rosbag2_bagfile_information"]
    counts = {
        entry["topic_metadata"]["name"]: entry["message_count"]
        for entry in metadata["topics_with_message_count"]
    }
    missing = [topic for topic in INPUT_TOPICS if counts.get(topic, 0) == 0]
    return {
        "duration_s": metadata["duration"]["nanoseconds"] / 1e9,
        "message_counts": counts,
        "missing_inputs": missing,
        "all_inputs_recorded": not missing,
        "note": "Topic presence is required, but does not prove lossless capture or motion parity.",
    }


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


def save_parameters(bag):
    """Retain the effective live settings, including launch overrides."""
    results = {}
    for node in ("/oak", "/rtabmap/oak_tracked_odometry", "/rtabmap/rtabmap"):
        filename = node.strip("/").replace("/", "_") + "_parameters.yaml"
        try:
            result = subprocess.run(
                ["ros2", "param", "dump", node, "--no-daemon", "--timeout", "2"],
                capture_output=True, text=True, timeout=8,
            )
            if result.returncode == 0:
                (bag / filename).write_text(result.stdout)
                results[node] = filename
            else:
                results[node] = {"error": result.stderr.strip() or result.stdout.strip()}
        except subprocess.TimeoutExpired:
            results[node] = {"error": "parameter snapshot timed out"}
    return results


def report_bag(bag, hold, max_hold_span):
    # Lazy imports keep inspect/command construction usable without ROS Python.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from loop_closure_test import report

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag)), rosbag2_py.ConverterOptions("", ""))
    topics = ("/rtabmap/vio_odometry", "/rtabmap/vio_odom_info", "/rtabmap/info")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
             if t.name in topics}
    node = SimpleNamespace(raw=[], raw_odom_stamps=[], lost_odom_stamps=set(),
                           corrected=[], corrections=[], closures=[], odom=[])
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg = deserialize_message(data, types[topic])
        t = timestamp / 1e9
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if topic == topics[0]:
            p = msg.pose.pose.position
            node.raw.append((t, p.x, p.y, p.z))
            node.raw_odom_stamps.append(stamp)
        elif topic == topics[1]:
            node.odom.append((t, msg.lost, msg.inliers, msg.features))
            if msg.lost:
                node.lost_odom_stamps.add(stamp)
        elif msg.loop_closure_id or msg.proximity_detection_id:
            p = msg.loop_closure_transform.translation
            node.closures.append((
                t, msg.ref_id, msg.loop_closure_id or msg.proximity_detection_id,
                math.sqrt(p.x*p.x + p.y*p.y + p.z*p.z),
                "global_loop_closure" if msg.loop_closure_id else "proximity_detection"))
    with (Path(bag) / "metadata.yaml").open() as stream:
        metadata = yaml.safe_load(stream)["rosbag2_bagfile_information"]
    start = metadata["starting_time"]["nanoseconds_since_epoch"] / 1e9
    end = start + metadata["duration"]["nanoseconds"] / 1e9
    return {"bag": str(bag), "time_basis": "bag receive timestamps",
            **report(node, hold, start, end, max_hold_span)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    record = commands.add_parser("record", help="Record from an already running tracked launch")
    record.add_argument("bag", type=Path)
    record.add_argument("--seconds", type=float, default=90)
    play = commands.add_parser("play", help="Replay inputs into an already running replay launch")
    play.add_argument("bag", type=Path)
    play.add_argument("--rate", type=float, default=1.0)
    inspect = commands.add_parser("inspect", help="Check that required inputs were recorded")
    inspect.add_argument("bag", type=Path)
    report = commands.add_parser("report", help="Measure recorded odometry using bag timestamps")
    report.add_argument("bag", type=Path)
    report.add_argument("--hold-seconds", type=float, default=10.0)
    report.add_argument("--max-hold-span", type=float, default=0.05)
    report.add_argument("--out", type=Path)
    args = parser.parse_args()
    if args.command == "inspect":
        report = inspect_bag(args.bag)
        print(json.dumps(report, indent=2))
        return 0 if report["all_inputs_recorded"] else 2
    if args.command == "report":
        if any(not math.isfinite(v) or v <= 0 for v in (args.hold_seconds, args.max_hold_span)):
            parser.error("hold-seconds and max-hold-span must be finite and positive")
        result = json.dumps(report_bag(args.bag, args.hold_seconds, args.max_hold_span), indent=2)
        print(result)
        if args.out:
            args.out.write_text(result + "\n")
        return 0
    value = args.seconds if args.command == "record" else args.rate
    if not math.isfinite(value) or value <= 0:
        parser.error("duration/rate must be finite and positive")
    if args.command == "record":
        if args.bag.exists():
            parser.error("output already exists; choose a new bag directory")
        command = record_command(args.bag)
    else:
        report = inspect_bag(args.bag)
        if not report["all_inputs_recorded"]:
            parser.error("bag is missing inputs: " + ", ".join(report["missing_inputs"]))
        command = play_command(args.bag, args.rate)
    snapshots = {}
    parameter_files = {}
    if args.command == "record":
        print("Reading live parameters. Keep still until the recorder reports subscriptions.", flush=True)
        # Read while the rig is still stationary. Asking a busy/lost estimator
        # after a carry can time out and lose the settings behind the result.
        with tempfile.TemporaryDirectory(prefix="oak_parity_parameters_") as tmp:
            snapshots = save_parameters(Path(tmp))
            parameter_files = {p.name: p.read_text() for p in Path(tmp).glob("*.yaml")}
    process = subprocess.Popen(command, start_new_session=True)
    try:
        if args.command == "record":
            # Includes discovery time. Hold still until the recorder reports
            # all subscriptions, then leave at least 10 s holds at both ends.
            deadline = time.monotonic() + args.seconds
            while process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.1)
        else:
            process.wait()
    except KeyboardInterrupt:
        pass
    finally:
        stop(process)
    if args.command == "record":
        if not (args.bag / "metadata.yaml").exists():
            return 2
        report = inspect_bag(args.bag)
        report["recorder_exit_code"] = process.returncode
        report["record_command"] = command
        for filename, contents in parameter_files.items():
            (args.bag / filename).write_text(contents)
        report["parameter_snapshots"] = snapshots
        (args.bag / "parity_capture.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        return 0 if report["all_inputs_recorded"] and process.returncode == 0 else 2
    return process.returncode


if __name__ == "__main__":
    raise SystemExit(main())
