#!/usr/bin/env python3
"""Record OAK parity inputs and baseline outputs, or replay only sensor inputs.

Source the v3 message and project workspaces first. For replay, start:
  ros2 launch px4_vio_bridge ros_rtabmap_oak_v3_native_features.launch.py \
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
    "/tf", "/rtabmap/vio_odometry", "/rtabmap/vio_odom_info_lite",
    "/rtabmap/info",
    "/rtabmap/mapPath", "/rtabmap/localization_pose",
    "/rtabmap/octomap_metadata", "/rtabmap/oak_feature_count", "/diagnostics",
)
FULL_ODOM_INFO_TOPIC = "/rtabmap/vio_odom_info"
ODOM_STATUS_TOPICS = (
    "/rtabmap/vio_odom_info",
    "/rtabmap/vio_odom_info_lite",
    # Older launch files remap only odom_info, leaving the sibling publisher
    # at its node-relative name.
    "/rtabmap/odom_info_lite",
)
REPO = Path(__file__).resolve().parents[1]
QOS_PATH = REPO / "ros2_ws/src/px4_vio_bridge/config/oak_parity_bag_qos.yaml"


def record_command(bag, record_full_odom_info=False):
    topics = (*INPUT_TOPICS, *BASELINE_TOPICS)
    if record_full_odom_info:
        topics += (FULL_ODOM_INFO_TOPIC,)
    return [
        "ros2", "bag", "record", "--output", str(bag), "--storage", "mcap",
        "--disable-keyboard-controls", "--qos-profile-overrides-path", str(QOS_PATH),
        "--topics", *topics,
    ]


def wait_while_running(process, seconds):
    """Wait for a capture phase, returning early if rosbag exits."""
    deadline = time.monotonic() + seconds
    while process.poll() is None and time.monotonic() < deadline:
        time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))


def record_timeline(process, total_seconds, discovery_seconds,
                    pre_roll_seconds, post_roll_seconds):
    """Run a cued still/motion/still capture within the requested duration."""
    motion_seconds = total_seconds - discovery_seconds - pre_roll_seconds - post_roll_seconds
    print(
        f"RECORDER STARTED: keep still for {discovery_seconds:.1f} s while topics discover.",
        flush=True,
    )
    wait_while_running(process, discovery_seconds)
    if process.poll() is not None:
        return motion_seconds
    print(
        f"PRE-ROLL: keep the rig still for {pre_roll_seconds:.1f} s so odometry initializes.",
        flush=True,
    )
    wait_while_running(process, pre_roll_seconds)
    if process.poll() is not None:
        return motion_seconds
    return_warning_seconds = min(15.0, motion_seconds / 2.0)
    print(
        f"MOVE NOW: carry slowly and follow a closed loop. A return warning follows "
        f"in {motion_seconds - return_warning_seconds:.1f} s.",
        flush=True,
    )
    wait_while_running(process, motion_seconds - return_warning_seconds)
    if process.poll() is not None:
        return motion_seconds
    print(
        f"RETURN SOON: {return_warning_seconds:.1f} s remain; move back to the marked pose.",
        flush=True,
    )
    wait_while_running(process, return_warning_seconds)
    if process.poll() is not None:
        return motion_seconds
    print(
        f"RETURN/HOLD NOW: restore the marked pose and keep still for "
        f"{post_roll_seconds:.1f} s.",
        flush=True,
    )
    wait_while_running(process, post_roll_seconds)
    return motion_seconds


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
    for node in ("/oak", "/rtabmap/rgbd_odometry",
                 "/rtabmap/oak_tracked_odometry", "/rtabmap/rtabmap"):
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
    topics = ("/rtabmap/vio_odometry", *ODOM_STATUS_TOPICS, "/rtabmap/info")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    bag_topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    types = {name: get_message(msg_type) for name, msg_type in bag_topics.items()
             if name in topics}
    # Full OdomInfo carries feature/local-map arrays and can add meaningful
    # work inside OdometryROS's processing mutex.  Diagnostic runs may record
    # the built-in lite publisher instead; its scalar lost/inlier/feature
    # fields are sufficient for this report. Prefer full if a bag has both so
    # a normal capture is never double-counted.
    status_topic = next((topic for topic in ODOM_STATUS_TOPICS if topic in types), None)
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
        elif topic == status_topic:
            node.odom.append((t, msg.lost, msg.inliers, msg.features))
            if msg.lost:
                node.lost_odom_stamps.add(stamp)
        elif topic == "/rtabmap/info" and (msg.loop_closure_id or msg.proximity_detection_id):
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
            "odometry_status_topic": status_topic,
            **report(node, hold, start, end, max_hold_span)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    record = commands.add_parser("record", help="Record from an already running tracked launch")
    record.add_argument("bag", type=Path)
    record.add_argument("--seconds", type=float, default=90,
                        help="total recording time, including all capture phases")
    record.add_argument("--discovery-seconds", type=float, default=5.0)
    record.add_argument("--pre-roll-seconds", type=float, default=10.0)
    record.add_argument("--post-roll-seconds", type=float, default=15.0)
    record.add_argument(
        "--record-full-odom-info", action="store_true",
        help="also record feature/local-map arrays (adds serialization load)",
    )
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
        phases = (args.discovery_seconds, args.pre_roll_seconds, args.post_roll_seconds)
        if any(not math.isfinite(v) or v < 0 for v in phases):
            parser.error("capture phase durations must be finite and nonnegative")
        if sum(phases) >= args.seconds:
            parser.error("seconds must exceed discovery + pre-roll + post-roll")
        if args.bag.exists():
            parser.error("output already exists; choose a new bag directory")
        command = record_command(args.bag, args.record_full_odom_info)
    else:
        report = inspect_bag(args.bag)
        if not report["all_inputs_recorded"]:
            parser.error("bag is missing inputs: " + ", ".join(report["missing_inputs"]))
        command = play_command(args.bag, args.rate)
    snapshots = {}
    parameter_files = {}
    if args.command == "record":
        print("Reading live parameters. Keep still until MOVE NOW is printed.", flush=True)
        # Read while the rig is still stationary. Asking a busy/lost estimator
        # after a carry can time out and lose the settings behind the result.
        with tempfile.TemporaryDirectory(prefix="oak_parity_parameters_") as tmp:
            snapshots = save_parameters(Path(tmp))
            parameter_files = {p.name: p.read_text() for p in Path(tmp).glob("*.yaml")}
    motion_seconds = None
    process = subprocess.Popen(command, start_new_session=True)
    try:
        if args.command == "record":
            motion_seconds = record_timeline(
                process, args.seconds, args.discovery_seconds,
                args.pre_roll_seconds, args.post_roll_seconds,
            )
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
        report["capture_timeline"] = {
            "total_seconds": args.seconds,
            "discovery_seconds": args.discovery_seconds,
            "stationary_pre_roll_seconds": args.pre_roll_seconds,
            "motion_seconds": (
                motion_seconds if motion_seconds is not None else
                args.seconds - sum(phases)
            ),
            "return_warning_seconds": min(
                15.0, (args.seconds - sum(phases)) / 2.0
            ),
            "stationary_post_roll_seconds": args.post_roll_seconds,
            "full_odom_info_recorded": args.record_full_odom_info,
        }
        for filename, contents in parameter_files.items():
            (args.bag / filename).write_text(contents)
        report["parameter_snapshots"] = snapshots
        (args.bag / "parity_capture.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        return 0 if report["all_inputs_recorded"] and process.returncode == 0 else 2
    return process.returncode


if __name__ == "__main__":
    raise SystemExit(main())
