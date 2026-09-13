#!/usr/bin/env python3
"""Publish a deterministic visual subset at recorded wall-time cadence.

This is a ROS delivery/scheduling diagnostic. It publishes complete matched
RGB-D tuples before DDS synchronization, never publishes PX4 topics, and
records wall-clock input-to-OdomInfo latency and core processing timings.
Start a fresh replay launch with matching sensor QoS and no IMU before each
run. Mapping may be enabled for full-stack comparisons.
"""

import argparse
import json
import math
from pathlib import Path
import statistics
import time

from depthai_rtabmap_bag import TOPICS, bag_messages, matched_frames, stamp_ns


def spin_until_deadline(node, deadline, *, spin_once, monotonic=time.monotonic):
    """Service outputs until due without ever passing ROS an infinite wait."""
    while True:
        remaining = deadline - monotonic()
        if remaining <= 0:
            return
        # A negative ROS timeout means wait forever. Sample the clock only
        # once per iteration so crossing the deadline cannot make it negative.
        spin_once(node, timeout_sec=min(0.005, remaining))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--stride", type=int, default=2,
                        help="Publish every Nth matched tuple (2 is nominally 15 Hz for a 30 Hz bag)")
    parser.add_argument("--rate-hz", type=float,
                        help="Select frames on this sensor-time cadence instead of fixed stride")
    parser.add_argument("--feature-source", choices=("host", "oak"), default="host")
    parser.add_argument("--sensor-qos", type=int, choices=(1, 2), default=1,
                        help="RTAB-Map sensor QoS: 1 reliable (default), 2 best effort")
    parser.add_argument("--drain-seconds", type=float, default=5.0)
    args = parser.parse_args()
    if args.stride < 1:
        parser.error("stride must be positive")
    if args.rate_hz is not None and (not math.isfinite(args.rate_hz) or args.rate_hz <= 0):
        parser.error("rate-hz must be finite and positive")
    if not math.isfinite(args.drain_seconds) or args.drain_seconds < 0:
        parser.error("drain-seconds must be finite and nonnegative")
    if args.out.exists():
        parser.error("output already exists; choose a new directory")

    import rclpy
    from rcl_interfaces.srv import GetParameters
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock
    import rosbag2_py
    from rtabmap_msgs.msg import Info, OdomInfo
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from depthai_ros_msgs_v3.msg import TrackedFeatures
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node("parity_realtime_frame_replay")
    audit = {"matched_frames": 0, "unmatched_messages": 0, "max_tuple_skew_ns": 0}
    published = {}
    statuses = []
    pose_by_stamp = {}
    mapper_infos = []
    octomap_metadata = []
    producer_lateness = []
    wall_start = None
    epoch = None
    playback_complete = False
    estimator_input_stride = 1

    def spin_until(predicate, description, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError(f"Timed out waiting for {description}")
            rclpy.spin_once(node, timeout_sec=0.01)

    def receive_status(msg):
        stamp = stamp_ns(msg)
        if stamp in published:
            now = time.monotonic()
            statuses.append({
                "sequence": published[stamp],
                "stamp_ns": stamp,
                "t_s": (stamp - epoch) / 1e9,
                "arrival_s": now - wall_start,
                "sensor_to_output_s": now - wall_start - (stamp - epoch) / 1e9,
                "valid": not msg.lost,
                "inliers": msg.inliers,
                "features": msg.features,
                "time_estimation_s": msg.time_estimation,
                "local_bundle_time_s": msg.local_bundle_time,
                "local_bundle_constraints": msg.local_bundle_constraints,
                "local_key_frames": msg.local_key_frames,
                "local_map_size": msg.local_map_size,
                "key_frame_added": msg.key_frame_added,
            })

    def receive_pose(msg):
        position = msg.pose.pose.position
        pose_by_stamp[stamp_ns(msg)] = [position.x, position.y, position.z]

    try:
        odometry_node = "/rtabmap/" + (
            "rgbd_odometry" if args.feature_source == "host" else "oak_tracked_odometry")
        client = node.create_client(GetParameters, odometry_node + "/get_parameters")
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(f"Start a fresh replay launch with {odometry_node} first")
        parameter_names = ["use_sim_time", "wait_imu_to_init", "qos", "frame_id"]
        if args.feature_source == "oak":
            parameter_names.append("input_stride")
        request = GetParameters.Request(names=parameter_names)
        future = client.call_async(request)
        spin_until(future.done, "estimator parameters")
        values = future.result().values
        if (not values[0].bool_value or values[1].bool_value or
                values[2].integer_value != args.sensor_qos):
            raise ValueError(
                "Requires use_sim_time:=true, use_imu:=false, and matching --sensor-qos")
        if args.feature_source == "oak":
            estimator_input_stride = values[4].integer_value
            if estimator_input_stride < 1:
                raise ValueError("OAK estimator has an invalid input_stride")

        reliability = (ReliabilityPolicy.RELIABLE if args.sensor_qos == 1
                       else ReliabilityPolicy.BEST_EFFORT)
        sensor_qos = QoSProfile(depth=1, reliability=reliability)
        output_qos = QoSProfile(depth=20, reliability=reliability)
        publishers = [node.create_publisher(kind, topic, sensor_qos) for kind, topic in zip(
            (Image, Image, TrackedFeatures, CameraInfo), TOPICS)]
        clock = node.create_publisher(Clock, "/clock", 10)
        static = node.create_publisher(TFMessage, "/tf_static", QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(
            OdomInfo, "/rtabmap/vio_odom_info_lite", receive_status, output_qos)
        node.create_subscription(
            Odometry, "/rtabmap/vio_odometry", receive_pose, output_qos)
        node.create_subscription(
            Info, "/rtabmap/info", lambda msg: mapper_infos.append({
                "loop_closure_id": msg.loop_closure_id,
                "proximity_detection_id": msg.proximity_detection_id,
            }), 20)
        node.create_subscription(
            String, "/rtabmap/octomap_metadata",
            lambda msg: octomap_metadata.append(msg.data), 20)
        required = (0, 1, 3) if args.feature_source == "host" else range(4)
        spin_until(
            lambda: all(publishers[i].get_subscription_count() for i in required),
            "all visual subscribers")

        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(args.bag)),
                    rosbag2_py.ConverterOptions("", ""))
        reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf_static"]))
        transforms = {}
        while reader.has_next():
            for transform in deserialize_message(reader.read_next()[1], TFMessage).transforms:
                transforms[transform.child_frame_id] = transform
            if "oak" in transforms and "oak_left_camera_optical_frame" in transforms:
                break
        if not transforms:
            raise ValueError("Bag has no static calibration TF")

        frames = iter(matched_frames(bag_messages(args.bag), audit))
        first = next(frames)
        epoch = stamp_ns(first[0])
        clock.publish(Clock(clock=first[0].header.stamp))
        settle = time.monotonic() + 0.3
        while time.monotonic() < settle:
            rclpy.spin_once(node, timeout_sec=0.01)
        static.publish(TFMessage(transforms=list(transforms.values())))
        settle = time.monotonic() + 0.3
        while time.monotonic() < settle:
            rclpy.spin_once(node, timeout_sec=0.01)

        args.out.mkdir(parents=True)
        wall_start = time.monotonic()
        last_stamp = epoch
        next_sensor_due_s = 1.0 / args.rate_hz if args.rate_hz else None
        for sequence, frame in enumerate(frames, 1):
            stamp = stamp_ns(frame[0])
            sensor_s = (stamp - epoch) / 1e9
            if args.rate_hz:
                if sensor_s + 1e-9 < next_sensor_due_s:
                    continue
                while next_sensor_due_s <= sensor_s + 1e-9:
                    next_sensor_due_s += 1.0 / args.rate_hz
            elif sequence % args.stride:
                continue
            due = wall_start + (stamp - epoch) / 1e9
            spin_until_deadline(node, due, spin_once=rclpy.spin_once)
            producer_lateness.append(time.monotonic() - due)
            for message in frame:
                message.header.stamp = frame[0].header.stamp
            published[stamp] = sequence
            last_stamp = stamp
            clock.publish(Clock(clock=frame[0].header.stamp))
            for publisher, message in zip(publishers, frame):
                publisher.publish(message)
        playback_complete = True

        deadline = time.monotonic() + args.drain_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        complete = playback_complete and epoch is not None and bool(published)
        statuses.sort(key=lambda row: row["stamp_ns"])
        if args.out.exists():
            for row in statuses:
                if row["stamp_ns"] in pose_by_stamp:
                    row["position_m"] = pose_by_stamp[row["stamp_ns"]]
            with (args.out / "poses.jsonl").open("w") as stream:
                for row in statuses:
                    stream.write(json.dumps(row, allow_nan=False) + "\n")
            valid = [row for row in statuses if row["valid"]]
            posed = [row for row in valid if "position_m" in row]
            published_stamps = sorted(published)
            sensor_span_s = (
                (published_stamps[-1] - published_stamps[0]) / 1e9
                if len(published_stamps) > 1 else 0.0)
            measured_input_rate_hz = (
                (len(published_stamps) - 1) / sensor_span_s
                if sensor_span_s > 0 else None)
            expected_estimator_inputs = (
                (len(published) + estimator_input_stride - 1) // estimator_input_stride)
            report = {
                "mode": "deterministic pre-synchronization real-time ROS tuple replay",
                "bag": str(args.bag),
                "feature_source": args.feature_source,
                "sensor_qos": args.sensor_qos,
                "stride": None if args.rate_hz else args.stride,
                "requested_rate_hz": args.rate_hz,
                "nominal_input_rate_hz": args.rate_hz or measured_input_rate_hz,
                "input_audit": audit,
                "delivery_reorder_buffer_s": 1.0,
                "complete": complete,
                "published_frames": len(published),
                "estimator_input_stride": estimator_input_stride,
                "expected_estimator_inputs": expected_estimator_inputs,
                "status_outputs": len(statuses),
                "dropped_before_status": len(published) - len(statuses),
                "dropped_after_input_selection": expected_estimator_inputs - len(statuses),
                "lost_outputs": len(statuses) - len(valid),
                "lost_fraction": ((len(statuses) - len(valid)) / len(statuses)
                                  if statuses else None),
                "published_poses": len(posed),
                "max_pose_step_m": max((
                    math.dist(a["position_m"], b["position_m"])
                    for a, b in zip(posed, posed[1:])), default=None),
                "net_displacement_m": (
                    math.dist(posed[0]["position_m"], posed[-1]["position_m"])
                    if len(posed) > 1 else None),
                "longest_valid_output_gap_sensor_s": max(
                    (b["t_s"] - a["t_s"] for a, b in zip(posed, posed[1:])),
                    default=None),
                "final_valid_pose_age_sensor_s": (
                    (last_stamp - valid[-1]["stamp_ns"]) / 1e9 if valid else None),
                "sensor_to_output_median_s": statistics.median(
                    row["sensor_to_output_s"] for row in statuses) if statuses else None,
                "sensor_to_output_max_s": max(
                    (row["sensor_to_output_s"] for row in statuses), default=None),
                "producer_lateness_median_s": statistics.median(producer_lateness)
                if producer_lateness else None,
                "producer_lateness_max_s": max(producer_lateness, default=None),
                "mapper_info_outputs": len(mapper_infos),
                "mapper_global_closures": sum(
                    bool(row["loop_closure_id"]) for row in mapper_infos),
                "mapper_proximity_detections": sum(
                    bool(row["proximity_detection_id"]) for row in mapper_infos),
                "octomap_metadata_outputs": len(octomap_metadata),
                "final_octomap_metadata": None,
            }
            if octomap_metadata:
                try:
                    report["final_octomap_metadata"] = json.loads(octomap_metadata[-1])
                except json.JSONDecodeError:
                    report["final_octomap_metadata"] = octomap_metadata[-1]
            (args.out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
            print(json.dumps(report, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
