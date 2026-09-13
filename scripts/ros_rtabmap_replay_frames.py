#!/usr/bin/env python3
"""Replay exactly one visual tuple at a time into a fresh ROS estimator.

Start the tracked launch with start_camera:=false use_sim_time:=true
use_imu:=false odometry_rate_hz:=0.0 sensor_qos:=1 foxglove:=false. No real-time rate is
measured: each input waits for its OdomInfo acknowledgement. Uses the same
tuple matcher and initialization-frame exclusion as depthai_rtabmap_bag.py.
"""

import argparse
import json
import math
from pathlib import Path
import time

from depthai_rtabmap_bag import (
    TOPICS, bag_messages, load_selection, matched_frames, stamp_ns,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument(
        "--stride-offset", type=int, default=0,
        help="Select sequence numbers where sequence modulo stride equals this offset",
    )
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument(
        "--ack-timeout", type=float, default=10.0,
        help="seconds to wait for each matching OdomInfo acknowledgment",
    )
    parser.add_argument("--selection-jsonl", type=Path,
                        help="Process exactly the sequence/stamp rows in another replay's poses.jsonl")
    parser.add_argument("--feature-source", choices=("oak", "host"), default="oak",
                        help="oak: custom tracked component; host: standard rgbd_odometry detects features")
    parser.add_argument(
        "--oak-feature-control", choices=("original", "center", "withhold"),
        default="original",
        help=("Diagnostic control for OAK input: preserve positions, collapse all "
              "positions to image center, or omit the TrackedFeatures message"),
    )
    parser.add_argument("--normalize-projection", action="store_true",
                        help="Diagnostic only: zero CameraInfo P translations to match SDK's camera model")
    parser.add_argument("--native-body-frame", action="store_true",
                        help="Use SDK 3.5's internal camera-model frame; launch base_frame_id:=native_sdk_internal")
    args = parser.parse_args()
    if args.stride < 1 or args.max_frames < 0 or not 0 <= args.stride_offset < args.stride:
        parser.error("stride must be positive, max-frames nonnegative, and stride-offset in [0,stride)")
    if not math.isfinite(args.ack_timeout) or args.ack_timeout <= 0:
        parser.error("ack-timeout must be finite and positive")
    if args.selection_jsonl and (args.stride != 1 or args.stride_offset != 0):
        parser.error("--selection-jsonl cannot be combined with stride selection")
    if args.feature_source != "oak" and args.oak_feature_control != "original":
        parser.error("--oak-feature-control is only valid with --feature-source oak")
    if args.out.exists():
        parser.error("output already exists; choose a new directory")
    try:
        selection = load_selection(args.selection_jsonl)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.error(str(error))

    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rcl_interfaces.srv import GetParameters
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from rtabmap_msgs.msg import OdomInfo
    from sensor_msgs.msg import CameraInfo, Image
    from depthai_ros_msgs_v3.msg import TrackedFeatures
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import TransformStamped
    from rclpy.serialization import deserialize_message
    import rosbag2_py

    rclpy.init()
    node = rclpy.create_node("parity_frame_replay")
    results = []
    audit = {"matched_frames": 0, "unmatched_messages": 0, "max_tuple_skew_ns": 0}
    manifest = {"bag": str(args.bag), "stride": args.stride,
                "stride_offset": args.stride_offset, "max_frames": args.max_frames,
                "ack_timeout_s": args.ack_timeout,
                "mode": "ROS visual core, one frame in flight, no IMU", "input_audit": audit,
                "delivery_reorder_buffer_s": 1.0,
                "native_body_frame": args.native_body_frame,
                "feature_source": args.feature_source,
                "oak_feature_control": args.oak_feature_control,
                "selection_jsonl": str(args.selection_jsonl) if args.selection_jsonl else None,
                "selected_input_frames": len(selection) if selection else None,
                "projection_translation_zeroed": args.normalize_projection,
                "complete": False}

    def wait_until(predicate, description):
        deadline = time.monotonic() + args.ack_timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError(f"Timed out waiting for {description}; comparison is incomplete")
            rclpy.spin_once(node, timeout_sec=0.01)

    try:
        odometry_node = "/rtabmap/" + ("rgbd_odometry" if args.feature_source == "host" else "oak_tracked_odometry")
        manifest["odometry_node"] = odometry_node
        client = node.create_client(GetParameters, odometry_node + "/get_parameters")
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(f"Start a fresh replay launch with {odometry_node} first")
        names = ["use_sim_time", "wait_imu_to_init", "max_update_rate", "qos", "frame_id"]
        request = GetParameters.Request(names=names)
        future = client.call_async(request)
        wait_until(future.done, "estimator parameters")
        values = future.result().values
        if (not values[0].bool_value or values[1].bool_value or values[2].double_value != 0.0
                or values[3].integer_value != 1):
            raise ValueError("Requires use_sim_time:=true use_imu:=false odometry_rate_hz:=0.0 sensor_qos:=1")
        manifest["base_frame_id"] = values[4].string_value
        if args.native_body_frame and values[4].string_value != "native_sdk_internal":
            raise ValueError("--native-body-frame requires base_frame_id:=native_sdk_internal")
        publishers = [node.create_publisher(kind, topic, 10) for kind, topic in zip(
            (Image, Image, TrackedFeatures, CameraInfo), TOPICS)]
        clock = node.create_publisher(Clock, "/clock", 10)
        static = node.create_publisher(TFMessage, "/tf_static", QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        statuses, poses = {}, {}
        node.create_subscription(OdomInfo, "/rtabmap/vio_odom_info",
                                 lambda m: statuses.__setitem__(stamp_ns(m), m), 10)
        node.create_subscription(Odometry, "/rtabmap/vio_odometry",
                                 lambda m: poses.__setitem__(stamp_ns(m), m), 10)
        required = (0, 1, 3) if args.feature_source == "host" else range(4)
        wait_until(lambda: all(publishers[i].get_subscription_count() for i in required), "all visual subscribers")

        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(args.bag)), rosbag2_py.ConverterOptions("", ""))
        reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf_static"]))
        transforms = {}
        while reader.has_next():
            for transform in deserialize_message(reader.read_next()[1], TFMessage).transforms:
                transforms[transform.child_frame_id] = transform
        if not transforms:
            raise ValueError("Bag has no static calibration TF")
        frames = iter(matched_frames(bag_messages(args.bag), audit))
        first = next(frames)
        epoch = stamp_ns(first[0])
        if args.native_body_frame:
            # SDK 3.5 buildInternal() uses opticalTransform.inverse() as its
            # camera model's local transform, with no camera lever arm. Use
            # that virtual body as the calibration root, since OdometryROS
            # will publish odom -> body. Retaining the rig's oak -> optical
            # edge would give the optical frame conflicting parents.
            extra = TransformStamped()
            extra.header.frame_id = "native_sdk_internal"
            extra.child_frame_id = first[0].header.frame_id
            extra.transform.rotation.x = 0.5
            extra.transform.rotation.y = -0.5
            extra.transform.rotation.z = 0.5
            extra.transform.rotation.w = 0.5
            transforms = {extra.child_frame_id: extra}
        # The first /clock jump can clear TF buffers. Establish simulated time
        # before sending calibration, then allow DDS/TF callbacks to settle.
        clock.publish(Clock(clock=first[0].header.stamp))
        until = time.monotonic() + 0.3
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.01)
        static.publish(TFMessage(transforms=list(transforms.values())))
        until = time.monotonic() + 0.3
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.01)
        args.out.mkdir(parents=True)
        with (args.out / "poses.jsonl").open("w", buffering=1) as output:
            for sequence, frame in enumerate(frames, 1):
                if selection is not None and sequence not in selection:
                    continue
                if sequence % args.stride != args.stride_offset:
                    continue
                stamp = stamp_ns(frame[0])
                if selection is not None and stamp != selection[sequence]:
                    raise ValueError(
                        f"selection stamp mismatch at sequence {sequence}: "
                        f"expected {selection[sequence]}, bag has {stamp}")
                # Identical canonicalization to the SDK replay: tracks can be
                # fractions of a microsecond away from their image stamp.
                for message in frame:
                    message.header.stamp = frame[0].header.stamp
                if args.normalize_projection:
                    # Standard RGBDOdometry retains the driver's stereo P[3].
                    # Use the same left RGB-D projection as the SDK and custom
                    # adapter for this controlled estimator comparison.
                    frame[3].p[3] = 0.0
                    frame[3].p[7] = 0.0
                if args.oak_feature_control == "center":
                    for feature in frame[2].features:
                        feature.position.x = frame[0].width / 2.0
                        feature.position.y = frame[0].height / 2.0
                clock.publish(Clock(clock=frame[0].header.stamp))
                before = time.monotonic()
                published_frame = (
                    frame if args.oak_feature_control != "withhold"
                    else (frame[0], frame[1], frame[3])
                )
                published_indices = (
                    range(4) if args.oak_feature_control != "withhold"
                    else (0, 1, 3)
                )
                for index, message in zip(published_indices, published_frame):
                    publisher = publishers[index]
                    publisher.publish(message)
                wait_until(lambda: stamp in statuses, "matching OdomInfo")
                status = statuses.pop(stamp)
                result = {"sequence": sequence, "stamp_ns": stamp, "t_s": (stamp-epoch)/1e9,
                          "valid": not status.lost, "inliers": status.inliers}
                # In RTAB-Map 0.22.1 a zero interval means no velocity estimate
                # yet, at startup and on the first success after tracking loss.
                # With publish_null_when_lost=false OdometryROS suppresses
                # that pose, while OdomInfo acknowledges the successful frame.
                without_velocity = not status.lost and status.interval == 0.0
                result["pose_suppressed_without_velocity"] = without_velocity
                if not status.lost and not without_velocity:
                    wait_until(lambda: stamp in poses, "matching valid odometry")
                    pose = poses.pop(stamp).pose.pose
                    result["position"] = [pose.position.x, pose.position.y, pose.position.z]
                    result["quaternion_xyzw"] = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                result["processing_s"] = time.monotonic()-before
                output.write(json.dumps(result, allow_nan=False) + "\n")
                results.append(result)
                # OdometryROS publishes OdomInfo while still holding its data
                # mutex. Give its remaining publication/diagnostic work time
                # to finish before submitting the next tuple; processData()
                # otherwise drops that tuple on lockTry(). This is offline
                # pacing, excluded from the per-frame elapsed measurement.
                until = time.monotonic() + 0.01
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.001)
                if len(results) % 100 == 0:
                    print(f"{len(results)} outputs, {result['t_s']:.1f} s sensor time, "
                          f"{sum(not r['valid'] for r in results)} lost", flush=True)
                if args.max_frames and len(results) >= args.max_frames:
                    break
            if selection is not None and len(results) != len(selection):
                raise ValueError(
                    f"selection requested {len(selection)} frames but replay found {len(results)}")
            manifest["complete"] = True
    finally:
        node.destroy_node()
        rclpy.shutdown()
        valid = [r for r in results if r["valid"]]
        published = [r for r in valid if "position" in r]
        manifest.update(outputs=len(results), lost_outputs=len(results)-len(valid), published_poses=len(published))
        if results:
            manifest["lost_fraction"] = (len(results)-len(valid))/len(results)
            manifest["longest_valid_output_gap_sensor_s"] = max(
                (b["t_s"]-a["t_s"] for a, b in zip(published, published[1:])), default=None)
            manifest["final_valid_pose_age_sensor_s"] = results[-1]["t_s"]-published[-1]["t_s"] if published else None
        if args.out.exists():
            (args.out / "report.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
