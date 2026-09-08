#!/usr/bin/env python3
"""Replay exactly one visual tuple at a time into a fresh ROS estimator.

Start the tracked launch with start_camera:=false use_sim_time:=true
use_imu:=false odometry_rate_hz:=0.0 sensor_qos:=1 foxglove:=false. No real-time rate is
measured: each input waits for its OdomInfo acknowledgement. Uses the same
tuple matcher and initialization-frame exclusion as depthai_rtabmap_bag.py.
"""

import argparse
import json
from pathlib import Path
import time

from depthai_rtabmap_bag import TOPICS, bag_messages, matched_frames, stamp_ns


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--native-body-frame", action="store_true",
                        help="Use SDK 3.5's internal camera-model frame; launch base_frame_id:=native_sdk_internal")
    args = parser.parse_args()
    if args.stride < 1 or args.max_frames < 0:
        parser.error("stride must be positive and max-frames nonnegative")
    if args.out.exists():
        parser.error("output already exists; choose a new directory")

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
    manifest = {"bag": str(args.bag), "stride": args.stride, "max_frames": args.max_frames,
                "mode": "ROS visual core, one frame in flight, no IMU", "input_audit": audit,
                "native_body_frame": args.native_body_frame,
                "complete": False}

    def wait_until(predicate, description):
        deadline = time.monotonic() + 10
        while not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError(f"Timed out waiting for {description}; comparison is incomplete")
            rclpy.spin_once(node, timeout_sec=0.01)

    try:
        client = node.create_client(GetParameters, "/rtabmap/oak_tracked_odometry/get_parameters")
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError("Start a fresh tracked replay launch first")
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
        wait_until(lambda: all(p.get_subscription_count() for p in publishers), "all visual subscribers")

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
                if sequence % args.stride:
                    continue
                stamp = stamp_ns(frame[0])
                # Identical canonicalization to the SDK replay: tracks can be
                # fractions of a microsecond away from their image stamp.
                for message in frame:
                    message.header.stamp = frame[0].header.stamp
                clock.publish(Clock(clock=frame[0].header.stamp))
                before = time.monotonic()
                for publisher, message in zip(publishers, frame):
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
