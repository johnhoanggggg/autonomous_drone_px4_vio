#!/usr/bin/env python3
"""Run a ROS parity bag through DepthAI 3.5's host RTABMapVIO visual core.

Source the ROS/v3 message workspaces and use a Python environment with depthai,
numpy and rosbag2_py. Requires the original OAK: the SDK reads its calibration
and runs its internal Sync node there. No live camera, ROS publisher or mapper
is created. Raw IMU is omitted deliberately: this SDK's F2M ignores IMU without
an orientation quaternion. This is an offline visual diagnostic, not live VIO
or full sensor/map parity. Restart the process for each comparison.
"""

import argparse
from datetime import timedelta
import json
import math
from pathlib import Path
import statistics
import time


TOPICS = (
    "/oak/left/image_rect", "/oak/stereo/image_raw",
    "/oak/left_rect_feature_tracker/tracked_features", "/oak/left/camera_info",
)
SYNC_NS = 1_000_000


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def pose_is_valid(position, quaternion):
    # DepthAI 3.5 with its default local transform emits this *unit* quaternion
    # at the origin on a forced blank-image/depth failure. The SDK exposes no
    # lost bit, so conservatively reject this exact failure sentinel as well
    # as non-unit/nonfinite output. Quaternion norm alone is insufficient.
    if all(v == 0.0 for v in position) and all(v == 0.5 for v in quaternion):
        return False
    return (all(math.isfinite(v) for v in (*position, *quaternion))
            and abs(sum(v*v for v in quaternion) - 1.0) < 0.01)


def matched_frames(messages, audit):
    """Join image/depth/tracks/calibration within 1 ms with bounded buffering."""
    pending = {}
    last = -1
    for topic, message in messages:
        stamp = stamp_ns(message)
        key = min(pending, key=lambda k: abs(k-stamp)) if pending else stamp
        if abs(key-stamp) > SYNC_NS:
            key = stamp
        group = pending.setdefault(key, {})
        if topic in group:
            raise ValueError(f"Duplicate {topic} at {stamp}")
        group[topic] = message
        while pending:
            oldest = min(pending)
            group = pending[oldest]
            if len(group) == len(TOPICS):
                stamps = [stamp_ns(group[t]) for t in TOPICS]
                if max(stamps) - min(stamps) > SYNC_NS:
                    raise ValueError("Input tuple exceeds 1 ms sync gate")
                current = stamps[0]
                if current <= last:
                    raise ValueError("Matched image stamps are not strictly increasing")
                last = current
                audit["matched_frames"] += 1
                audit["max_tuple_skew_ns"] = max(audit["max_tuple_skew_ns"], max(stamps)-min(stamps))
                del pending[oldest]
                yield tuple(group[t] for t in TOPICS)
            elif stamp - oldest > 250_000_000:
                audit["unmatched_messages"] += len(pending.pop(oldest))
            else:
                break
    audit["unmatched_messages"] += sum(len(g) for g in pending.values())


def bag_messages(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path)), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in TOPICS}
    if set(types) != set(TOPICS):
        raise ValueError(f"Missing visual inputs: {set(TOPICS) - set(types)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    while reader.has_next():
        topic, data, _ = reader.read_next()
        yield topic, deserialize_message(data, types[topic])


def image_bytes(message, encoding, np):
    if message.encoding != encoding:
        raise ValueError(f"Expected {encoding}, got {message.encoding}")
    itemsize = 1 if encoding == "mono8" else 2
    if message.step < message.width * itemsize or len(message.data) != message.step * message.height:
        raise ValueError("Invalid image stride or buffer size")
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    packed = rows[:, :message.width * itemsize].copy()
    if itemsize == 2 and message.is_bigendian:
        packed = packed.reshape(-1, 2)[:, ::-1].copy()
    return packed.reshape(-1)


def send_frame(queues, frames, sequence, epoch_ns, dai, np):
    image, depth, features, info = frames
    if (image.width, image.height) != (depth.width, depth.height) or (
            image.width, image.height) != (info.width, info.height):
        raise ValueError("Image/depth/calibration dimensions differ")
    # Sync uses one canonical image stamp. Preserve the original nanosecond
    # stamp in results; Python's SDK timedelta has microsecond precision.
    stamp = timedelta(microseconds=(stamp_ns(image)-epoch_ns)//1000 + 10_000_000)
    for index, (message, encoding, kind) in enumerate((
            (image, "mono8", dai.ImgFrame.Type.GRAY8),
            (depth, "16UC1", dai.ImgFrame.Type.RAW16))):
        frame = dai.ImgFrame()
        frame.setType(kind)
        frame.setWidth(message.width)
        frame.setHeight(message.height)
        frame.setInstanceNum(int(dai.CameraBoardSocket.CAM_B))
        frame.setData(image_bytes(message, encoding, np))
        frame.setTimestamp(stamp)
        frame.setTimestampDevice(stamp)
        frame.setSequenceNum(sequence)
        queues[index].send(frame)
    tracked = dai.TrackedFeatures()
    points = []
    for feature in features.features:
        point = dai.TrackedFeature()
        point.position = dai.Point2f(feature.position.x, feature.position.y)
        point.id = feature.id
        points.append(point)
    tracked.trackedFeatures = points
    tracked.setTimestamp(stamp)
    tracked.setTimestampDevice(stamp)
    tracked.setSequenceNum(sequence)
    queues[2].send(tracked)


def wait_message(queue, pipeline, timeout=10):
    deadline = time.monotonic() + timeout
    while pipeline.isRunning() and time.monotonic() < deadline:
        message = queue.tryGet()
        if message is not None:
            return message
        time.sleep(0.001)
    raise RuntimeError("DepthAI output timed out; comparison is incomplete")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--out", type=Path, required=True, help="New directory for results and device calibration")
    parser.add_argument("--stride", type=int, default=1, help="Process every Nth matched frame")
    parser.add_argument("--max-frames", type=int, default=0, help="Limit submitted frames; 0 processes the full bag")
    parser.add_argument("--params", type=Path, help="Optional JSON map of RTAB-Map string parameters")
    args = parser.parse_args()
    if args.stride < 1 or args.max_frames < 0:
        parser.error("stride must be positive and max-frames nonnegative")
    if args.out.exists():
        parser.error("output already exists; choose a new directory")
    import depthai as dai
    import numpy as np

    params = json.loads(args.params.read_text()) if args.params else {}
    if not isinstance(params, dict) or any(not isinstance(k, str) or not isinstance(v, str) for k, v in params.items()):
        parser.error("params must contain a JSON object with string keys and values")
    audit = {"matched_frames": 0, "unmatched_messages": 0, "max_tuple_skew_ns": 0}
    frames = iter(matched_frames(bag_messages(args.bag), audit))
    first = next(frames)
    args.out.mkdir(parents=True)
    results = []
    manifest = {
        "bag": str(args.bag), "depthai_version": dai.__version__, "params": params,
        "stride": args.stride, "max_frames": args.max_frames, "input_audit": audit,
        "mode": "visual core, one frame in flight, no mapper, no IMU",
        "note": "Offline diagnostic. Device calibration and SDK frame convention are retained; not a real-time or map parity test.",
        "validity": "finite unit quaternion excluding SDK 3.5 default-frame failure sentinel: zero position and quaternion (0.5,0.5,0.5,0.5); no SDK lost bit is available",
        "complete": False,
    }
    try:
        with dai.Pipeline() as pipeline, (args.out / "poses.jsonl").open("w", buffering=1) as output:
            device = pipeline.getDefaultDevice()
            calibration = device.readCalibration()
            calibration.eepromToJsonFile(str(args.out / "device_calibration.json"))
            manifest["device"] = str(device.getDeviceInfo())
            intrinsics = np.array(calibration.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, first[0].width, first[0].height))
            projection = np.array(first[3].p).reshape(3, 4)
            if not np.allclose(intrinsics, projection[:, :3], atol=1e-4, rtol=0):
                raise ValueError("Bag intrinsics do not match this OAK's SDK camera model")
            manifest["bag_projection"] = projection.tolist()
            odom = pipeline.create(dai.node.RTABMapVIO)
            odom.setUseFeatures(True)
            odom.setParams(params)
            queues = [odom.rect.createInputQueue(), odom.depth.createInputQueue(), odom.features.createInputQueue()]
            pose_queue = odom.transform.createOutputQueue()
            frame_queue = odom.passthroughRect.createOutputQueue()
            pipeline.start()
            epoch = stamp_ns(first[0])
            # SDK consumes its first tuple only to initialize the camera model.
            send_frame(queues, first, 0, epoch, dai, np)
            time.sleep(1)
            for sequence, frame in enumerate(frames, 1):
                if sequence % args.stride:
                    continue
                before = time.monotonic()
                send_frame(queues, frame, sequence, epoch, dai, np)
                pose = wait_message(pose_queue, pipeline)
                passthrough = wait_message(frame_queue, pipeline)
                if passthrough.getSequenceNum() != sequence:
                    raise RuntimeError("Output does not correspond to submitted image")
                p, q = pose.getTranslation(), pose.getQuaternion()
                position, quaternion = [p.x, p.y, p.z], [q.qx, q.qy, q.qz, q.qw]
                result = {"sequence": sequence, "stamp_ns": stamp_ns(frame[0]),
                          "t_s": (stamp_ns(frame[0])-epoch)/1e9,
                          "processing_s": time.monotonic()-before,
                          "position": position, "quaternion_xyzw": quaternion,
                          "valid": pose_is_valid(position, quaternion)}
                output.write(json.dumps(result, allow_nan=False) + "\n")
                results.append(result)
                if len(results) % 100 == 0:
                    print(f"{len(results)} outputs, {result['t_s']:.1f} s sensor time, "
                          f"{sum(not r['valid'] for r in results)} lost", flush=True)
                if args.max_frames and len(results) >= args.max_frames:
                    break
            manifest["complete"] = True
    finally:
        valid = [r for r in results if r["valid"]]
        manifest.update(outputs=len(results), lost_outputs=len(results)-len(valid))
        if results:
            manifest["lost_fraction"] = (len(results)-len(valid))/len(results)
            manifest["processing_median_s"] = statistics.median(r["processing_s"] for r in results)
            manifest["longest_valid_output_gap_sensor_s"] = max(
                (b["t_s"]-a["t_s"] for a, b in zip(valid, valid[1:])), default=None)
            manifest["final_valid_pose_age_sensor_s"] = results[-1]["t_s"]-valid[-1]["t_s"] if valid else None
        (args.out / "report.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
