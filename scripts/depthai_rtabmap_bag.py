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
import struct
import time


TOPICS = (
    "/oak/left/image_rect", "/oak/stereo/image_raw",
    "/oak/left_rect_feature_tracker/tracked_features", "/oak/left/camera_info",
)
SYNC_NS = 1_000_000


def load_selection(path):
    """Return an ordered sequence->stamp map from a replay JSONL artifact."""
    if path is None:
        return None
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    selection = {}
    previous = 0
    for row in rows:
        sequence = row.get("sequence")
        stamp = row.get("stamp_ns")
        if (not isinstance(sequence, int) or isinstance(sequence, bool) or sequence <= previous
                or not isinstance(stamp, int) or isinstance(stamp, bool) or stamp <= 0):
            raise ValueError("selection rows require strictly increasing positive integer sequence and stamp_ns")
        selection[sequence] = stamp
        previous = sequence
    if not selection:
        raise ValueError("selection JSONL is empty")
    return selection


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


def validate_feature_passthrough(supplied, expected, sequence):
    """Require the SDK to acknowledge the actual input features, not just images."""
    def float32(value):
        return struct.unpack("<f", struct.pack("<f", value))[0]

    expected_points = [(float32(f.position.x), float32(f.position.y)) for f in expected.features]
    actual_points = [(float(f.position.x), float(f.position.y)) for f in supplied.trackedFeatures]
    if supplied.getSequenceNum() != sequence or actual_points != expected_points:
        raise RuntimeError("SDK feature passthrough does not match supplied tracks")


def matched_frames(messages, audit):
    """Join within 1 ms, allowing one second of recorded delivery reordering."""
    pending = {}
    last = -1

    def drain(stamp=None):
        nonlocal last
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
            elif stamp is None or stamp - oldest > 1_000_000_000:
                # Recorded CameraInfo may arrive over 300 ms after its image.
                # Retain bounded history without widening the 1 ms sync gate.
                audit["unmatched_messages"] += len(pending.pop(oldest))
            else:
                break

    for topic, message in messages:
        stamp = stamp_ns(message)
        key = min(pending, key=lambda k: abs(k-stamp)) if pending else stamp
        if abs(key-stamp) > SYNC_NS:
            key = stamp
        group = pending.setdefault(key, {})
        if topic in group:
            raise ValueError(f"Duplicate {topic} at {stamp}")
        group[topic] = message
        yield from drain(stamp)
    # A missing final input must not hide later complete tuples at EOF.
    yield from drain()


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
    if len(queues) == 2:
        return  # Explicit host feature detection; no feature input is created.
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
    parser.add_argument(
        "--stride-offset", type=int, default=0,
        help="Select sequence numbers where sequence modulo stride equals this offset",
    )
    parser.add_argument("--max-frames", type=int, default=0, help="Limit submitted frames; 0 processes the full bag")
    parser.add_argument("--params", type=Path, help="Optional JSON map of RTAB-Map string parameters")
    parser.add_argument(
        "--enable-slam", action="store_true",
        help="Also run DepthAI RTABMapSLAM with the flight stack's occupancy-grid settings",
    )
    parser.add_argument(
        "--feature-source", choices=("host", "oak", "device"), default="host",
        help=("host: RTABMap detects features; oak: require verified supplied-feature "
              "passthrough; device: replay images through an OAK FeatureTracker "
              "linked directly to RTABMapVIO"),
    )
    args = parser.parse_args()
    if args.stride < 1 or args.max_frames < 0 or not 0 <= args.stride_offset < args.stride:
        parser.error("stride must be positive, max-frames nonnegative, and stride-offset in [0,stride)")
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
    slam_outputs = 0
    grid_outputs = 0
    manifest = {
        "bag": str(args.bag), "depthai_version": dai.__version__, "params": params,
        "stride": args.stride, "stride_offset": args.stride_offset,
        "max_frames": args.max_frames, "input_audit": audit,
        "delivery_reorder_buffer_s": 1.0,
        "mode": ("VIO plus RTABMapSLAM, one frame in flight, no IMU"
                 if args.enable_slam else "visual core, one frame in flight, no mapper, no IMU"),
        "feature_source": args.feature_source,
        "feature_passthrough_verified": False if args.feature_source == "oak" else None,
        "device_feature_tracker_direct_link": args.feature_source == "device",
        "slam_enabled": args.enable_slam,
        "note": ("Offline diagnostic. Device calibration and SDK frame convention are retained; "
                 "not a real-time CPU test."),
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
            use_features = args.feature_source in ("oak", "device")
            odom.setUseFeatures(use_features)
            odom.setParams(params)
            if args.feature_source == "device":
                tracker = pipeline.create(dai.node.FeatureTracker)
                tracker.setHardwareResources(1, 2)
                tracker.initialConfig.setCornerDetector(
                    dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
                tracker.initialConfig.setNumTargetFeatures(400)
                tracker.initialConfig.setOpticalFlow()
                tracker.initialConfig.setMotionEstimator(True)
                tracker.initialConfig.FeatureMaintainer.minimumDistanceBetweenFeatures = 49
                tracker.passthroughInputImage.link(odom.rect)
                tracker.outputFeatures.link(odom.features)
                queues = [tracker.inputImage.createInputQueue(), odom.depth.createInputQueue()]
            else:
                queues = [odom.rect.createInputQueue(), odom.depth.createInputQueue()]
            if args.feature_source == "oak":
                queues.append(odom.features.createInputQueue())
            pose_queue = odom.transform.createOutputQueue()
            frame_queue = odom.passthroughRect.createOutputQueue()
            features_queue = (
                odom.passthroughFeatures.createOutputQueue()
                if args.feature_source == "oak" else None
            )
            slam_queue = grid_queue = None
            if args.enable_slam:
                slam_params = {
                    "RGBD/CreateOccupancyGrid": "true",
                    "Grid/3D": "false",
                    "Grid/DepthDecimation": "8",
                    "Grid/CellSize": "0.03",
                    "Grid/RayTracing": "true",
                    "Grid/PreVoxelFiltering": "true",
                    "Grid/RangeMin": "0.30",
                    "Grid/RangeMax": "4.0",
                    "Grid/NoiseFilteringRadius": "0.15",
                    "Grid/NoiseFilteringMinNeighbors": "4",
                    "Grid/ClusterRadius": "0.20",
                    "Grid/MinClusterSize": "20",
                    "GridGlobal/FootprintRadius": "0.25",
                    "Rtabmap/SaveWMState": "true",
                }
                manifest["slam_params"] = slam_params
                manifest["slam_database"] = str(args.out / "depthai_slam.db")
                slam = pipeline.create(dai.node.RTABMapSLAM)
                slam.setParams(slam_params)
                slam.setPublishObstacleCloud(False)
                slam.setPublishGroundCloud(False)
                slam.setPublishGrid(True)
                slam.setDatabasePath(manifest["slam_database"])
                slam.setLoadDatabaseOnStart(False)
                slam.setSaveDatabaseOnClose(True)
                odom.transform.link(slam.odom)
                odom.passthroughRect.link(slam.rect)
                odom.passthroughDepth.link(slam.depth)
                slam_queue = slam.transform.createOutputQueue()
                grid_queue = slam.occupancyGridMap.createOutputQueue()
            pipeline.start()
            epoch = stamp_ns(first[0])
            # SDK consumes its first tuple only to initialize the camera model.
            send_frame(queues, first, 0, epoch, dai, np)
            time.sleep(1)
            for sequence, frame in enumerate(frames, 1):
                if sequence % args.stride != args.stride_offset:
                    continue
                before = time.monotonic()
                send_frame(queues, frame, sequence, epoch, dai, np)
                pose = wait_message(pose_queue, pipeline)
                passthrough = wait_message(frame_queue, pipeline)
                if passthrough.getSequenceNum() != sequence:
                    raise RuntimeError("Output does not correspond to submitted image")
                if features_queue is not None:
                    supplied = wait_message(features_queue, pipeline)
                    validate_feature_passthrough(supplied, frame[2], sequence)
                slam_pose = wait_message(slam_queue, pipeline) if slam_queue is not None else None
                if slam_pose is not None:
                    slam_outputs += 1
                if grid_queue is not None:
                    while grid_queue.tryGet() is not None:
                        grid_outputs += 1
                p, q = pose.getTranslation(), pose.getQuaternion()
                position, quaternion = [p.x, p.y, p.z], [q.qx, q.qy, q.qz, q.qw]
                result = {"sequence": sequence, "stamp_ns": stamp_ns(frame[0]),
                          "t_s": (stamp_ns(frame[0])-epoch)/1e9,
                          "processing_s": time.monotonic()-before,
                          "position": position, "quaternion_xyzw": quaternion,
                          "valid": pose_is_valid(position, quaternion)}
                if slam_pose is not None:
                    sp, sq = slam_pose.getTranslation(), slam_pose.getQuaternion()
                    result["slam_position"] = [sp.x, sp.y, sp.z]
                    result["slam_quaternion_xyzw"] = [sq.qx, sq.qy, sq.qz, sq.qw]
                output.write(json.dumps(result, allow_nan=False) + "\n")
                results.append(result)
                if len(results) % 100 == 0:
                    print(f"{len(results)} outputs, {result['t_s']:.1f} s sensor time, "
                          f"{sum(not r['valid'] for r in results)} lost", flush=True)
                if args.max_frames and len(results) >= args.max_frames:
                    break
            manifest["complete"] = True
            if args.feature_source == "oak":
                manifest["feature_passthrough_verified"] = True
            if grid_queue is not None:
                settle = time.monotonic() + 2.0
                while time.monotonic() < settle:
                    message = grid_queue.tryGet()
                    if message is None:
                        time.sleep(0.01)
                    else:
                        grid_outputs += 1
    finally:
        valid = [r for r in results if r["valid"]]
        manifest.update(outputs=len(results), lost_outputs=len(results)-len(valid))
        if args.enable_slam:
            manifest.update(slam_pose_outputs=slam_outputs, slam_grid_outputs=grid_outputs)
        if results:
            manifest["lost_fraction"] = (len(results)-len(valid))/len(results)
            manifest["processing_median_s"] = statistics.median(r["processing_s"] for r in results)
            manifest["longest_valid_output_gap_sensor_s"] = max(
                (b["t_s"]-a["t_s"] for a, b in zip(valid, valid[1:])), default=None)
            manifest["final_valid_pose_age_sensor_s"] = results[-1]["t_s"]-valid[-1]["t_s"] if valid else None
            manifest["max_adjacent_valid_pose_step_m"] = max(
                (math.dist(a["position"], b["position"])
                 for a, b in zip(valid, valid[1:])), default=None)
            if args.enable_slam:
                manifest["max_adjacent_valid_slam_pose_step_m"] = max(
                    (math.dist(a["slam_position"], b["slam_position"])
                     for a, b in zip(valid, valid[1:])), default=None)
        (args.out / "report.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
