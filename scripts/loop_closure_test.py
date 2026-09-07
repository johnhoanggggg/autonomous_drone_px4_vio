#!/usr/bin/env python3
"""Hand-carried loop-closure test for the hybrid stack.

Stationary tests keep the SLAM correction at identity, so this records a carried
loop and reports tracking loss, valid-pose discontinuities, global loop closures
and proximity detections. The ROS-native launch does not yet publish the relay
topics needed to measure its corrected pose directly.

Usage, with the hybrid already running:

    ros2 launch px4_vio_bridge rtabmap_slam_px4_3d.launch.py foxglove:=false
    python3 scripts/loop_closure_test.py --seconds 180

Then: hold still ~20 s, carry the drone slowly around a closed loop of a few
metres keeping textured scenery in view, return it to the marked start in the
original orientation, hold still ~20 s. No props.

Both the hybrid and the original full DepthAI pipeline can be measured with
this and compared directly, because both publish /rtabmap/vio_pose,
/rtabmap/pose and /rtabmap/odom_correction. Only the ROS mapper publishes
rtabmap_msgs/Info, so on the DepthAI pipeline the closure count will read 0;
use "correction_updates" and "corrected_pose_jumps", which work on both.

    # original full DepthAI pipeline (its own RTABMapSLAM does the closing)
    ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py foxglove:=false
    python3 scripts/loop_closure_test.py --label depthai --seconds 180

Note: the "odometry_while_moving" section only appears on the ROS-native stack
(ros_rtabmap_oak_v3.launch.py). The hybrid's odometry is a DepthAI host node,
which publishes no rtabmap_msgs/OdomInfo, so lost/inlier counts are unavailable
there and the section is simply omitted.
"""

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import Info, OdomInfo


def dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def mean_point(points):
    n = float(len(points))
    return tuple(sum(p[i] for p in points) / n for i in range(3))


def rms_radius(points):
    """3D RMS distance from the point-cloud mean."""
    centre = mean_point(points)
    return math.sqrt(sum(dist(point, centre) ** 2 for point in points)
                     / len(points))


class LoopClosureTest(Node):
    def __init__(self, args):
        super().__init__("loop_closure_test")
        self.args = args
        self.raw = []          # (t, x, y, z) from the pose PX4 consumes
        self.corrected = []    # (t, x, y, z) from the SLAM-corrected pose
        self.corrections = []  # (t, x, y, z of map->odom translation)
        self.closures = []     # (t, ref_id, loop_id, |transform|, kind)
        self.odom = []         # (t, lost, inliers, features)
        self.raw_odom_stamps = []
        self.lost_odom_stamps = set()

        # A best-effort reader is compatible with both best-effort sensor-data
        # publishers and reliable publishers. Creating one reader of each kind
        # would duplicate every sample from a reliable publisher and emit an
        # incompatibility warning for every best-effort publisher.
        qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        if args.raw_odom_topic:
            self.create_subscription(Odometry, args.raw_odom_topic,
                                     self._raw_odom_cb, qos)
        else:
            self.create_subscription(PoseStamped, args.raw_pose_topic,
                                     self._raw_cb, qos)
        self.create_subscription(PoseStamped, args.pose_topic,
                                 self._corrected_cb, qos)
        self.create_subscription(PoseStamped, args.correction_topic,
                                 self._correction_cb, qos)
        self.create_subscription(Info, args.info_topic, self._info_cb, qos)
        self.create_subscription(OdomInfo, args.odom_info_topic,
                                 self._odom_cb, qos)

    def _pt(self, msg):
        p = msg.pose.position
        return (time.monotonic(), p.x, p.y, p.z)

    def _raw_cb(self, msg):
        self.raw.append(self._pt(msg))
        self.raw_odom_stamps.append(None)

    def _raw_odom_cb(self, msg):
        p = msg.pose.pose.position
        self.raw.append((time.monotonic(), p.x, p.y, p.z))
        self.raw_odom_stamps.append(self._stamp_ns(msg))

    @staticmethod
    def _stamp_ns(msg):
        stamp = msg.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _corrected_cb(self, msg):
        self.corrected.append(self._pt(msg))

    def _correction_cb(self, msg):
        p = msg.pose.position
        self.corrections.append((time.monotonic(), p.x, p.y, p.z))

    def _info_cb(self, msg):
        if msg.loop_closure_id != 0 or msg.proximity_detection_id != 0:
            t = msg.loop_closure_transform.translation
            kind = ("global_loop_closure" if msg.loop_closure_id != 0
                    else "proximity_detection")
            self.closures.append((
                time.monotonic(), int(msg.ref_id),
                int(msg.loop_closure_id or msg.proximity_detection_id),
                math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2),
                kind,
            ))

    def _odom_cb(self, msg):
        if msg.lost:
            self.lost_odom_stamps.add(self._stamp_ns(msg))
        self.odom.append(
            (time.monotonic(), bool(msg.lost), int(msg.inliers),
             int(msg.features)))


def biggest_jumps(series, n=5):
    """Largest consecutive-sample position steps, in mm."""
    steps = [(1000.0 * dist(series[i][1:], series[i + 1][1:]),
              round(series[i + 1][0] - series[0][0], 1))
             for i in range(len(series) - 1)]
    steps.sort(reverse=True)
    return [{"jump_mm": round(s, 2), "at_s": t} for s, t in steps[:n]]


def pose_output_stats(series, gap_threshold=0.5):
    """Describe estimator throughput without counting long outages as work."""
    intervals = [
        (series[i + 1][0] - series[i][0], series[i + 1][0] - series[0][0])
        for i in range(len(series) - 1)
    ]
    active = [duration for duration, _ in intervals
              if duration < gap_threshold]
    gaps = [(duration, at_s) for duration, at_s in intervals
            if duration >= gap_threshold]
    active_duration = sum(active)
    return {
        "active_rate_hz": round(len(active) / active_duration, 1)
        if active_duration else 0.0,
        "gap_threshold_s": gap_threshold,
        "output_gap_count": len(gaps),
        "longest_output_gap_s": round(
            max((duration for duration, _ in gaps), default=0.0), 2),
        "output_gap_events": [
            {"at_s": round(at_s, 1), "duration_s": round(duration, 2)}
            for duration, at_s in gaps[:20]
        ],
    }


def report(node, hold, recording_start=None, recording_end=None, max_hold_span=0.05):
    out = {}
    # When publish_null_when_lost was enabled, RTAB-Map paired a lost OdomInfo
    # with an identity nav_msgs/Odometry at the same stamp. Those are invalid
    # samples, not estimator resets. Exclude them before measuring the valid
    # pose trajectory while continuing to report the tracking loss below.
    invalid_null_samples = sum(
        1 for stamp in node.raw_odom_stamps
        if stamp is not None and stamp in node.lost_odom_stamps)
    raw = [
        sample for sample, stamp in zip(node.raw, node.raw_odom_stamps)
        if stamp is None or stamp not in node.lost_odom_stamps
    ]
    corr = node.corrected

    if len(raw) > 20:
        span = raw[-1][0] - raw[0][0]
        window_start = (recording_start if recording_start is not None
                        else raw[0][0])
        window_end = (recording_end if recording_end is not None
                      else raw[-1][0])
        recording_span = window_end - window_start
        start = [p for p in raw
                 if window_start <= p[0] <= window_start + hold]
        end = [p for p in raw
               if window_end - hold <= p[0] <= window_end]

        # A return comparison is only meaningful when the valid pose stream
        # covers two distinct stationary windows. Previously, a pose stream
        # that stopped during motion could make both windows select the same
        # samples and incorrectly report a perfect 0 mm return.
        hold_errors = []
        if recording_span < 2.0 * hold:
            hold_errors.append(
                f"recording is {recording_span:.1f} s; at least "
                f"{2.0 * hold:.1f} s is required for non-overlapping holds")
        start_coverage = start[-1][0] - start[0][0] if len(start) > 1 else 0.0
        end_coverage = end[-1][0] - end[0][0] if len(end) > 1 else 0.0
        minimum_hold_coverage = 0.5 * hold
        if not start:
            hold_errors.append("no valid poses in the starting hold window")
        elif start_coverage < minimum_hold_coverage:
            hold_errors.append(
                f"starting hold has only {start_coverage:.1f} s of valid "
                f"coverage; at least {minimum_hold_coverage:.1f} s is required")
        if not end:
            hold_errors.append("no valid poses in the ending hold window")
        elif end_coverage < minimum_hold_coverage:
            hold_errors.append(
                f"ending hold has only {end_coverage:.1f} s of valid coverage; "
                f"at least {minimum_hold_coverage:.1f} s is required")

        # Coverage alone does not establish a hold: a camera still being
        # carried at the end previously produced a plausible "return error".
        # Use the diagonal of the position bounds, without assuming that the
        # start/end means themselves must be close (that is what we measure).
        hold_spans = {}
        for name, samples in (("start", start), ("end", end)):
            span_m = math.sqrt(sum(
                (max(p[axis] for p in samples) - min(p[axis] for p in samples)) ** 2
                for axis in (1, 2, 3))) if samples else None
            hold_spans[name] = span_m
            if span_m is not None and span_m > max_hold_span:
                hold_errors.append(
                    f"{name} hold position span is {span_m:.3f} m; "
                    f"maximum is {max_hold_span:.3f} m (motion or unstable odometry)")
        if window_end - raw[-1][0] > 0.5:
            hold_errors.append("valid pose output is stale at the end of the recording")

        return_error = None
        if not hold_errors:
            return_error = round(
                1000.0 * dist(mean_point([p[1:] for p in start]),
                              mean_point([p[1:] for p in end])), 1)
        travelled = sum(dist(raw[i][1:], raw[i + 1][1:])
                        for i in range(len(raw) - 1))
        raw_points = [p[1:] for p in raw]
        raw_steps = biggest_jumps(raw)
        output_stats = pose_output_stats(raw)
        out["raw_vio_pose_the_one_px4_uses"] = {
            "samples": len(raw),
            "duration_s": round(span, 1),
            "rate_hz": round((len(raw) - 1) / span, 1) if span else 0.0,
            "path_travelled_m": round(travelled, 2),
            "net_displacement_mm": round(
                1000.0 * dist(raw_points[0], raw_points[-1]), 2),
            "rms_about_mean_mm": round(1000.0 * rms_radius(raw_points), 2),
            "max_step_mm": raw_steps[0]["jump_mm"] if raw_steps else 0.0,
            "largest_steps": raw_steps,
            "invalid_null_samples_excluded": invalid_null_samples,
            "recording_duration_s": round(recording_span, 1),
            "last_valid_pose_age_s": round(
                max(0.0, window_end - raw[-1][0]), 2),
            **output_stats,
            "start_hold_samples": len(start),
            "end_hold_samples": len(end),
            "start_hold_coverage_s": round(start_coverage, 2),
            "end_hold_coverage_s": round(end_coverage, 2),
            "max_hold_span_m": max_hold_span,
            "start_hold_span_m": hold_spans["start"],
            "end_hold_span_m": hold_spans["end"],
            "hold_comparison_valid": not hold_errors,
            "hold_comparison_error": "; ".join(hold_errors) or None,
            "return_to_origin_error_mm": return_error,
        }
    else:
        out["raw_vio_pose_the_one_px4_uses"] = {"samples": len(raw),
                                                "error": "insufficient data"}

    global_closures = sum(
        1 for event in node.closures if event[4] == "global_loop_closure")
    proximity_detections = len(node.closures) - global_closures
    out["loop_closures"] = {
        "count": len(node.closures),
        "global_count": global_closures,
        "proximity_count": proximity_detections,
        "events": [{"at_s": round(t - (raw[0][0] if raw else t), 1),
                    "ref_id": r, "matched_id": l,
                    "transform_mm": round(1000.0 * m, 1), "kind": kind}
                   for t, r, l, m, kind in node.closures[:20]],
    }

    if node.corrections:
        c = node.corrections
        mags = [math.sqrt(x * x + y * y + z * z) for _, x, y, z in c]
        # A correction that changes is the cross-stack proxy for a closure:
        # only the ROS mapper publishes rtabmap_msgs/Info, but both stacks
        # publish this topic, so this is what makes the two comparable.
        eps = 1e-4  # 0.1 mm
        updates = []
        for i in range(len(c) - 1):
            step = dist(c[i][1:], c[i + 1][1:])
            if step > eps:
                updates.append({"at_s": round(c[i + 1][0] - c[0][0], 1),
                                "delta_mm": round(1000.0 * step, 2)})
        out["map_to_odom_correction"] = {
            "samples": len(mags),
            "max_mm": round(1000.0 * max(mags), 1),
            "final_mm": round(1000.0 * mags[-1], 1),
            "ever_non_identity": bool(max(mags) > 1e-6),
            "correction_updates": len(updates),
            "largest_updates": sorted(
                updates, key=lambda u: -u["delta_mm"])[:5],
        }

    # Count raw-pose discontinuities explicitly. A large step can be an odom
    # reset, but when it follows a long output gap it may instead be all motion
    # that went unobserved during the outage arriving in one update. Either is
    # unsafe for PX4, but the pose samples alone cannot distinguish them.
    if len(raw) > 2:
        resets = []
        for i in range(len(raw) - 1):
            step = dist(raw[i][1:], raw[i + 1][1:])
            preceding_gap = raw[i + 1][0] - raw[i][0]
            near_origin = math.sqrt(sum(v * v for v in raw[i + 1][1:])) < 0.05
            if step > 0.25:
                resets.append({"at_s": round(raw[i + 1][0] - raw[0][0], 1),
                               "jump_mm": round(1000.0 * step, 1),
                               "preceding_output_gap_s": round(preceding_gap, 2),
                               "after_output_gap": preceding_gap >= 0.5,
                               "landed_near_origin": bool(near_origin)})
        out["vio_resets_and_jumps"] = {
            "count": len(resets),
            "events": resets[:10],
            "note": ("Steps over 250 mm in the RAW pose. A step after an "
                     "output gap may be unobserved physical motion rather than "
                     "an estimator reset, but PX4 sees either discontinuity "
                     "directly."),
        }

    if len(corr) > 2:
        out["corrected_pose_jumps"] = {
            "samples": len(corr),
            "largest": biggest_jumps(corr),
            "note": ("PX4 is fed the RAW pose precisely to avoid these steps; "
                     "this quantifies what it avoids."),
        }

    lost_fraction = 0.0
    if node.odom:
        lost = sum(1 for _, l, _, _ in node.odom if l)
        lost_fraction = lost / len(node.odom)
        inl = [i for _, _, i, _ in node.odom]
        loss_episodes = []
        loss_start = None
        for stamp, is_lost, _, _ in node.odom:
            if is_lost and loss_start is None:
                loss_start = stamp
            elif not is_lost and loss_start is not None:
                loss_episodes.append((loss_start, stamp - loss_start))
                loss_start = None
        if loss_start is not None:
            loss_episodes.append((loss_start,
                                  node.odom[-1][0] - loss_start))
        out["odometry_while_moving"] = {
            "samples": len(node.odom),
            "lost_updates": lost,
            "lost_fraction": round(lost_fraction, 3),
            "loss_episodes": len(loss_episodes),
            "longest_loss_s": round(
                max((duration for _, duration in loss_episodes), default=0.0),
                2),
            "loss_events": [
                {
                    "at_s": round(start - node.odom[0][0], 1),
                    "duration_s": round(duration, 2),
                }
                for start, duration in loss_episodes[:20]
            ],
            "inliers_mean": round(sum(inl) / len(inl)),
            "inliers_min": min(inl),
        }

    verdict = []
    corr = out.get("map_to_odom_correction", {})
    if not node.closures and not corr.get("correction_updates"):
        if lost_fraction:
            verdict.append(
                "NO closure and no correction update because raw odometry "
                "failed before the route completed; this run evaluates "
                "tracking robustness, not loop closure.")
        else:
            verdict.append(
                "NO closure and no correction update - the test proved "
                "nothing about closure; repeat with a more textured route.")
    elif not node.closures and corr.get("correction_updates"):
        verdict.append(
            f"{corr['correction_updates']} correction updates but no "
            "rtabmap_msgs/Info: expected on the DepthAI pipeline, whose SLAM "
            "is a host node that publishes no Info topic. Compare using "
            "correction_updates and corrected_pose_jumps.")
    if corr and not corr.get("ever_non_identity"):
        verdict.append("Correction stayed identity, so the corrected-pose "
                       "path is still unexercised.")
    rs = out.get("vio_resets_and_jumps", {}).get("count", 0)
    if rs:
        after_gaps = sum(
            1 for event in out["vio_resets_and_jumps"]["events"]
            if event.get("after_output_gap"))
        verdict.append(
            f"{rs} large discontinuities in the RAW pose ({after_gaps} after "
            "an output gap). PX4 would see these steps directly; the pose "
            "samples alone cannot identify them as resets versus unobserved "
            "motion during tracking loss.")
    if lost_fraction:
        loss = out["odometry_while_moving"]
        verdict.append(
            f"Tracking was lost for {100.0 * lost_fraction:.1f}% of odometry "
            f"updates across {loss['loss_episodes']} episodes (longest "
            f"{loss['longest_loss_s']:.2f} s). Invalid identity samples are "
            "excluded from pose metrics; "
            "with publish_null_when_lost=false the pose topic pauses instead. "
            "This still fails the moving flight gate until dropouts are fixed.")
    raw_result = out["raw_vio_pose_the_one_px4_uses"]
    final_pose_age = raw_result.get("last_valid_pose_age_s", 0.0)
    if final_pose_age > 0.5:
        verdict.append(
            f"The raw pose topic produced no valid pose for the final "
            f"{final_pose_age:.2f} s of the recording. PX4 would time out "
            "this external-vision input.")
    longest_pose_gap = raw_result.get("longest_output_gap_s", 0.0)
    if longest_pose_gap > 0.5:
        verdict.append(
            f"The raw pose stream had an internal output gap of "
            f"{longest_pose_gap:.2f} s. This independently fails the PX4 "
            "continuity gate even on stacks without OdomInfo diagnostics.")
    r = raw_result.get("return_to_origin_error_mm")
    if r is not None:
        verdict.append(f"Raw VIO returned to within {r} mm - this is the drift "
                       "PX4 would see, and loop closure cannot remove it.")
    out["verdict"] = verdict
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--hold-seconds", type=float, default=20.0,
                    help="Stationary window at each end used for the "
                         "return-to-origin comparison.")
    ap.add_argument("--max-hold-span", type=float, default=0.05,
                    help="Maximum position span in metres within each stationary hold.")
    ap.add_argument("--raw-pose-topic", default="/rtabmap/vio_pose")
    ap.add_argument(
        "--raw-odom-topic", default="",
        help=("nav_msgs/Odometry alternative for the raw estimator. Use "
              "'/rtabmap/vio_odometry' with ros_rtabmap_oak_v3.launch.py. "
              "Leave empty on the hybrid so the same pose is not counted "
              "twice."),
    )
    ap.add_argument("--pose-topic", default="/rtabmap/pose")
    ap.add_argument("--correction-topic", default="/rtabmap/odom_correction")
    ap.add_argument("--info-topic", default="/rtabmap3d/info")
    ap.add_argument("--odom-info-topic", default="/rtabmap/vio_odom_info")
    ap.add_argument("--label", default="",
                    help="Tag for the report, e.g. 'hybrid' or 'depthai'.")
    ap.add_argument("--out", default="")
    ap.add_argument(
        "--startup-timeout", type=float, default=10.0,
        help=("Wait this many seconds for the first raw pose before starting "
              "the timed test. Exit early if the estimator is not publishing."),
    )
    args = ap.parse_args()
    if not math.isfinite(args.max_hold_span) or args.max_hold_span <= 0:
        ap.error("max-hold-span must be finite and positive")

    rclpy.init()
    node = LoopClosureTest(args)
    raw_topic = args.raw_odom_topic or args.raw_pose_topic
    print(f"Waiting up to {args.startup_timeout:.0f} s for {raw_topic}. "
          "Keep the drone still.")
    startup_deadline = time.monotonic() + args.startup_timeout
    try:
        while (rclpy.ok() and not node.raw
               and time.monotonic() < startup_deadline):
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print("\nstopped before recording")

    if not node.raw:
        publisher_count = node.count_publishers(raw_topic)
        result = {
            "raw_vio_pose_the_one_px4_uses": {
                "samples": 0,
                "error": (
                    f"no messages received on {raw_topic} during the "
                    f"{args.startup_timeout:.1f} s startup preflight"),
                "publisher_count": publisher_count,
            },
            "loop_closures": {
                "count": 0,
                "global_count": 0,
                "proximity_count": 0,
                "events": [],
            },
            "verdict": [
                "NO RAW ODOMETRY RECEIVED. This is a launch, ROS domain, "
                "topic, or dead-publisher problem; it is not a route-texture "
                "or loop-closure result. Start exactly one estimator stack "
                "and verify the raw topic is moving before repeating."
            ],
        }
        if args.label:
            result = {"label": args.label, **result}
        text = json.dumps(result, indent=2)
        print(text)
        if args.out:
            with open(args.out, "w") as fh:
                fh.write(text + "\n")
            print(f"written to {args.out}")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(2)

    print(f"Recording {args.seconds:.0f} s. Hold still ~{args.hold_seconds:.0f} s, "
          "carry a closed loop, return to the marked start, hold still again.")
    recording_start = time.monotonic()
    end = recording_start + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print("\nstopped early")

    recording_end = time.monotonic()
    result = report(node, args.hold_seconds, recording_start, recording_end, args.max_hold_span)
    if args.label:
        result = {"label": args.label, **result}
    text = json.dumps(result, indent=2)
    print(text)
    if args.out:
        with open(args.out, "w") as fh:
            fh.write(text + "\n")
        print(f"written to {args.out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
