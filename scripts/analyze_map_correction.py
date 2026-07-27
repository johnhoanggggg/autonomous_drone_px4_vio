#!/usr/bin/env python3
"""Analyse a map_correction observation bag (handheld loop, no flight needed).

Usage (needs the ROS 2 environment, NOT .venv-mavlink):
    source /opt/ros/jazzy/setup.bash
    source /home/john/ros2_ws/install/setup.bash
    python3 scripts/analyze_map_correction.py ros2_ws/flight_logs/<bag_dir>

Answers the question the observation stage exists to answer: how big are the
loop-closure corrections in this room, and how do they compare to the noise
floor of the SLAM-vs-VIO difference when nothing is moving?

`analyze_flight.py` will not read these bags -- it requires an armed vehicle.
"""
import glob
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

D = math.degrees
STATIONARY_SPEED = 0.05  # m/s; below this the carrier is treated as still


def resolve(path):
    if os.path.isdir(path):
        hits = sorted(glob.glob(os.path.join(path, "*.mcap")))
        if not hits:
            sys.exit(f"no .mcap under {path}")
        return hits[-1]
    return path


def load(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msgs = {}
    while reader.has_next():
        topic, data, ts = reader.read_next()
        try:
            m = deserialize_message(data, get_message(types[topic]))
        except Exception:
            continue
        msgs.setdefault(topic, []).append((ts, m))
    for v in msgs.values():
        v.sort(key=lambda p: p[0])
    return msgs


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_series(entries):
    """-> t (s, bag clock), xyz (N,3), yaw (N,) radians."""
    t = np.array([ts for ts, _ in entries], dtype=np.float64) / 1e9
    xyz = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z]
                    for _, m in entries])
    yaw = np.array([yaw_of(m.pose.orientation) for _, m in entries])
    return t, xyz, yaw


def wrap_pi(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def pct(values, q):
    return float(np.percentile(values, q)) if len(values) else float("nan")


def main():
    if len(sys.argv) != 2:
        print(__doc__.strip(), file=sys.stderr)
        sys.exit(2)
    uri = resolve(sys.argv[1])
    print(f"=== {os.path.basename(uri)} ===")
    msgs = load(uri)
    if not msgs:
        sys.exit("bag contained no readable messages")

    t0 = min(v[0][0] for v in msgs.values())
    duration = (max(v[-1][0] for v in msgs.values()) - t0) / 1e9
    print(f"duration {duration:.1f} s")
    print("\n--- TOPICS ---")
    for topic in sorted(msgs):
        n = len(msgs[topic])
        print(f"  {topic:44s} {n:6d} msgs  {n / max(duration, 1e-9):6.1f} Hz")

    for required in ("/rtabmap/vio_pose", "/vio/map_correction_target"):
        if required not in msgs:
            sys.exit(f"bag is missing {required}; nothing to analyse")

    # ---- VIO tracking losses ----
    # RTAB-Map reports an exact-origin position with an all-0.5 quaternion while
    # its transform is lost. Those samples are not poses, and the 1-2 m "motion"
    # into and out of the sentinel is not motion -- counting either one turns
    # every derived number into nonsense.
    vt, vxyz, _ = pose_series(msgs["/rtabmap/vio_pose"])
    vq = np.array([[m.pose.orientation.w, m.pose.orientation.x,
                    m.pose.orientation.y, m.pose.orientation.z]
                   for _, m in msgs["/rtabmap/vio_pose"]])
    # Mirrors pose_rejection_reason() in the bridge: q and -q are the same
    # rotation, so accept either sign of the all-0.5 pattern.
    reset_q = np.minimum(np.max(np.abs(vq - 0.5), axis=1),
                         np.max(np.abs(vq + 0.5), axis=1)) <= 1e-3
    sentinel = np.all(np.abs(vxyz) < 1e-6, axis=1) & reset_q
    print("\n--- VIO TRACKING LOSS ---")
    if sentinel.any():
        idx = np.flatnonzero(sentinel)
        breaks = np.flatnonzero(np.diff(idx) > 1)
        spans = np.split(idx, breaks + 1)
        print(f"  *** {int(sentinel.sum())} reset-sentinel samples in "
              f"{len(spans)} dropout(s) ***")
        for s in spans:
            print(f"      t={vt[s[0]] - t0 / 1e9:6.2f}s .. {vt[s[-1]] - t0 / 1e9:6.2f}s  "
                  f"({vt[s[-1]] - vt[s[0]]:.2f}s, {len(s)} samples)")
        print("  Corrections solved near these times are artifacts of the odom frame")
        print("  restarting, NOT loop closures. Treat this run as suspect.")
    else:
        print("  none -- VIO tracked continuously")

    # ---- carrier motion, from the raw VIO track, sentinels excluded ----
    good = ~sentinel
    gvt, gvxyz = vt[good], vxyz[good]
    steps = np.linalg.norm(np.diff(gvxyz, axis=0), axis=1)
    dt = np.diff(gvt)
    # A gap where sentinels were removed is not a real inter-sample interval.
    real = dt < 0.5
    speed = np.divide(steps, dt, out=np.zeros_like(steps), where=dt > 0)
    extent = np.linalg.norm(gvxyz - gvxyz[0], axis=1).max()
    print("\n--- CARRIER MOTION (from /rtabmap/vio_pose, sentinels excluded) ---")
    print(f"  path length      {float(steps[real].sum()):6.2f} m")
    print(f"  max dist from start {extent:6.2f} m  <- the loop has to be big enough "
          f"for VIO to drift")
    print(f"  speed  median    {np.median(speed[real]):6.3f} m/s   "
          f"max {speed[real].max():6.3f} m/s")
    print(f"  net displacement {np.linalg.norm(gvxyz[-1] - gvxyz[0]):6.3f} m "
          f"(start->end; small means the loop was closed physically)")
    vt, vxyz = gvt, gvxyz

    # ---- the correction target: signal vs noise ----
    tt, txyz, tyaw = pose_series(msgs["/vio/map_correction_target"])
    dpos = np.linalg.norm(np.diff(txyz, axis=0), axis=1)
    dyaw = np.abs(wrap_pi(np.diff(tyaw)))

    # Label each target sample by whether the carrier was moving at that time.
    speed_at_target = np.interp(tt[1:], vt[1:], speed)
    still = speed_at_target < STATIONARY_SPEED
    moving = ~still

    print("\n--- CORRECTION TARGET: per-sample change ---")
    print(f"  {'':22s} {'n':>7s} {'median':>9s} {'p95':>9s} {'max':>9s}")
    for label, mask in (("stationary", still), ("moving", moving), ("all", np.ones_like(still))):
        if mask.sum() == 0:
            continue
        print(f"  {label + ' translation (cm)':22s} {int(mask.sum()):7d} "
              f"{pct(dpos[mask], 50) * 100:9.2f} {pct(dpos[mask], 95) * 100:9.2f} "
              f"{dpos[mask].max() * 100:9.2f}")
        print(f"  {label + ' yaw (deg)':22s} {int(mask.sum()):7d} "
              f"{D(pct(dyaw[mask], 50)):9.3f} {D(pct(dyaw[mask], 95)):9.3f} "
              f"{D(dyaw[mask].max()):9.3f}")

    noise_floor = pct(dpos[still], 99) if still.sum() else 0.0
    yaw_floor = pct(dyaw[still], 99) if still.sum() else 0.0
    print(f"\n  noise floor (stationary p99): {noise_floor * 100:.2f} cm / "
          f"{D(yaw_floor):.3f} deg per sample")

    # ---- closure events ----
    gate = max(noise_floor * 5.0, 0.02)
    events = np.flatnonzero(dpos > gate)
    print(f"\n--- CLOSURE EVENTS (per-sample jump > {gate * 100:.1f} cm) ---")
    if len(events) == 0:
        print("  none. Either no loop closed, or corrections never rose above the")
        print("  noise floor -- in which case there is nothing worth injecting.")
    else:
        print(f"  {len(events)} event(s)")
        print(f"  {'t(s)':>7s} {'jump(cm)':>9s} {'yaw(deg)':>9s} {'ramp@3cm/s(s)':>14s} {'moving':>7s}")
        for i in sorted(events, key=lambda i: -dpos[i])[:15]:
            print(f"  {tt[i + 1] - t0 / 1e9:7.2f} {dpos[i] * 100:9.2f} {D(dyaw[i]):9.3f} "
                  f"{dpos[i] / 0.03:14.1f} {'yes' if moving[i] else 'no':>7s}")

    # ---- total correction magnitude, and the gates ----
    mag = np.linalg.norm(txyz, axis=1)
    print("\n--- CORRECTION MAGNITUDE (target, absolute) ---")
    print(f"  translation  median {np.median(mag) * 100:6.2f} cm  "
          f"p95 {pct(mag, 95) * 100:6.2f} cm  max {mag.max() * 100:6.2f} cm")
    print(f"  yaw          median {D(np.median(np.abs(tyaw))):6.3f} deg  "
          f"max {D(np.abs(tyaw).max()):6.3f} deg")
    print(f"  gates are max_correction_m 0.5 / max_correction_yaw_deg 15 -- "
          f"{'NOT ' if mag.max() < 0.5 and D(np.abs(tyaw).max()) < 15 else ''}approached")

    # ---- did the applied correction keep up? ----
    if "/vio/map_correction" in msgs:
        at, axyz, _ = pose_series(msgs["/vio/map_correction"])
        target_at_applied = np.vstack([
            np.interp(at, tt, txyz[:, i]) for i in range(3)]).T
        pending = np.linalg.norm(target_at_applied - axyz, axis=1)
        never_caught = float((pending > 0.01).mean() * 100.0)
        print("\n--- APPLIED vs TARGET ---")
        print(f"  pending  median {np.median(pending) * 100:6.2f} cm  "
              f"p95 {pct(pending, 95) * 100:6.2f} cm  max {pending.max() * 100:6.2f} cm")
        print(f"  {never_caught:.0f}% of samples had >1 cm still to apply")
        if never_caught > 50.0:
            print("  The ramp spends most of its time chasing. Either the target is")
            print("  noise-dominated or the slew rate is too slow for this signal.")

    # ---- VIO health, so a bad run can be discarded ----
    if "/rtabmap/vio_feature_count" in msgs:
        fc = np.array([m.data for _, m in msgs["/rtabmap/vio_feature_count"]])
        print("\n--- VIO HEALTH ---")
        print(f"  features median {int(np.median(fc))}  min {int(fc.min())}  "
              f"samples below 160: {int((fc < 160).sum())}")


if __name__ == "__main__":
    main()
