#!/usr/bin/env python3
"""Analyse a native or legacy map-correction bag (handheld loop, no flight needed).

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


def quaternion_series(entries):
    """Return normalized message quaternions as an (N,4) wxyz array."""
    q = np.array([
        [m.pose.orientation.w, m.pose.orientation.x,
         m.pose.orientation.y, m.pose.orientation.z]
        for _, m in entries
    ], dtype=np.float64)
    norms = np.linalg.norm(q, axis=1)
    return np.divide(q, norms[:, None], out=np.zeros_like(q), where=norms[:, None] > 0)


def rotate_vectors(quaternions, vectors):
    """Rotate row vectors by matching wxyz quaternions."""
    axis = quaternions[:, 1:]
    twice_cross = 2.0 * np.cross(axis, vectors)
    return (
        vectors
        + quaternions[:, :1] * twice_cross
        + np.cross(axis, twice_cross)
    )


def multiply_quaternions(first, second):
    """Hamilton product for matching rows of normalized wxyz quaternions."""
    aw, av = first[:, 0], first[:, 1:]
    bw, bv = second[:, 0], second[:, 1:]
    return np.column_stack((
        aw * bw - np.sum(av * bv, axis=1),
        aw[:, None] * bv + bw[:, None] * av + np.cross(av, bv),
    ))


def quaternion_yaws(quaternions):
    w, x, y, z = quaternions.T
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_roll_pitch(quaternions):
    w, x, y, z = quaternions.T
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    return roll, pitch


def header_times(entries):
    """Return ROS header stamps in seconds for stamped pose messages."""
    return np.array([
        m.header.stamp.sec + m.header.stamp.nanosec * 1.0e-9
        for _, m in entries
    ], dtype=np.float64)


def nearest_indices(reference_times, query_times):
    """Index of the nearest ordered reference time for every query time."""
    indices = np.searchsorted(reference_times, query_times)
    indices = np.clip(indices, 1, len(reference_times) - 1)
    lower = indices - 1
    use_lower = (
        np.abs(reference_times[lower] - query_times)
        < np.abs(reference_times[indices] - query_times)
    )
    return np.where(use_lower, lower, indices)


def correction_from_pose_pairs(vio_entries, slam_entries):
    """Reconstruct planar map<-odom corrections as the legacy node does."""
    vio_stamps = header_times(vio_entries)
    slam_stamps = header_times(slam_entries)
    _, vio_xyz, vio_yaw = pose_series(vio_entries)
    _, slam_xyz, slam_yaw = pose_series(slam_entries)
    pairs = nearest_indices(vio_stamps, slam_stamps)
    pair_dt = np.abs(vio_stamps[pairs] - slam_stamps)
    valid = pair_dt <= 0.15

    pairs = pairs[valid]
    slam_xyz = slam_xyz[valid]
    correction_yaw = wrap_pi(slam_yaw[valid] - vio_yaw[pairs])
    c = np.cos(correction_yaw)
    s = np.sin(correction_yaw)
    vx = vio_xyz[pairs, 0]
    vy = vio_xyz[pairs, 1]
    correction_xyz = np.column_stack((
        slam_xyz[:, 0] - (c * vx - s * vy),
        slam_xyz[:, 1] - (s * vx + c * vy),
        slam_xyz[:, 2] - vio_xyz[pairs, 2],
    ))
    return slam_stamps[valid], correction_xyz, correction_yaw, pair_dt[valid]


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

    native_topic = "/rtabmap/odom_correction"
    legacy_target_topic = "/vio/map_correction_target"
    correction_topic = (
        native_topic if native_topic in msgs else legacy_target_topic
    )
    for required in ("/rtabmap/vio_pose", correction_topic):
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
    all_vio_header_times = header_times(msgs["/rtabmap/vio_pose"])
    all_vio_valid = ~sentinel
    print("\n--- VIO TRACKING LOSS ---")
    if sentinel.any():
        idx = np.flatnonzero(sentinel)
        breaks = np.flatnonzero(np.diff(idx) > 1)
        spans = np.split(idx, breaks + 1)
        print(f"  *** {int(sentinel.sum())} reset-sentinel samples in "
              f"{len(spans)} dropout(s) ***")
        suspect_dropout = False
        for s in spans:
            print(f"      t={vt[s[0]] - t0 / 1e9:6.2f}s .. {vt[s[-1]] - t0 / 1e9:6.2f}s  "
                  f"({vt[s[-1]] - vt[s[0]]:.2f}s, {len(s)} samples)")
            before = s[0] - 1
            after = s[-1] + 1
            if before >= 0 and after < len(vt):
                recovery_position = np.linalg.norm(vxyz[after] - vxyz[before])
                recovery_yaw = abs(wrap_pi(yaw_of(
                    msgs["/rtabmap/vio_pose"][after][1].pose.orientation
                ) - yaw_of(
                    msgs["/rtabmap/vio_pose"][before][1].pose.orientation
                )))
                print(f"        recovery across dropout {recovery_position * 100:.1f} cm / "
                      f"{D(recovery_yaw):.2f} deg")
                if recovery_position > 0.5 or recovery_yaw > math.radians(45.0):
                    suspect_dropout = True
            else:
                suspect_dropout = True
            if vt[s[-1]] - vt[s[0]] >= 0.5:
                suspect_dropout = True
        if suspect_dropout:
            print("  Sustained/discontinuous loss can reset the odom frame; corrections")
            print("  around it are not trustworthy. Treat this run as suspect.")
        else:
            print("  Brief invalid sample recovered continuously; reject that sample but")
            print("  do not classify later native corrections as reset artifacts.")
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

    # ---- native correction (or the legacy target in old bags): signal vs noise ----
    tt, txyz, tyaw = pose_series(msgs[correction_topic])
    dpos = np.linalg.norm(np.diff(txyz, axis=0), axis=1)
    dyaw = np.abs(wrap_pi(np.diff(tyaw)))

    # Label each target sample by whether the carrier was moving at that time.
    speed_at_target = np.interp(tt[1:], vt[1:], speed)
    still = speed_at_target < STATIONARY_SPEED
    moving = ~still

    print(f"\n--- CORRECTION SOURCE {correction_topic}: per-sample change ---")
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
    event_label = "NATIVE CORRECTION EVENTS" if correction_topic == native_topic else "LEGACY TARGET EVENTS"
    print(f"\n--- {event_label} (per-sample jump > {gate * 100:.1f} cm) ---")
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
    print("\n--- CORRECTION MAGNITUDE (absolute) ---")
    print(f"  translation  median {np.median(mag) * 100:6.2f} cm  "
          f"p95 {pct(mag, 95) * 100:6.2f} cm  max {mag.max() * 100:6.2f} cm")
    print(f"  yaw          median {D(np.median(np.abs(tyaw))):6.3f} deg  "
          f"max {D(np.abs(tyaw).max()):6.3f} deg")
    print(f"  gates are max_correction_m 0.5 / max_correction_yaw_deg 15 -- "
          f"{'NOT ' if mag.max() < 0.5 and D(np.abs(tyaw).max()) < 15 else ''}approached")

    # ---- native DepthAI correction, or what old bags can prove without it ----
    print("\n--- RTAB-MAP NATIVE MAP<-ODOM CORRECTION ---")
    if native_topic in msgs:
        nt, nxyz, nyaw = pose_series(msgs[native_topic])
        native_message_quaternions = np.array([
            [m.pose.orientation.w, m.pose.orientation.x,
             m.pose.orientation.y, m.pose.orientation.z]
            for _, m in msgs[native_topic]
        ])
        native_unit_quaternion = (
            np.abs(np.linalg.norm(native_message_quaternions, axis=1) - 1.0)
            <= 1.0e-3
        )
        native_quaternions = quaternion_series(msgs[native_topic])
        native_header_t = header_times(msgs[native_topic])
        native_vio_indices = nearest_indices(all_vio_header_times, native_header_t)
        healthy_native = (
            all_vio_valid[native_vio_indices]
            & native_unit_quaternion
            & (
                np.abs(
                    all_vio_header_times[native_vio_indices] - native_header_t
                ) < 0.2
            )
        )
        print(f"  recorded {len(nt)} native corrections at {len(nt) / max(duration, 1e-9):.1f} Hz")
        if not native_unit_quaternion.all():
            invalid_native_time = nt[~native_unit_quaternion] - t0 / 1e9
            print(f"  rejected {len(invalid_native_time)} non-unit native quaternion(s) "
                  f"at t={invalid_native_time.min():.2f}..{invalid_native_time.max():.2f}s")
        if legacy_target_topic in msgs:
            ltt, ltxyz, ltyaw = pose_series(msgs[legacy_target_topic])
            compare_t = nt[healthy_native]
            target_xyz = np.column_stack([
                np.interp(compare_t, ltt, ltxyz[:, axis]) for axis in range(3)
            ])
            target_yaw = np.interp(compare_t, ltt, np.unwrap(ltyaw))
            position_error = np.linalg.norm(nxyz[healthy_native] - target_xyz, axis=1)
            yaw_error = np.abs(wrap_pi(nyaw[healthy_native] - target_yaw))
            print(f"  native vs legacy /vio target during valid VIO ({len(compare_t)} samples):")
            print(f"    translation error median {pct(position_error, 50) * 100:.2f} cm  "
                  f"p95 {pct(position_error, 95) * 100:.2f} cm  "
                  f"max {position_error.max() * 100:.2f} cm")
            print(f"    yaw error         median {D(pct(yaw_error, 50)):.3f} deg  "
                  f"p95 {D(pct(yaw_error, 95)):.3f} deg  "
                  f"max {D(yaw_error.max()):.3f} deg")
        else:
            print("  legacy /vio target absent, as expected for the native-only pipeline")
        native_roll, native_pitch = quaternion_roll_pitch(native_quaternions)
        print(f"  native tilt correction max roll {D(np.max(np.abs(native_roll[healthy_native]))):.2f} deg / "
              f"pitch {D(np.max(np.abs(native_pitch[healthy_native]))):.2f} deg")

        native_step_position = np.linalg.norm(np.diff(nxyz, axis=0), axis=1)
        native_step_yaw = np.abs(wrap_pi(np.diff(nyaw)))
        healthy_steps = healthy_native[:-1] & healthy_native[1:]
        native_events = np.flatnonzero(
            healthy_steps
            & (
                (native_step_position > 0.02)
                | (native_step_yaw > math.radians(0.2))
            )
        )
        print("  native optimization steps above 2 cm or 0.2 deg:")
        if len(native_events):
            for index in native_events:
                print(f"    t={nt[index + 1] - t0 / 1e9:6.2f}s  "
                      f"step={native_step_position[index] * 100:5.2f} cm / "
                      f"{D(native_step_yaw[index]):5.2f} deg  "
                      f"absolute={np.linalg.norm(nxyz[index + 1]) * 100:5.2f} cm / "
                      f"{D(nyaw[index + 1]):+5.2f} deg")
        else:
            print("    none")

        if "/rtabmap/pose" in msgs:
            # Validate direction independently: C_native * raw should reproduce
            # the corrected pose. Also search a few adjacent raw samples. The
            # pre-fix host bridge could read the previous passthrough sample
            # because DepthAI sends transform just before passthroughOdom.
            _, raw_xyz, _ = pose_series(msgs["/rtabmap/vio_pose"])
            _, slam_xyz, slam_yaw = pose_series(msgs["/rtabmap/pose"])
            raw_quaternions = quaternion_series(msgs["/rtabmap/vio_pose"])
            raw_t = all_vio_header_times
            slam_t = header_times(msgs["/rtabmap/pose"])
            relation_native_t = native_header_t
            raw_base = nearest_indices(raw_t, slam_t)
            native_indices = nearest_indices(relation_native_t, slam_t)
            slam_vio_indices = nearest_indices(all_vio_header_times, slam_t)
            healthy_slam = (
                all_vio_valid[slam_vio_indices]
                & native_unit_quaternion[native_indices]
                & (np.abs(all_vio_header_times[slam_vio_indices] - slam_t) < 0.2)
                & (np.abs(relation_native_t[native_indices] - slam_t) < 0.2)
            )
            candidates = []
            for offset in range(-2, 3):
                raw_indices = np.clip(raw_base + offset, 0, len(raw_t) - 1)
                correction_q = native_quaternions[native_indices]
                predicted_xyz = (
                    rotate_vectors(correction_q, raw_xyz[raw_indices])
                    + nxyz[native_indices]
                )
                xy_error = np.hypot(
                    predicted_xyz[:, 0] - slam_xyz[:, 0],
                    predicted_xyz[:, 1] - slam_xyz[:, 1],
                )[healthy_slam]
                predicted_orientation = multiply_quaternions(
                    correction_q, raw_quaternions[raw_indices]
                )
                predicted_yaw = quaternion_yaws(predicted_orientation)
                relation_yaw_error = np.abs(
                    wrap_pi(predicted_yaw - slam_yaw)
                )[healthy_slam]
                score = pct(xy_error, 95) + pct(relation_yaw_error, 95)
                candidates.append((score, offset, raw_indices, xy_error, relation_yaw_error))
            _, offset, raw_indices, xy_error, relation_yaw_error = min(candidates)
            time_offset = raw_t[raw_indices][healthy_slam] - slam_t[healthy_slam]
            print("  full transform relation C_native * raw_pose -> corrected_pose:")
            print(f"    best raw-frame offset {offset:+d}; median stamp offset "
                  f"{np.median(time_offset) * 1000:.1f} ms")
            print(f"    XY error median {pct(xy_error, 50) * 100:.2f} cm  "
                  f"p95 {pct(xy_error, 95) * 100:.2f} cm  "
                  f"max {xy_error.max() * 100:.2f} cm")
            print(f"    yaw error median {D(pct(relation_yaw_error, 50)):.3f} deg  "
                  f"p95 {D(pct(relation_yaw_error, 95)):.3f} deg  "
                  f"max {D(relation_yaw_error.max()):.3f} deg")
            if offset:
                print("    Non-zero frame offset exposes raw/corrected host pairing skew.")

        invalid_native = ~healthy_native
        if invalid_native.any():
            invalid_magnitude = np.linalg.norm(nxyz[invalid_native], axis=1)
            invalid_yaw = np.abs(wrap_pi(nyaw[invalid_native]))
            print("  while VIO was invalid/unpaired:")
            print(f"    native max {invalid_magnitude.max():.2f} m / "
                  f"{D(invalid_yaw.max()):.2f} deg -- must be rejected, not followed")
    elif "/rtabmap/pose" in msgs:
        print("  not recorded (the bridge did not expose it for this bag)")
        derived_t, derived_xyz, derived_yaw, pair_dt = correction_from_pose_pairs(
            msgs["/rtabmap/vio_pose"], msgs["/rtabmap/pose"])
        # The target is deadbanded, so compare it with the most recent pose-pair
        # solution rather than interpolating through correction steps.
        target_indices = np.searchsorted(derived_t, tt, side="right") - 1
        valid = target_indices >= 0
        indices = target_indices[valid]
        position_error = np.linalg.norm(txyz[valid] - derived_xyz[indices], axis=1)
        yaw_error = np.abs(wrap_pi(tyaw[valid] - derived_yaw[indices]))
        print("  reconstructed map<-odom from corrected_pose * inverse(raw_pose)")
        print(f"    paired {len(derived_t)} poses; max timestamp separation "
              f"{pair_dt.max() * 1000:.1f} ms")
        print("    reconstructed vs deadbanded /vio target:")
        print(f"      translation error median {pct(position_error, 50) * 100:.2f} cm  "
              f"p95 {pct(position_error, 95) * 100:.2f} cm")
        print(f"      yaw error         median {D(pct(yaw_error, 50)):.3f} deg  "
              f"p95 {D(pct(yaw_error, 95)):.3f} deg")
        if pct(position_error, 95) <= 0.02 and D(pct(yaw_error, 95)) <= 0.5:
            print("  This validates the transform convention and legacy estimator, but")
            print("  cannot validate the native stream because it was not recorded.")
        else:
            print("  The reconstructed and recorded corrections do not agree closely;")
            print("  this bag cannot validate the transform (check VIO resets/pairing).")
        print("  Only a new recording can compare DepthAI's native stream directly.")

    # ---- did the applied correction keep up? ----
    if "/vio/map_correction" in msgs and legacy_target_topic in msgs:
        ltt, ltxyz, _ = pose_series(msgs[legacy_target_topic])
        at, axyz, _ = pose_series(msgs["/vio/map_correction"])
        target_at_applied = np.vstack([
            np.interp(at, ltt, ltxyz[:, i]) for i in range(3)]).T
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
