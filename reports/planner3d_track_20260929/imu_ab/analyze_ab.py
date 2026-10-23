"""Compare replayed odometry runs (IMU aiding off/on) on one recorded carry.

Usage (ROS sourced): python3 analyze_ab.py <input_bag> <replay_dir>
Phases follow the recording cues: still 5-15 s, carry 15-105 s, back on the
start spot and still 105-end. Per run: frames that produced a pose, tracking
loss, odometry tilt from gravity while still (OAK accelerometer, y/z negated
for the driver's flipped IMU frame), and return drift (end hold vs start hold;
the drone was put back on the same spot, so ideally zero).
"""
import glob
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    while r.has_next():
        tp, d, _ = r.read_next()
        yield tp, deserialize_message(d, get_message(types[tp]))


def qm(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


in_bag, replay_dir = sys.argv[1], sys.argv[2]
import yaml
meta = yaml.safe_load(open(os.path.join(in_bag, 'metadata.yaml')))['rosbag2_bagfile_information']
frames = next(t['message_count'] for t in meta['topics_with_message_count']
              if t['topic_metadata']['name'] == '/oak/left/image_rect')
imu, static = [], {}
for tp, m in read(in_bag, ['/oak/imu/data', '/tf_static']):
    if tp == '/oak/imu/data':
        a, w = m.linear_acceleration, m.angular_velocity
        imu.append((stamp(m.header), a.x, -a.y, -a.z, math.sqrt(w.x ** 2 + w.y ** 2 + w.z ** 2)))
    elif tp == '/tf_static':
        for t in m.transforms:
            r = t.transform.rotation
            static[t.child_frame_id] = (t.header.frame_id, qm(r.x, r.y, r.z, r.w))
imu = np.array(imu)
t0 = imu[0, 0]
# Chain the static calibration from the IMU frame up to the camera body frame 'oak'.
frame = next(c for c in static if c.endswith('imu_frame'))
R_oi = np.eye(3)
while frame != 'oak':
    parent, R = static[frame]
    R_oi = R @ R_oi
    frame = parent
# Gravity (up) in the IMU frame and a quiet flag, on a 0.1 s grid.
grid = np.arange(imu[0, 0] + 0.5, imu[-1, 0], 0.1)
up, quiet = [], []
for g in grid:
    m = (imu[:, 0] > g - 0.5) & (imu[:, 0] <= g)
    a = imu[m, 1:4].mean(0)
    n = np.linalg.norm(a)
    up.append(a / n)
    quiet.append(abs(n - 9.81) < 0.3 and imu[m, 4].max() < 0.3)
up, quiet = np.array(up), np.array(quiet)


def phase(t):
    return 'still_start' if t < 15 else ('carry' if t < 105 else 'still_end')


print('input: %d camera frames, %.1f s' % (frames, imu[-1, 0] - t0))
print('%-8s %6s %6s %6s %8s | %-24s | %-24s | %s' % ('run', 'poses', 'lost%', 'lost%', 'lost%', 'tilt still start/end', 'tilt max (quiet)', 'return drift xy / z / yaw'))
print('%-8s %6s %6s %6s %8s |' % ('', 'frac', 'all', 'carry', 'end-hold'))
for out in sorted(glob.glob(os.path.join(replay_dir, '*.out'))):
    name = os.path.basename(out)[:-4]
    poses, info = [], []
    for tp, m in read(out, ['/rtabmap/vio_odometry', '/rtabmap/vio_odom_info_lite']):
        if tp == '/rtabmap/vio_odometry':
            p, q = m.pose.pose.position, m.pose.pose.orientation
            poses.append((stamp(m.header) - t0, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        else:
            info.append((stamp(m.header) - t0, bool(m.lost)))
    P = np.array(poses)
    I = np.array(info, dtype=float)
    lost = lambda lo, hi: 100 * I[(I[:, 0] >= lo) & (I[:, 0] < hi), 1].mean() if ((I[:, 0] >= lo) & (I[:, 0] < hi)).any() else float('nan')
    tilts = {}
    for t, *_x, qx, qy, qz, qw in P:
        gi = np.searchsorted(grid, t + t0) - 1
        if gi < 0 or not quiet[gi]:
            continue
        u = qm(qx, qy, qz, qw) @ R_oi @ up[gi]
        tilts.setdefault(phase(t), []).append(math.degrees(math.acos(np.clip(u[2], -1, 1))))
    start = P[(P[:, 0] >= 8) & (P[:, 0] < 15)]
    end = P[P[:, 0] >= 108]
    yaw = lambda r: math.degrees(math.atan2(2 * (r[7] * r[6] + r[4] * r[5]), 1 - 2 * (r[5] ** 2 + r[6] ** 2)))
    if len(start) and len(end):
        d = end[:, 1:4].mean(0) - start[:, 1:4].mean(0)
        dyaw = (yaw(end[-1]) - yaw(start[-1]) + 180) % 360 - 180
        drift = '%5.1f cm / %+5.1f cm / %+5.1f deg' % (100 * math.hypot(d[0], d[1]), 100 * d[2], dyaw)
    else:
        drift = 'no pose in %s hold' % ('start' if not len(start) else 'end')
    ts = lambda k: '%.2f' % np.mean(tilts[k]) if k in tilts else '  - '
    allq = [x for v in tilts.values() for x in v]
    print('%-8s %6.2f %6.1f %6.1f %8.1f | %6s / %-15s | %-24s | %s' % (
        name, len(P) / frames, lost(0, 999), lost(15, 105), lost(108, 999),
        ts('still_start'), ts('still_end'), '%.2f' % max(allq) if allq else '-', drift))
