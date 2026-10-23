"""Clearance of the drone and of its active path to the final OctoMap in a flight bag.

Usage (ROS sourced): python3 clearance.py <bag_dir> <dump.pkl>
Uses the last /rtabmap/octomap (the most complete map); obstacle voxels only (above the floor band).
"""
import pickle
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from octomap_msgs.msg import Octomap

sys.path.insert(0, '/home/john/autonomous_drone_px4_vio/scripts')
from octomap_binary import decode_binary, expand  # noqa: E402

bag, pkl = sys.argv[1], sys.argv[2]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/rtabmap/octomap']))
last = None
while r.has_next():
    last = r.read_next()[1]
msg = deserialize_message(last, Octomap)
c, s = decode_binary(msg.data, msg.resolution)
V = expand(c, s, msg.resolution)
D = pickle.load(open(pkl, 'rb'))
st = D['/planner3d/flight/status']
floor = np.percentile(V[:, 2], 5) + 0.10
V = V[V[:, 2] > floor]
half = msg.resolution / 2


def clearance(p):
    return float(np.min(np.linalg.norm(V - p, axis=1))) - half


def seg_clearance(P, step=0.03):
    best = 9.0
    for a, b in zip(P[:-1], P[1:]):
        n = max(1, int(np.linalg.norm(b - a) / step))
        for k in range(n + 1):
            best = min(best, clearance(a + (b - a) * k / n))
    return best


rows = []
paths = [(t, np.array(p)) for t, p in D['/planner3d/reference_path'] if len(p) >= 2]
for t, v in st:
    if v.get('state') not in ('FOLLOWING', 'HOLD', 'YAW_ALIGN') or not v.get('body_map'):
        continue
    rows.append((t, clearance(np.array(v['body_map'])), v['state']))
R = [(t, c) for t, c, s in rows]
T = np.array([x[0] for x in R])
C = np.array([x[1] for x in R])
pc = []
for t, P in paths:
    if T[0] <= t <= T[-1]:
        pc.append((t, seg_clearance(P[1:] if len(P) > 2 else P)))
pc = np.array(pc)
print('floor cut at z > %.2f; %d obstacle voxels' % (floor, len(V)))
print('drone clearance: min %.2f m at t=%.1f | p5 %.2f | time below 0.30 m: %.1f s, below 0.25 m: %.1f s' % (
    C.min(), T[np.argmin(C)], np.percentile(C, 5), 0.1 * (C < 0.30).sum(), 0.1 * (C < 0.25).sum()))
print('path clearance (excluding its start at the drone): min %.2f m at t=%.1f, p5 %.2f' % (pc[:, 1].min(), pc[np.argmin(pc[:, 1]), 0], np.percentile(pc[:, 1], 5)))
low = T[C < 0.30]
if len(low):
    spans = []
    s0 = p = low[0]
    for t in low[1:]:
        if t - p > 0.25:
            spans.append((s0, p))
            s0 = t
        p = t
    spans.append((s0, p))
    print('below 0.30 m during:', ', '.join('%.1f-%.1f' % sp for sp in spans))
