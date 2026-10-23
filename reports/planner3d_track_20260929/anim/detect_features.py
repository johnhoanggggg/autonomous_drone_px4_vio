"""Shi-Tomasi corners on every camera frame of a DepthAI-backend bag (/rtabmap/image/compressed).

Per-frame detection only (no tracking), same settings as render_tracking.py: up to 200 corners,
quality 0.01, 14 px apart, block 7. NOT the feature tracker that flew (no positions were recorded).
Writes <out.npz>: t (bag time, same t=0 as render_global.py), n (corners per frame), xy (all corners, px).
Run with PYTHONNOUSERSITE=1 (~/.local numpy 2.x breaks the system cv2); render_global.py --features calls it.
Usage (ROS sourced): python3 detect_features.py <bag_dir> <out.npz> [image_topic]
(image_topic defaults to the DepthAI one; animate3d.py --camera-feat passes /oak/left/image_rect/compressed.)
"""
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, out = sys.argv[1], sys.argv[2]
IMAGE = sys.argv[3] if len(sys.argv) > 3 else '/rtabmap/image/compressed'
MAX_FEAT, MIN_DIST = 200, 14

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
t0 = r.get_metadata().starting_time.nanoseconds
r.set_filter(rosbag2_py.StorageFilter(topics=[IMAGE]))
ts, ns, xy = [], [], []
while r.has_next():
    _, data, stamp = r.read_next()
    m = deserialize_message(data, get_message(types[IMAGE]))
    img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_GRAYSCALE)
    c = cv2.goodFeaturesToTrack(img, MAX_FEAT, 0.01, MIN_DIST, blockSize=7)
    c = np.zeros((0, 2), np.float32) if c is None else c.reshape(-1, 2)
    ts.append((stamp - t0) / 1e9)
    ns.append(len(c))
    xy.append(c)
np.savez(out, t=np.array(ts), n=np.array(ns), xy=np.vstack(xy) if xy else np.zeros((0, 2)))
print(f'{out}: {len(ts)} frames, median {int(np.median(ns))} corners')
