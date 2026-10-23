"""What is around and under the drone along its track (last OctoMap of the bag).
Usage (ROS sourced): python3 surroundings.py <bag> <pkl>
Per 0.1 m of map x: drone height, the highest obstacle under the drone
(|dy| < 0.2 m), clearance below, and the nearest obstacle left/right at
drone height +-0.15 m.
"""
import pickle
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, '/home/john/autonomous_drone_px4_vio/scripts')
from octomap_binary import decode_binary, expand  # noqa: E402

bag, pkl = sys.argv[1], sys.argv[2]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=['/rtabmap/octomap']))
msg = None
while r.has_next():
    tp, d, _ = r.read_next()
    msg = deserialize_message(d, get_message(types[tp]))
c, s = decode_binary(msg.data, msg.resolution)
V = expand(c, s, msg.resolution)
D = pickle.load(open(pkl, 'rb'))
st = D['/planner3d/flight/status']
t0 = next(t for t, v in st if v.get('state') == 'ROUTE_START')
B = np.array([v['body_map'] for t, v in st if t >= t0 and v.get('body_map') and v.get('state') in ('FOLLOWING', 'HOLD', 'YAW_ALIGN')])
floor = np.percentile(V[:, 2], 5)
print('floor ~%.2f m' % floor)
print('map x    drone y  z  | top of obstacle under drone  gap below | nearest left  right (at drone height)')
for lo in np.arange(0.0, 2.0, 0.1):
    m = (B[:, 0] >= lo) & (B[:, 0] < lo + 0.1)
    if m.sum() < 2:
        continue
    y, z = B[m, 1].mean(), B[m, 2].mean()
    col = V[(np.abs(V[:, 0] - (lo + 0.05)) < 0.1) & (np.abs(V[:, 1] - y) < 0.2) & (V[:, 2] < z - 0.05)]
    top = col[:, 2].max() if len(col) else np.nan
    band = V[(np.abs(V[:, 0] - (lo + 0.05)) < 0.15) & (np.abs(V[:, 2] - z) < 0.15)]
    left = band[band[:, 1] > y][:, 1].min() - y if (band[:, 1] > y).any() else np.nan
    right = y - band[band[:, 1] < y][:, 1].max() if (band[:, 1] < y).any() else np.nan
    print('%.1f-%.1f  %+5.2f %4.2f | %5.2f %9.2f | %8.2f %6.2f' % (lo, lo + 0.1, y, z, top, z - top, left, right))
