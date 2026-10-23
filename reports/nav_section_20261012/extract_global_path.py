"""Dump a 2D (global planner) flight bag to the pickle plot_path_map.py reads.

Keys: grids [(t, G, res, ox, oy)], paths [(t, Nx2)], body (Nx3: t, x, y), t_arm,
states [(t, state)] (flight adapter state changes). Times are seconds from bag start.
Usage (ROS sourced): python3 extract_global_path.py <bag_dir> <out.pkl>
"""
import pickle
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, out = sys.argv[1], sys.argv[2]
meta = yaml.safe_load(open(Path(bag) / 'metadata.yaml'))['rosbag2_bagfile_information']
t0 = meta['starting_time']['nanoseconds_since_epoch']

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ['/rtabmap/grid', '/planner/path', '/rtabmap/body_pose', '/planner/flight/status',
        '/fmu/out/vehicle_status_v1', '/fmu/out/vehicle_status']
want = [w for w in want if w in types]
r.set_filter(rosbag2_py.StorageFilter(topics=want))
cls = {w: get_message(types[w]) for w in want}

grids, paths, body, states, t_arm = [], [], [], [], None
last_state = None
while r.has_next():
    tp, data, tn = r.read_next()
    t = (tn - t0) / 1e9
    m = deserialize_message(data, cls[tp])
    if tp == '/rtabmap/grid':
        info = m.info
        G = np.asarray(m.data, np.int8).reshape(info.height, info.width)
        grids.append((t, G, info.resolution, info.origin.position.x, info.origin.position.y))
    elif tp == '/planner/path' and len(m.poses):
        paths.append((t, np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses])))
    elif tp == '/rtabmap/body_pose':
        body.append([t, m.pose.position.x, m.pose.position.y])
    elif tp == '/planner/flight/status':
        s = m.data.split()[0] if m.data else ''
        if s != last_state:
            states.append((t, s))
            last_state = s
    elif tp.startswith('/fmu/out/vehicle_status') and t_arm is None and m.arming_state == 2:
        t_arm = t

# keep every 5th grid plus the last one (the plots only need the late maps)
grids = grids[::5] + ([grids[-1]] if grids and len(grids) % 5 != 1 else [])
pickle.dump(dict(grids=grids, paths=paths, body=np.array(body), states=states, t_arm=t_arm),
            open(out, 'wb'))
print(f'{out}: {len(grids)} grids, {len(paths)} paths, {len(body)} poses, t_arm {t_arm}, '
      f'{len(states)} state changes')
