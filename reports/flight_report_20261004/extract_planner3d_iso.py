"""Dump a planner3d bag to the pickle render_octomap_iso.py reads, plus route/goal.

Map = last /rtabmap/octomap before the end of the flown track, cropped to the
track's surroundings. Keys: vox, res, body ([t,x,y,z]), states, routes, goal, ground_z.
Usage (ROS sourced): python3 extract_planner3d_iso.py <bag_dir> <out.pkl> [margin_m]
"""
import json
import pickle
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, '/home/john/autonomous_drone_px4_vio/scripts')
from octomap_binary import decode_binary, expand  # noqa: E402

bag, out = sys.argv[1], sys.argv[2]
MARGIN = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ['/rtabmap/octomap', '/planner3d/flight/status', '/planner3d/reference_path', '/waypoint/clicked']
r.set_filter(rosbag2_py.StorageFilter(topics=[w for w in want if w in types]))
t0 = r.get_metadata().starting_time.nanoseconds
maps, status, routes, goals = [], [], [], []
while r.has_next():
    tp, d, ns = r.read_next()
    t = (ns - t0) / 1e9
    m = deserialize_message(d, get_message(types[tp]))
    if tp == '/rtabmap/octomap':
        maps.append((t, bytes(m.data), m.resolution))
    elif tp == '/planner3d/flight/status':
        try:
            status.append((t, json.loads(m.data)))
        except ValueError:
            pass
    elif tp == '/planner3d/reference_path':
        routes.append((t, np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in m.poses])))
    elif tp == '/waypoint/clicked':
        goals.append((t, np.array([m.point.x, m.point.y, m.point.z])))

body = np.array([[t, *v['body_map']] for t, v in status if v.get('body_map')])
t_end = body[-1, 0]
states = [(t, v.get('state')) for t, v in status]
ground_z = next((v['ground_map_z'] for t, v in status if v.get('ground_map_z') is not None), body[0, 3])
row = [m for m in maps if m[0] <= t_end + 0.5][-1]
c, s = decode_binary(row[1], row[2])
v = expand(c, s, row[2])
lo = body[:, 1:4].min(axis=0) - MARGIN
hi = body[:, 1:4].max(axis=0) + MARGIN
v = v[(v[:, 0] > lo[0]) & (v[:, 0] < hi[0]) & (v[:, 1] > lo[1]) & (v[:, 1] < hi[1])
      & (v[:, 2] > ground_z - 0.15) & (v[:, 2] < hi[2] + 0.3)]
goal = goals[-1][1] if goals else None
pickle.dump(dict(vox=v, res=row[2], body=body, states=states, routes=routes, goal=goal, ground_z=ground_z,
                 map_t=row[0]), open(out, 'wb'))
seen = []
for t, st in states:
    if not seen or seen[-1][1] != st:
        seen.append((round(t, 1), st))
print('voxels', len(v), 'res', row[2], 'map_t', round(row[0], 1), 'track', round(body[0, 0], 1), '-', round(t_end, 1))
print('goal', goal, 'routes', len(routes), 'ground_z', ground_z)
print('states', seen)
print('track lo/hi', body[:, 1:4].min(0).round(2), body[:, 1:4].max(0).round(2))
