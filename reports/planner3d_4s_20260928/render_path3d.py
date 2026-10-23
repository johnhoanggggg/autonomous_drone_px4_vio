"""Path render for a 3D planner flight: map voxels (top + side), planned route, command, drone track, holds.

Usage (ROS sourced): python3 render_path3d.py <bag_dir> <out.png> [t_end_bag]
The map is the last /rtabmap/octomap at or before t_end (default: bag end).
"""
import json
import sys

import numpy as np
sys.path.append('/home/john/autonomous_drone_px4_vio/.venv-mavlink/lib/python3.12/site-packages')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, '/home/john/autonomous_drone_px4_vio/scripts')
from octomap_binary import decode_binary, expand  # noqa: E402

bag, out = sys.argv[1], sys.argv[2]
t_end = float(sys.argv[3]) if len(sys.argv) > 3 else 1e9
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = {'/rtabmap/octomap', '/planner3d/flight/status', '/planner3d/reference_path', '/waypoint/clicked'}
r.set_filter(rosbag2_py.StorageFilter(topics=list(want & set(types))))
t0 = r.get_metadata().starting_time.nanoseconds; tree = None; st = []; paths = []; goal = None
while r.has_next():
    tp, d, ns = r.read_next()
    t = (ns - t0)/1e9
    if t > t_end: continue
    m = deserialize_message(d, get_message(types[tp]))
    if tp == '/rtabmap/octomap': tree = (t, m)
    elif tp == '/planner3d/flight/status': st.append((t, json.loads(m.data)))
    elif tp == '/planner3d/reference_path': paths.append((t, np.array([(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in m.poses])))
    elif tp == '/waypoint/clicked': goal = (m.point.x, m.point.y)
mt, msg = tree
c, s = decode_binary(msg.data, msg.resolution); vox = expand(c, s, msg.resolution)
air = st and [v for _, v in st if v.get('state') in ('FOLLOWING', 'HOLD', 'YAW_ALIGN', 'GOAL_REACHED')]
body = np.array([(t, *v['body_map']) for t, v in st if v.get('body_map') and v.get('state') not in ('ARM_INHIBITED',)])
cmd = np.array([(t, *v['command_map']) for t, v in st if v.get('command_map')])
route_t = [t for t, v in st if v.get('state') == 'ROUTE_START']
t_route = route_t[0] if route_t else body[0, 0]
zmin_air, zmax_air = 0.12, max(1.0, body[:, 3].max() + 0.3)
lo, hi = body[:, 1:3].min(0) - 0.8, body[:, 1:3].max(0) + 0.8
if goal: lo = np.minimum(lo, np.array(goal) - 0.5); hi = np.maximum(hi, np.array(goal) + 0.5)
inb = (vox[:, 0] > lo[0]) & (vox[:, 0] < hi[0]) & (vox[:, 1] > lo[1]) & (vox[:, 1] < hi[1])
v = vox[inb & (vox[:, 2] > zmin_air) & (vox[:, 2] < zmax_air)]
fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 11), gridspec_kw={'height_ratios': [2.2, 1]})
sc = a1.scatter(v[:, 0], v[:, 1], c=v[:, 2], s=6, marker='s', cmap='Greys', vmin=zmin_air - 0.3, vmax=zmax_air, alpha=.8)
plt.colorbar(sc, ax=a1, label='obstacle voxel height (m, map)', shrink=.7)
for t, p in paths:
    if t >= t_route and len(p): a1.plot(p[:, 0], p[:, 1], c='tab:orange', lw=.5, alpha=.25)
first = next((p for t, p in paths if t >= t_route), None)
if first is not None: a1.plot(first[:, 0], first[:, 1], c='tab:orange', lw=2, ls='--', label='first route')
fl = body[:, 0] >= t_route - 3
a1.plot(cmd[:, 1], cmd[:, 2], c='tab:red', lw=.8, alpha=.7, label='command')
tr = a1.scatter(body[fl, 1], body[fl, 2], c=body[fl, 0], s=10, cmap='viridis', zorder=3, label='drone (bag s)')
plt.colorbar(tr, ax=a1, label='bag time (s)', shrink=.7)
prev = None
for t, sv in st:
    tag = sv.get('fault') if sv.get('state') == 'HOLD' else None
    if tag and tag != prev and sv.get('body_map') and t >= t_route:
        a1.plot(*sv['body_map'][:2], 'rx', ms=9, mew=2, zorder=4)
        a1.annotate('%.1f %s' % (t, tag.replace('ROUTE_OBSTACLE_OR_GEOFENCE', 'ROUTE_BLOCKED')), sv['body_map'][:2], fontsize=7, color='darkred', xytext=(4, 6), textcoords='offset points')
    prev = tag
a1.plot(body[fl][0, 1], body[fl][0, 2], 'g^', ms=10, label='takeoff')
if goal: a1.plot(*goal, 'r*', ms=16, label='goal')
a1.set_aspect('equal'); a1.set_xlim(lo[0], hi[0]); a1.set_ylim(lo[1], hi[1]); a1.grid(alpha=.3); a1.legend(fontsize=8, loc='best')
a1.set_title('%s  top view (obstacles %.2f-%.2f m, map at %.1f s)' % (bag.rstrip('/').split('/')[-1], zmin_air, zmax_air, mt))
# side view along x
a2.scatter(vox[inb, 0], vox[inb, 2], s=3, c='lightgrey', marker='s')
if first is not None: a2.plot(first[:, 0], first[:, 2], c='tab:orange', lw=2, ls='--')
a2.plot(cmd[:, 1], cmd[:, 3], c='tab:red', lw=.8)
a2.scatter(body[fl, 1], body[fl, 3], c=body[fl, 0], s=8, cmap='viridis')
a2.set_xlim(lo[0], hi[0]); a2.set_ylim(-0.3, zmax_air + 0.3); a2.set_xlabel('map x (m)'); a2.set_ylabel('z (m)'); a2.grid(alpha=.3)
a2.set_title('side view (all voxels in the plotted xy box)')
plt.tight_layout(); plt.savefig(out, dpi=80)
print('wrote', out, 'voxels', len(vox))
