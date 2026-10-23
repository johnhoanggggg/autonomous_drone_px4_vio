"""Dump a 3D flight bag for hold_sim: paths, OctoMaps and the recorded disturbance.

Usage (ROS sourced): python3 dump_replay.py <bag_dir> <out_dir> <ulog>

Everything is expressed in the map frame frozen at route start (PX4 NED motion
re-expressed through the map correction at that moment), so the simulator can
work in one frame. The disturbance is the part of the recorded acceleration a
simple PX4 cascade does not explain from the recorded setpoints:
  v_sp = ff + Kp (s - p);  a = lag(Kv (v_sp - v), tau) + d
"""
import json
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.append('/home/john/autonomous_drone_px4_vio/.venv-mavlink/lib/python3.12/site-packages')
from pyulog import ULog  # noqa: E402

TAU_XY, TAU_Z = 0.15, 0.05


def rot(q, v):
    x, y, z, w = q
    t = 2 * np.cross([x, y, z], v)
    return v + w * t + np.cross([x, y, z], t)


bag, out, ulg = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(out, exist_ok=True)
P = ULog(ulg).initial_parameters
gains = {'kp_xy': P['MPC_XY_P'], 'kv_xy': P['MPC_XY_VEL_P_ACC'],
         'kp_z': P['MPC_Z_P'], 'kv_z': P['MPC_Z_VEL_P_ACC']}
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ['/rtabmap/octomap', '/planner3d/reference_path', '/planner3d/flight/status',
        '/planner3d/flight/config', '/fmu/out/vehicle_local_position_v1', '/fmu/in/trajectory_setpoint',
        '/planner3d/status']
r.set_filter(rosbag2_py.StorageFilter(topics=[w for w in want if w in types]))
t0 = r.get_metadata().starting_time.nanoseconds
maps, paths, st, lp, sp, cfg, planner = [], [], [], [], [], None, []
while r.has_next():
    tp, d, ns = r.read_next()
    t = (ns - t0) / 1e9
    m = deserialize_message(d, get_message(types[tp]))
    if tp == '/rtabmap/octomap':
        name = 'map_%03d.bin' % len(maps)
        open(os.path.join(out, name), 'wb').write(bytes(m.data))
        maps.append({'t': t, 'file': name, 'resolution': m.resolution, 'id': m.id, 'binary': m.binary})
    elif tp == '/planner3d/reference_path':
        paths.append({'t': t, 'points': [[q.pose.position.x, q.pose.position.y, q.pose.position.z] for q in m.poses]})
    elif tp == '/planner3d/flight/status':
        st.append((t, json.loads(m.data)))
    elif tp == '/planner3d/flight/config':
        cfg = json.loads(m.data)
    elif tp == '/planner3d/status':
        try:
            planner.append((t, json.loads(m.data)))
        except ValueError:
            pass
    elif tp == '/fmu/out/vehicle_local_position_v1':
        lp.append((t, m.x, m.y, m.z, m.vx, m.vy, m.vz))
    elif tp == '/fmu/in/trajectory_setpoint':
        sp.append((t, *m.position, *m.velocity))
route_t = next(t for t, v in st if v.get('state') == 'ROUTE_START')
end_t = max(t for t, v in st if v.get('state') not in ('LAND', 'DONE'))
t_fix, v0 = next((t, v) for t, v in st if t >= route_t and v.get('command_origin_ned') and v.get('map_from_odom'))
body0, act0, qc = np.array(v0['body_map']), np.array(v0['command_origin_ned']), v0['map_from_odom'][3:]
vec = lambda n: rot(qc, np.array([n[1], n[0], -n[2]]))  # NED vector -> frozen map
pos = lambda n: body0 + vec(np.array(n) - act0)
lp = np.array(lp)
sp = np.array([s for s in sp if all(math.isfinite(x) for x in s[1:4])])
g = np.arange(route_t, end_t, 0.01)
P_ = np.array([pos(np.interp(g, lp[:, 0], lp[:, k]) for k in (1, 2, 3)) for g in [None]] if False else
              [pos([np.interp(t, lp[:, 0], lp[:, 1]), np.interp(t, lp[:, 0], lp[:, 2]), np.interp(t, lp[:, 0], lp[:, 3])]) for t in g])
V_ = np.array([vec([np.interp(t, lp[:, 0], lp[:, 4]), np.interp(t, lp[:, 0], lp[:, 5]), np.interp(t, lp[:, 0], lp[:, 6])]) for t in g])
idx = np.searchsorted(sp[:, 0], g, side='right') - 1  # zero-order hold of the 10 Hz setpoint
idx = np.clip(idx, 0, len(sp) - 1)
S_ = np.array([pos(sp[i, 1:4]) for i in idx])
F_ = np.array([vec(np.nan_to_num(sp[i, 4:7])) for i in idx])
# Measured acceleration from smoothed velocity.
k = 5
Vs = np.column_stack([np.convolve(V_[:, a], np.ones(2 * k + 1) / (2 * k + 1), 'same') for a in range(3)])
A_meas = np.gradient(Vs, 0.01, axis=0)
kp = np.array([gains['kp_xy'], gains['kp_xy'], gains['kp_z']])
kv = np.array([gains['kv_xy'], gains['kv_xy'], gains['kv_z']])
tau = np.array([TAU_XY, TAU_XY, TAU_Z])
a_cmd = kv * (F_ + kp * (S_ - P_) - V_)
a_lag = np.zeros_like(a_cmd)
for i in range(1, len(g)):
    a_lag[i] = a_lag[i - 1] + (a_cmd[i - 1] - a_lag[i - 1]) * 0.01 / tau
D_ = A_meas - a_lag
goal = next(v['requested_goal'] for t, v in []) if False else None
# What the planner needs to replan from a simulated drone (planner-in-the-loop replay).
ps = [v for t, v in planner if isinstance(v, dict) and t <= end_t]
goal = next(v['requested_goal'] for v in reversed(ps) if v.get('requested_goal'))
geofence = next(v['geofence'] for v in reversed(ps) if v.get('geofence'))
floor = next((v.get('min_route_height_above_start') for v in reversed(ps)
              if v.get('min_route_height_above_start') is not None), None)
floor_z = next((v.get('route_floor_z') for v in reversed(ps) if v.get('route_floor_z') is not None), None)
ground_map_z = next((v['ground_map_z'] for t, v in st if v.get('ground_map_z') is not None), None)
takeoff_xy = next(v['body_map'][:2] for t, v in st if v.get('body_map'))
meta_planner = {'goal': goal, 'geofence': geofence, 'min_route_height_above_start': floor,
                'route_floor_z': floor_z, 'ground_map_z': ground_map_z, 'takeoff_xy': takeoff_xy,
                'lethal_radius': next(v['lethal_radius'] for v in ps if 'lethal_radius' in v),
                'recovery_connector_radius': next((v['recovery_connector_radius'] for v in ps
                                                   if 'recovery_connector_radius' in v), 0.30),
                'route_margin': next((v['route_margin'] for v in ps if 'route_margin' in v), 0.10)}
meta = {'planner': meta_planner,
        'route_t': route_t, 'end_t': end_t, 'dt': 0.01, 'gains': gains, 'tau': tau.tolist(),
        'config': cfg, 'maps': maps, 'paths': [p for p in paths if p['t'] <= end_t],
        'p0': P_[0].tolist(), 'v0': V_[0].tolist(), 'command0': S_[0].tolist(),
        'map_correction_note': 'frozen at %.2f s' % t_fix}
np.save(os.path.join(out, 'disturbance.npy'), np.column_stack([g, D_]))
np.savetxt(os.path.join(out, 'disturbance.csv'), np.column_stack([g, D_]), delimiter=',', fmt='%.6f')
np.savetxt(os.path.join(out, 'recorded.csv'), np.column_stack([g, P_, S_, F_]), delimiter=',', fmt='%.6f')
np.save(os.path.join(out, 'recorded.npy'), np.column_stack([g, P_, S_, F_]))
json.dump(meta, open(os.path.join(out, 'meta.json'), 'w'))
print('%s: %d maps, %d paths, route %.1f-%.1f s, disturbance rms xy %.3f z %.3f m/s2, gains %s' % (
    os.path.basename(bag.rstrip('/')), len(maps), len(meta['paths']), route_t, end_t,
    np.sqrt(np.mean(D_[:, 0] ** 2 + D_[:, 1] ** 2)), np.sqrt(np.mean(D_[:, 2] ** 2)), gains))
