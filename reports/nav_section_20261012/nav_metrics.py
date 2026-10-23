"""Navigation summary numbers for 2D global-planner and 3D planner bags (report section).

Per flight: armed time, route-phase time, flown path length vs first planned route length,
straight start->goal distance, end distance to goal, goal reached, hold time, cross-track
(2D: follower's own /planner/follower/cross_track; 3D: horizontal distance from the drone to the
route that was active 1 s earlier), height range, planner replans and planning time.
Usage (ROS sourced): python3 nav_metrics.py <bag_dir> [...]
"""
import json
import re
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
    t0 = r.get_metadata().starting_time.nanoseconds
    while r.has_next():
        tp, d, ns = r.read_next()
        yield tp, (ns - t0) / 1e9, deserialize_message(d, get_message(types[tp]))


def plen(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0


def seg_dist(p, path):
    """Distance from point p to polyline path (same dimension)."""
    if len(path) == 1:
        return float(np.linalg.norm(p - path[0]))
    a, b = path[:-1], path[1:]
    ab = b - a
    t = np.clip(((p - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-12), 0, 1)
    return float(np.linalg.norm(a + ab * t[:, None] - p, axis=1).min())


def latest_before(rows, t):
    best = None
    for row in rows:
        if row[0] > t:
            break
        best = row
    return best


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float('nan')


def states_time(st, t_a, t_b, names):
    tot = 0.0
    for (t, s), (tn, _) in zip(st, st[1:] + [(t_b, None)]):
        lo, hi = max(t, t_a), min(tn, t_b)
        if hi > lo and s in names:
            tot += hi - lo
    return tot


def flight_2d(bag):
    body, paths, st, xt, goal, armed, pms, reached = [], [], [], [], None, [], [], []
    for tp, t, m in read(bag, ['/rtabmap/body_pose', '/planner/path', '/planner/flight/status', '/planner/follower/cross_track',
                               '/waypoint/clicked', '/fmu/out/vehicle_status_v1', '/planner/planning_ms',
                               '/planner/follower/goal_reached']):
        if tp == '/rtabmap/body_pose':
            body.append([t, m.pose.position.x, m.pose.position.y, m.pose.position.z])
        elif tp == '/planner/path' and len(m.poses):
            paths.append((t, np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses])))
        elif tp == '/planner/flight/status':
            st.append((t, m.data.split()[0] if m.data else ''))
        elif tp == '/planner/follower/cross_track':
            xt.append((t, m.data))
        elif tp == '/waypoint/clicked':
            goal = np.array([m.point.x, m.point.y])
        elif tp == '/fmu/out/vehicle_status_v1':
            armed.append((t, m.arming_state == 2))
        elif tp == '/planner/planning_ms':
            pms.append(m.data)
        elif tp == '/planner/follower/goal_reached' and m.data:
            reached.append(t)
    body = np.array(body)
    on = [t for t, a in armed if a]
    t_a, t_b = (on[0], on[-1]) if on else (body[0, 0], body[-1, 0])
    b = body[(body[:, 0] >= t_a) & (body[:, 0] <= t_b)]
    fol = [t for t, s in st if s == 'FOLLOWING']
    r_a = fol[0] if fol else t_a
    r_b = reached[0] if reached else t_b
    br = b[(b[:, 0] >= r_a) & (b[:, 0] <= r_b)]
    x = np.abs([v for t, v in xt if r_a <= t <= r_b])
    first = paths[0][1] if paths else np.zeros((1, 2))
    return dict(kind='2D', armed=t_b - t_a, route_time=r_b - r_a, flown=plen(br[:, 1:3]), first_route=plen(first),
                straight=float(np.linalg.norm(goal - br[0, 1:3])) if goal is not None else float('nan'),
                end_to_goal=float(np.linalg.norm(goal - b[-1, 1:3])) if goal is not None else float('nan'),
                reached=bool(reached), hold=states_time(st, r_a, r_b, {'HOLD', 'COMMAND_HOLD'}),
                xt_p50=pct(x, 50), xt_p95=pct(x, 95), xt_max=float(x.max()) if len(x) else float('nan'),
                z_lo=float(b[:, 3].min()), z_hi=float(b[:, 3].max()), replans=len(paths),
                plan_ms_p50=pct(pms, 50), plan_ms_p95=pct(pms, 95),
                states=sorted({s for t, s in st if r_a <= t <= r_b}))


def flight_3d(bag):
    status, routes, goal, pms = [], [], None, []
    for tp, t, m in read(bag, ['/planner3d/flight/status', '/planner3d/reference_path', '/waypoint/clicked',
                               '/planner3d/status']):
        if tp == '/planner3d/flight/status':
            try:
                status.append((t, json.loads(m.data)))
            except ValueError:
                pass
        elif tp == '/planner3d/reference_path' and len(m.poses):
            routes.append((t, np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in m.poses])))
        elif tp == '/waypoint/clicked':
            goal = np.array([m.point.x, m.point.y, m.point.z])
        elif tp == '/planner3d/status':
            mm = re.search(r'plan(?:ning)?_ms["=: ]+([0-9.]+)', m.data)
            if mm:
                pms.append(float(mm.group(1)))
    st = [(t, v.get('state')) for t, v in status]
    body = np.array([[t, *v['body_map']] for t, v in status if v.get('body_map')])
    t_a = next(t for t, s in st if s == 'CLIMB')
    t_b = body[-1, 0]
    r_a = next((t for t, s in st if s in ('FOLLOWING', 'ROUTE_START', 'YAW_ALIGN') and t > t_a), t_a)
    r_b = next((t for t, s in st if s in ('GOAL_REACHED', 'SAFE_APPROACH_REACHED')), t_b)
    br = body[(body[:, 0] >= r_a) & (body[:, 0] <= r_b)]
    xt, ze = [], []
    for row in br:
        rt = latest_before(routes, row[0] - 1.0)
        if rt is None:
            continue
        xt.append(seg_dist(row[1:3], rt[1][:, :2]))
        ze.append(seg_dist(row[1:4], rt[1]))
    first = routes[0][1] if routes else np.zeros((1, 3))
    gz = goal if goal is not None else first[-1]
    end_goal = routes[-1][1][-1] if routes else gz
    ground = next((v['ground_map_z'] for t, v in status if v.get('ground_map_z') is not None), body[0, 3])
    return dict(kind='3D', armed=t_b - t_a, route_time=r_b - r_a, flown=plen(br[:, 1:4]), first_route=plen(first),
                straight=float(np.linalg.norm(end_goal - br[0, 1:4])),
                end_to_goal=float(np.linalg.norm(gz[:2] - body[-1, 1:3])),
                reached=any(s in ('GOAL_REACHED', 'SAFE_APPROACH_REACHED') for t, s in st),
                hold=states_time(st, r_a, r_b, {'HOLD'}),
                xt_p50=pct(xt, 50), xt_p95=pct(xt, 95), xt_max=max(xt) if xt else float('nan'),
                d3_p95=pct(ze, 95),
                z_lo=float(body[:, 3].min() - ground), z_hi=float(body[:, 3].max() - ground),
                climb=float(br[:, 3].max() - br[0, 3]) if len(br) else 0.0, replans=len(routes),
                plan_ms_p50=pct(pms, 50), plan_ms_p95=pct(pms, 95),
                states=sorted({s for t, s in st if r_a <= t <= r_b if s}))


for bag in sys.argv[1:]:
    name = bag.rstrip('/').split('/')[-1]
    try:
        res = flight_3d(bag) if 'planner_3d' in name else flight_2d(bag)
    except Exception as e:   # noqa: BLE001
        print(name, 'FAILED', type(e).__name__, e)
        continue
    print(name)
    print('  ' + ', '.join(f'{k}={v:.2f}' if isinstance(v, float) else f'{k}={v}' for k, v in res.items()))
