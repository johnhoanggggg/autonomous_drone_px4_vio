"""VIO rate across a flight, per 1 s bin, with the flight phase and likely causes.

Usage (ROS sourced): python3 vio_rate.py <bag> [<bag> ...] [--table]
Per bin: processed frames (odometry header stamps; the camera runs at 10 Hz),
lost frames, inliers/features, estimation time, local-bundle time, latency
(bag receive - image stamp), body rotation rate, horizontal speed, height,
oak_container CPU, total CPU, and the adapter state.
"""
import json
import math
import sys
from collections import defaultdict

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ['/rtabmap/vio_odometry', '/rtabmap/vio_odom_info_lite', '/fmu/out/sensor_combined',
          '/fmu/out/vehicle_local_position_v1', '/fmu/out/vehicle_status_v1',
          '/fmu/out/vehicle_land_detected', '/perf/processes', '/planner3d/flight/status']


def load(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    out = defaultdict(list)
    while r.has_next():
        tp, d, t = r.read_next()
        m = deserialize_message(d, get_message(types[tp]))
        t *= 1e-9
        if tp == '/rtabmap/vio_odometry':
            s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            out['odom'].append((t, s))
        elif tp == '/rtabmap/vio_odom_info_lite':
            out['info'].append((t, m.lost, m.inliers, m.features, m.time_estimation, m.local_bundle_time, m.interval))
        elif tp == '/fmu/out/sensor_combined':
            out['gyro'].append((t, math.degrees(np.linalg.norm(m.gyro_rad))))
        elif tp == '/fmu/out/vehicle_local_position_v1':
            out['lpos'].append((t, math.hypot(m.vx, m.vy), -m.z))
        elif tp == '/fmu/out/vehicle_status_v1':
            out['armed'].append((t, m.arming_state == 2))
        elif tp == '/fmu/out/vehicle_land_detected':
            out['landed'].append((t, m.landed))
        elif tp == '/perf/processes':
            j = json.loads(m.data)
            out['cpu'].append((t, j['processes'].get('oak_container', {}).get('cpu_percent', np.nan), j['cpu_percent']))
        elif tp == '/planner3d/flight/status':
            out['state'].append((t, json.loads(m.data).get('state', '?')))
    return out


def last(series, t, default=None):
    v = default
    for row in series:
        if row[0] > t:
            break
        v = row[1:]
    return v


def bins(out):
    odom = np.array(out['odom'])
    t0 = odom[0, 0]
    t1 = odom[-1, 0]
    rows = []
    for b in np.arange(t0, t1 - 0.5, 1.0):
        sel = lambda key: [x for x in out[key] if b <= x[0] < b + 1]
        od = sel('odom')
        inf = sel('info')
        gy = [x[1] for x in sel('gyro')]
        lp = sel('lpos')
        cpu = last(out['cpu'], b + 1, (np.nan, np.nan))
        armed = last(out['armed'], b + 0.5, (False,))[0]
        landed = last(out['landed'], b + 0.5, (True,))[0]
        state = last(out['state'], b + 0.5, ('-',))[0]
        ok = [x for x in inf if not x[1]]
        rows.append(dict(
            t=b - t0, frames=len(od), lost=sum(1 for x in inf if x[1]),
            inliers=np.median([x[2] for x in ok]) if ok else 0,
            features=np.median([x[3] for x in inf]) if inf else 0,
            est=1000 * np.median([x[4] for x in inf]) if inf else np.nan,
            est_max=1000 * max([x[4] for x in inf]) if inf else np.nan,
            bundle=1000 * np.median([x[5] for x in inf]) if inf else np.nan,
            gap=max([x[6] for x in inf]) if inf else np.nan,
            lat=1000 * np.median([x[0] - x[1] for x in od]) if od else np.nan,
            rot=np.percentile(gy, 95) if gy else np.nan,
            speed=np.mean([x[1] for x in lp]) if lp else np.nan,
            height=np.mean([x[2] for x in lp]) if lp else np.nan,
            oak=cpu[0], cpu=cpu[1],
            phase='ground' if not armed else ('armed-ground' if landed else 'air'),
            state=state))
    return rows


if __name__ == '__main__':
    bags = [a for a in sys.argv[1:] if not a.startswith('--')]
    for bag in bags:
        rows = bins(load(bag))
        print('==', bag.rstrip('/').split('/')[-1])
        if '--table' in sys.argv:
            print('   t fr lost inl  feat  est/max ms bund gap   lat  rot95 spd  hgt  oak%  cpu% phase        state')
            for r in rows:
                print('%4.0f %2d %3d %4.0f %5.0f %4.0f/%4.0f %4.0f %4.2f %5.0f %5.0f %4.2f %4.2f %5.0f %5.1f %-12s %s' % (
                    r['t'], r['frames'], r['lost'], r['inliers'], r['features'], r['est'], r['est_max'], r['bundle'],
                    r['gap'], r['lat'], r['rot'], r['speed'], r['height'], r['oak'], r['cpu'], r['phase'], r['state']))
        np.save(bag.rstrip('/') + '.vio_rate.npy', rows, allow_pickle=True) if False else None
        json.dump(rows, open('/tmp/claude-1000/-home-john/6080f5c0-7dee-4871-bccc-d0b0da0dde85/scratchpad/' + bag.rstrip('/').split('/')[-1] + '.vio.json', 'w'), default=float)
