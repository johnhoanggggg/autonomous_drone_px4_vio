"""Dump a 3D-planner flight bag to a pickle: statuses, paths, PX4 state, VIO.

Usage (ROS sourced): python3 dump3d.py <bag_dir> <out.pkl>
Times are seconds from the bag's starting_time.
"""
import json
import pickle
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, out = sys.argv[1], sys.argv[2]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = {'/planner3d/flight/status', '/planner3d/flight/config', '/planner3d/status', '/rosout',
        '/fmu/out/vehicle_status_v1', '/fmu/out/vehicle_local_position_v1', '/fmu/in/trajectory_setpoint',
        '/rtabmap/vio_pose', '/rtabmap/vio_odom_info_lite', '/waypoint/clicked', '/fmu/out/battery_status_v1',
        '/planner3d/reference_path'}
r.set_filter(rosbag2_py.StorageFilter(topics=list(want & set(types))))
t0 = r.get_metadata().starting_time.nanoseconds
D = {k: [] for k in want}
while r.has_next():
    tp, d, ns = r.read_next()
    t = (ns - t0) / 1e9
    m = deserialize_message(d, get_message(types[tp]))
    if tp == '/rtabmap/vio_odom_info_lite':
        D[tp].append((t, {'inliers': m.inliers, 'lost': m.lost, 'features': m.features}))
    elif tp in ('/planner3d/flight/status', '/planner3d/flight/config', '/planner3d/status'):
        try:
            D[tp].append((t, json.loads(m.data)))
        except ValueError:
            pass
    elif tp == '/rosout':
        D[tp].append((t, m.name, m.level, m.msg))
    elif tp == '/fmu/out/vehicle_status_v1':
        D[tp].append((t, m.arming_state, m.nav_state, m.failsafe))
    elif tp == '/fmu/out/vehicle_local_position_v1':
        D[tp].append((t, m.timestamp, m.x, m.y, m.z, m.vx, m.vy, m.vz, m.heading))
    elif tp == '/fmu/in/trajectory_setpoint':
        D[tp].append((t, m.timestamp, *m.position, *m.velocity, *m.acceleration, m.yaw))
    elif tp == '/rtabmap/vio_pose':
        p = m.pose.pose.position if hasattr(m.pose, 'pose') else m.pose.position
        D[tp].append((t, p.x, p.y, p.z))
    elif tp == '/waypoint/clicked':
        D[tp].append((t, m.point.x, m.point.y, m.point.z))
    elif tp == '/fmu/out/battery_status_v1':
        D[tp].append((t, m.voltage_v, m.current_a, m.remaining, m.cell_count))
    elif tp == '/planner3d/reference_path':
        D[tp].append((t, [(q.pose.position.x, q.pose.position.y, q.pose.position.z) for q in m.poses]))
pickle.dump(D, open(out, 'wb'))
print(bag.rstrip('/').split('/')[-1], 'end %.1f' % t, {k.split('/')[-1]: len(v) for k, v in D.items()})
