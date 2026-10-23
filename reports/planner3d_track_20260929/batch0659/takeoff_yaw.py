"""Takeoff heading check: VIO yaw change vs gyro-integrated yaw (ULog) around arming.

Usage (ROS sourced): python3 takeoff_yaw.py   (flight list inside)
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.append('/home/john/autonomous_drone_px4_vio/.venv-mavlink/lib/python3.12/site-packages')
from pyulog import ULog  # noqa: E402

FL = '/home/john/autonomous_drone_px4_vio/ros2_ws/flight_logs/'
FLIGHTS = [('065455Z', 'on', '06_30_19'), ('065606Z', 'on', '06_31_30'), ('065755Z', 'on', '06_33_18'),
           ('065954Z', 'on', '06_35_19'), ('070328Z', 'off', '06_38_53'), ('070423Z', 'off', '06_39_46'),
           ('070833Z', 'on', '06_43_55')]


def yaw_q(w, x, y, z):
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


for stamp, imu, ulg in FLIGHTS:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=FL + 'offboard_planner_3d_unknown_20260929T' + stamp, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/fmu/in/vehicle_visual_odometry', '/fmu/out/vehicle_status_v1']))
    t0 = r.get_metadata().starting_time.nanoseconds
    ev, arm_bag = [], None
    while r.has_next():
        tp, d, ns = r.read_next()
        m = deserialize_message(d, get_message(types[tp]))
        t = (ns - t0) / 1e9
        if 'visual' in tp:
            ev.append((t, yaw_q(*[float(x) for x in m.q])))
        elif arm_bag is None and m.arming_state == 2:
            arm_bag = t
    ev = np.array(ev)
    U = ULog(FL + 'px4_20260929_%s.ulg' % ulg)
    T = {d.name: d.data for d in U.data_list if d.multi_id == 0}
    s = lambda a: (np.asarray(a, dtype=np.float64) - float(U.start_timestamp)) / 1e6
    st = T['vehicle_status']
    arm_ulog = s(st['timestamp'])[st['arming_state'] == 2][0]
    g = T['vehicle_angular_velocity']
    tg = s(g['timestamp']) - arm_ulog + arm_bag  # ULog time -> bag time (aligned at arming)
    gz = np.degrees(g['xyz[2]'])
    for a, b in [(arm_bag - 1.0, arm_bag + 5.0)]:
        m = (tg >= a) & (tg <= b)
        gyro = np.trapezoid(gz[m], tg[m]) if m.sum() > 1 else float('nan')
        ya = np.interp(a, ev[:, 0], np.unwrap(np.radians(ev[:, 1])))
        yb = np.interp(b, ev[:, 0], np.unwrap(np.radians(ev[:, 1])))
        vio = math.degrees(yb - ya)
        print('%s imu=%-3s  arm->+5 s: VIO yaw change %+6.1f deg, gyro says %+6.1f deg  -> VIO-only jump %+6.1f deg' % (
            stamp, imu, vio, gyro, vio - gyro))
