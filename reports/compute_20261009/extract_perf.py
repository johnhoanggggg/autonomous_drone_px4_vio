#!/usr/bin/env python3
"""Compute/memory stats from the latest planner3d bag (process_monitor /perf topics).

source /opt/ros/jazzy/setup.bash && source ../../ros2_ws/install/setup.bash && python3 extract_perf.py
"""
import glob, json, pickle, sys
from collections import defaultdict
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/john/autonomous_drone_px4_vio/ros2_ws/flight_logs/offboard_planner_3d_unknown_20261009T050943Z"
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=glob.glob(BAG + "/*.mcap")[0], storage_id="mcap"),
       rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
scal = defaultdict(list); procs = []; status = []; stamps = defaultdict(list)
while r.has_next():
    tp, data, ts = r.read_next()
    t = ts * 1e-9
    stamps[tp].append(t)
    if tp in ("/perf/cpu_percent", "/perf/mem_percent", "/perf/cpu_temp_c", "/perf/throttled", "/perf/load1"):
        scal[tp].append((t, deserialize_message(data, get_message(types[tp])).data))
    elif tp == "/perf/processes":
        procs.append((t, json.loads(deserialize_message(data, get_message(types[tp])).data)))
    elif tp == "/fmu/out/vehicle_status_v1":
        m = deserialize_message(data, get_message(types[tp]))
        status.append((t, m.arming_state, m.nav_state))
out = dict(bag=BAG, scal={k: np.array(v) for k, v in scal.items()}, procs=procs, status=np.array(status),
           stamps={k: np.array(v) for k, v in stamps.items()}, types=types)
pickle.dump(out, open("perf.pkl", "wb"))
print("perf samples", {k: len(v) for k, v in scal.items()}, "process tables", len(procs))
print("example table keys:", list(procs[len(procs) // 2][1].keys())[:30])
