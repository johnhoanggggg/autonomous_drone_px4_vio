#!/usr/bin/env python3
"""Pull the control data for the report's controls section out of the 10-05 ULogs.

Run with the pyulog env:  ../../.venv-mavlink/bin/python extract_controls.py
Writes hover.npz (7-min hover hold) and bump.pkl (bump-test responses).
Square B inner-loop data is already in ../vicon_vs_px4_20261005/ulog_squareB.npz.
"""
import pickle
import sys
from pathlib import Path
import numpy as np
from pyulog import ULog

LOGS = Path("/home/john/autonomous_drone_px4_vio/ros2_ws/flight_logs")
HERE = Path(__file__).resolve().parent

# --- hover hold: position vs offboard setpoint while in OFFBOARD and armed
u = ULog(str(LOGS / "px4_17_42_03.ulg"), ["vehicle_local_position", "trajectory_setpoint", "vehicle_status"])
d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
lp, ts, vs = d["vehicle_local_position"], d["trajectory_setpoint"], d["vehicle_status"]
np.savez(HERE / "hover.npz",
         lp=np.c_[lp["timestamp"] / 1e6, lp["x"], lp["y"], lp["z"]],
         sp=np.c_[ts["timestamp"] / 1e6, ts["position[0]"], ts["position[1]"], ts["position[2]"]],
         st=np.c_[vs["timestamp"] / 1e6, vs["arming_state"], vs["nav_state"]])

# --- bump test: reuse load() from scripts/plot_bump_test.py (that script plots at import,
# so exec only the part before its plotting section)
src = (HERE / "../../scripts/plot_bump_test.py").read_text()
ns = {}
sys.argv = ["plot_bump_test.py", "unused"]
exec(src.split("\nruns = [")[0], ns)
runs = [ns["load"](str(LOGS / f)) for f in ("px4_bump_19_46_26.ulg", "px4_bump_19_47_20.ulg")]
keep = [{k: r[k] for k in ("path", "tl", "dh", "rows", "t_land")} for r in runs]
pickle.dump(keep, open(HERE / "bump.pkl", "wb"))
print("hover lp", lp["x"].shape, "bump events", [len(r["rows"]) for r in runs])
