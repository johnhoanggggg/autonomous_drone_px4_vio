"""Pair the comparison bags to ULogs and measure demand/response while braking."""

from bisect import bisect_right
import json
import math
from pathlib import Path
import sys

import numpy as np
from pyulog import ULog


HERE = Path(__file__).parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from analyze_attitude_response import analyse as analyse_attitude  # noqa: E402


CASES = {
    "072654 baseline OFF": (
        ROOT / "reports/migration_tunnel_20260921_0726/072654.json",
        ROOT / "reports/migration_tunnel_20260921_0726/onboard/px4_22_07_06.ulg",
    ),
    "111754 damping ON": (HERE / "111754.json", HERE / "onboard/px4_01_53_15.ulg"),
    "112101 damping ON": (HERE / "112101.json", HERE / "onboard/px4_01_56_23.ulg"),
}


def event_time(logs, text):
    return next((time for time, row in logs
                 if row.get("name") == "cpp_flight_adapter" and row.get("msg") == text), None)


def select_debug(debug_times, debug_rows, times):
    indices = np.searchsorted(debug_times, times, side="right") - 1
    return [debug_rows[index] if index >= 0 and times[pos] - debug_times[index] <= .12 else None
            for pos, index in enumerate(indices)]


result = {}
for label, (bag_path, ulog_path) in CASES.items():
    bag = json.loads(bag_path.read_text(encoding="utf-8"))
    ulog = ULog(str(ulog_path))
    topics = {item.name: item.data for item in ulog.data_list if item.multi_id == 0}
    local = topics["vehicle_local_position"]
    bag_local = {int(row[1]["timestamp"]): row for row in bag["/fmu/out/vehicle_local_position_v1"]}
    matches = [(index, bag_local[int(stamp)]) for index, stamp in enumerate(local["timestamp"])
               if int(stamp) in bag_local]
    assert len(matches) > 500, (label, len(matches))
    position_difference = max(
        math.hypot(float(local["x"][index]) - row[1]["x"],
                   float(local["y"][index]) - row[1]["y"])
        for index, row in matches
    )
    offset = float(np.median([
        row[0] - local["timestamp"][index] / 1e6 for index, row in matches
    ]))
    logs = bag.get("/rosout", [])
    start = event_time(logs, "-> ROUTE")
    land = event_time(logs, "-> LAND")
    goal = next((time for time, row in bag.get("/planner/flight/status", [])
                 if row["data"].startswith("GOAL_REACHED")), None)
    end = goal or land or bag["/fmu/out/vehicle_local_position_v1"][-1][0]

    battery = topics["battery_status"]
    battery_time = battery["timestamp"] / 1e6 + offset
    battery_window = (battery_time >= start) & (battery_time <= end)
    route_window = [("route_to_goal_or_land", start, end)]
    _, attitude = analyse_attitude(ulog_path, offset, route_window)

    row = {
        "ulog": str(ulog_path.relative_to(ROOT)),
        "bytes": ulog_path.stat().st_size,
        "start_timestamp": ulog.start_timestamp / 1e6,
        "end_timestamp": ulog.last_timestamp / 1e6,
        "bag_time_offset": offset,
        "exact_local_position_matches": len(matches),
        "maximum_xy_position_difference": position_difference,
        "dropouts": [{"time": dropout.timestamp / 1e6 + offset,
                       "duration_ms": dropout.duration} for dropout in ulog.dropouts],
        "parameter_changes": ulog.changed_parameters,
        "parameters": {key: value for key, value in ulog.initial_parameters.items()
                       if key.startswith(("MPC_XY", "MC_ROLL", "MC_PITCH", "MC_YAW", "EKF2_EV_DELAY"))},
        "route_battery_voltage_median": float(np.median(battery["voltage_v"][battery_window])),
        "route_battery_current_median": float(np.median(battery["current_a"][battery_window])),
        "attitude": attitude["windows"]["route_to_goal_or_land"],
    }

    debug = [(time, json.loads(message["data"]))
             for time, message in bag.get("/planner/flight/damping_debug", [])]
    if debug:
        debug_times = np.array([item[0] for item in debug])
        debug_rows = [item[1] for item in debug]
        setpoint = topics["vehicle_local_position_setpoint"]
        setpoint_time = setpoint["timestamp"] / 1e6 + offset
        keep = (setpoint_time >= start) & (setpoint_time <= (land or end))
        selected_times = setpoint_time[keep]
        selected_debug = select_debug(debug_times, debug_rows, selected_times)
        braking = np.array([item is not None and item["reason"] == "braking_or_hold"
                            for item in selected_debug])
        vx = np.interp(selected_times, local["timestamp"] / 1e6 + offset, local["vx"])
        vy = np.interp(selected_times, local["timestamp"] / 1e6 + offset, local["vy"])
        speed = np.hypot(vx, vy)
        direction_x = np.divide(vx, speed, out=np.zeros_like(vx), where=speed > .02)
        direction_y = np.divide(vy, speed, out=np.zeros_like(vy), where=speed > .02)
        ax_demand = setpoint["acceleration[0]"][keep]
        ay_demand = setpoint["acceleration[1]"][keep]
        demand_along_motion = ax_demand * direction_x + ay_demand * direction_y
        local_time = local["timestamp"] / 1e6 + offset
        local_speed = np.hypot(local["vx"], local["vy"])
        measured_acceleration = np.gradient(local_speed, local_time)
        braking_on_local = np.array([
            item is not None and item["reason"] == "braking_or_hold"
            for item in select_debug(debug_times, debug_rows, local_time)
        ]) & (local_time >= start) & (local_time <= (land or end))
        reliable_direction = braking & (speed > .02)
        row["braking_response"] = {
            "internal_setpoint_samples": int(braking.sum()),
            "demand_opposed_motion_fraction": float(np.mean(demand_along_motion[reliable_direction] < 0)),
            "demand_along_motion_median": float(np.median(demand_along_motion[reliable_direction])),
            "demand_along_motion_p05": float(np.percentile(demand_along_motion[reliable_direction], 5)),
            "measured_speed_derivative_median": float(np.median(measured_acceleration[braking_on_local])),
            "measured_decelerating_fraction": float(np.mean(measured_acceleration[braking_on_local] < 0)),
        }
    result[label] = row

(HERE / "onboard_analysis.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
print(json.dumps(result, indent=2))
