#!/usr/bin/env python3
"""Dump all PX4 parameters over MAVLink to a QGC-compatible .params file for backup."""
import sys
import time
from datetime import datetime, timezone

from pymavlink import mavutil


def main() -> None:
    out_path = sys.argv[1] if len(sys.argv) > 1 else None
    if out_path is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        out_path = f"/home/john/autonomous_drone_px4_vio/params_backup_{stamp}.params"

    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    print("Waiting for heartbeat...")
    master.wait_heartbeat(timeout=10)
    print("Connected, sys", master.target_system, "comp", master.target_component)

    master.mav.param_request_list_send(master.target_system, master.target_component)

    params = {}
    expected_count = None
    last_progress = time.time()
    timeout_s = 60
    start = time.time()

    while time.time() - start < timeout_s:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if msg is None:
            if expected_count is not None and len(params) >= expected_count:
                break
            if time.time() - last_progress > 8:
                break
            continue
        last_progress = time.time()
        expected_count = msg.param_count
        params[msg.param_id] = (msg.param_value, msg.param_type, msg.param_index)
        if len(params) % 200 == 0:
            print(f"  received {len(params)}/{expected_count}")
        if expected_count is not None and len(params) >= expected_count:
            break

    print(f"Received {len(params)} params (expected {expected_count})")

    with open(out_path, "w") as f:
        f.write("# PX4 parameter backup\n")
        f.write(f"# Generated {datetime.now(timezone.utc).isoformat()}\n")
        f.write("# Vehicle Component Name Value Type\n")
        for name in sorted(params.keys()):
            value, ptype, _ = params[name]
            f.write(f"1\t1\t{name}\t{value}\t{ptype}\n")

    print(f"Wrote backup to {out_path}")


if __name__ == "__main__":
    main()
