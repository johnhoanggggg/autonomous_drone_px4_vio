#!/usr/bin/env python3
"""Start PX4 multicopter autotune over USB MAVLink while the offboard hover is flying.

PX4 handles MAV_CMD_DO_AUTOTUNE_ENABLE only in the MAVLink receiver (it sets
MC_AT_START); the ROS/uXRCE-DDS vehicle_command path cannot start it. So this
talks to the FC over the USB cable, which must stay connected in flight
(CBRK_USB_CHK=197848 allows arming with USB).

Usage (run on the Pi once the hover is established):
    .venv-mavlink/bin/python3 scripts/start_autotune.py --check   # preconditions only, sends nothing
    .venv-mavlink/bin/python3 scripts/start_autotune.py           # start + monitor

Safety rules:
  * sends the start only when ARMED, in OFFBOARD/POSCTL/ALTCTL, and at least
    --min-alt metres up;
  * re-sending the command is how progress is polled, but PX4 treats a re-send
    while autotune is IDLE as a fresh start. So polling stops for good once
    autotune reports FAILED, COMPLETE or waiting-for-disarm (95%), or the vehicle
    disarms or leaves the allowed modes;
  * abort in flight by moving the RC roll or pitch stick more than 5%
    (autotune fails immediately), or land/kill as usual. Nothing is saved on
    failure. With MC_AT_APPLY=1, new gains are written only after disarm.
"""
import argparse
import os
import sys
import time

from pymavlink import mavutil

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fetch_ulog import connect  # noqa: E402

GAINS = ["MC_ROLLRATE_K", "MC_ROLLRATE_P", "MC_ROLLRATE_I", "MC_ROLLRATE_D", "MC_ROLL_P",
         "MC_PITCHRATE_K", "MC_PITCHRATE_P", "MC_PITCHRATE_I", "MC_PITCHRATE_D", "MC_PITCH_P",
         "MC_YAWRATE_K", "MC_YAWRATE_P", "MC_YAWRATE_I", "MC_YAWRATE_D", "MC_YAW_P"]
ALLOWED_MAIN_MODES = {2: "ALTCTL", 3: "POSCTL", 6: "OFFBOARD"}
STAGES = {0: "init", 20: "roll", 40: "pitch", 60: "yaw", 80: "verification",
          85: "apply", 90: "test", 95: "waiting for disarm", 100: "complete"}
CMD = mavutil.mavlink.MAV_CMD_DO_AUTOTUNE_ENABLE
DEFAULT_BACKUP = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                              "params_backup_20260927_pre_autotune.params")


class Vehicle:
    def __init__(self, master):
        self.m = master
        self.armed = None
        self.main_mode = None
        self.alt = None
        self.acks = []

    def pump(self, duration):
        end = time.time() + duration
        while time.time() < end:
            msg = self.m.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                continue
            kind = msg.get_type()
            if kind == "HEARTBEAT" and msg.get_srcComponent() == 1 and msg.autopilot != 8:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.main_mode = (msg.custom_mode >> 16) & 0xFF
            elif kind == "LOCAL_POSITION_NED":
                self.alt = -msg.z
            elif kind == "COMMAND_ACK" and msg.command == CMD:
                self.acks.append((msg.result, getattr(msg, "progress", 0)))
            elif kind == "STATUSTEXT":
                print(f"  [FC] {msg.text}")

    def ok_to_command(self, min_alt):
        reasons = []
        if not self.armed:
            reasons.append("not armed")
        if self.main_mode not in ALLOWED_MAIN_MODES:
            reasons.append(f"mode {self.main_mode} not OFFBOARD/POSCTL/ALTCTL")
        if self.alt is None:
            reasons.append("no altitude")
        elif self.alt < min_alt:
            reasons.append(f"altitude {self.alt:.2f} m < {min_alt:.2f} m")
        return reasons

    def send_autotune(self):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     CMD, 0, 1, 0, 0, 0, 0, 0, 0)

    def read_param(self, name, tries=5):
        for _ in range(tries):
            self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component,
                                               name.encode(), -1)
            end = time.time() + 1.0
            while time.time() < end:
                msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
                if msg is not None and msg.param_id == name:
                    return float(msg.param_value)
        return None


def backup_values(path):
    values = {}
    if not os.path.exists(path):
        return values
    for line in open(path):
        parts = line.split()
        if len(parts) >= 4 and not line.startswith("#"):
            try:
                values[parts[2]] = float(parts[3])
            except ValueError:
                pass
    return values


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--check", action="store_true", help="report preconditions only; send nothing")
    p.add_argument("--min-alt", type=float, default=0.5, help="minimum height above EKF origin (m)")
    p.add_argument("--timeout", type=float, default=180.0, help="give up monitoring after this many seconds")
    p.add_argument("--backup", default=DEFAULT_BACKUP, help="params backup to diff against after disarm")
    args = p.parse_args()

    v = Vehicle(connect())
    # Ask for local position at 10 Hz on this link so the altitude check is fresh.
    v.m.mav.command_long_send(v.m.target_system, v.m.target_component,
                              mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                              32, 100000, 0, 0, 0, 0, 0)
    v.pump(1.5)
    mode = ALLOWED_MAIN_MODES.get(v.main_mode, v.main_mode)
    alt = "?" if v.alt is None else f"{v.alt:.2f} m"
    print(f"armed={v.armed} mode={mode} altitude={alt}")
    reasons = v.ok_to_command(args.min_alt)
    if reasons:
        print("NOT starting autotune: " + "; ".join(reasons))
        return 1
    if args.check:
        print("preconditions OK (--check: nothing sent)")
        return 0

    print("starting autotune (abort: move RC roll/pitch stick >5%, or land/kill)")
    v.send_autotune()
    start = time.time()
    last_stage = None
    polling = True
    while time.time() - start < args.timeout:
        v.pump(1.0)
        if v.acks:
            result, progress = v.acks[-1]
            v.acks.clear()
            stage = STAGES.get(progress, f"{progress}%")
            if result == mavutil.mavlink.MAV_RESULT_FAILED:
                print("autotune FAILED/aborted: nothing will be saved")
                polling = False
            elif result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED:
                print("autotune UNSUPPORTED on this vehicle")
                return 1
            elif stage != last_stage:
                print(f"  {time.time() - start:5.1f} s  {stage} ({progress}%)")
                last_stage = stage
            if progress >= 95 or result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                polling = False
        if polling and v.ok_to_command(0.0):
            print("vehicle disarmed or left the allowed modes: polling stopped")
            polling = False
        if polling:
            v.send_autotune()  # progress poll (safe: autotune is not idle here)
        if v.armed is False:
            break
    if v.armed:
        print("monitor timeout while still armed; autotune status unknown. Land and disarm normally.")
        return 1

    print("disarmed. Reading gains (written on disarm only if autotune passed verification):")
    v.pump(3.0)
    before = backup_values(args.backup)
    changed = False
    for name in GAINS:
        now = v.read_param(name)
        old = before.get(name)
        mark = ""
        if now is not None and old is not None and abs(now - old) > 1e-6:
            mark = "  <- changed"
            changed = True
        old_s = "?" if old is None else f"{old:.4f}"
        now_s = "?" if now is None else f"{now:.4f}"
        print(f"  {name:16s} {old_s:>9s} -> {now_s:>9s}{mark}")
    print("gains CHANGED by autotune" if changed else "no gain changes (autotune did not complete, or found the same gains)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
