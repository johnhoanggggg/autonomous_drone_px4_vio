#!/usr/bin/env python3
"""Run PX4 NSH commands over MAVLink SERIAL_CONTROL and print the output.

Usage:
    nsh.py "param show EKF2_EV_CTRL"
    nsh.py "param set EKF2_EV_CTRL 11" "param save"
Each argument is one NSH command line, run in order.
"""
import sys
import time

from pymavlink import mavutil

SERIAL_CONTROL_DEV_SHELL = 10  # SERIAL_CONTROL_DEV_SHELL
FLAG_REPLY = 1
FLAG_RESPOND = 2
FLAG_EXCLUSIVE = 4
FLAG_BLOCKING = 8
FLAG_MULTI = 16


def send_line(master, line: str) -> None:
    data = (line + "\n").encode("ascii")
    while data:
        chunk = data[:70]
        data = data[70:]
        buf = list(chunk) + [0] * (70 - len(chunk))
        master.mav.serial_control_send(
            SERIAL_CONTROL_DEV_SHELL,
            FLAG_RESPOND | FLAG_EXCLUSIVE | FLAG_MULTI,
            0, 0, len(chunk), bytes(buf),
        )


def drain(master, seconds: float) -> str:
    out = []
    end = time.time() + seconds
    while time.time() < end:
        msg = master.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.count > 0:
            out.append(bytes(msg.data[: msg.count]).decode("ascii", "replace"))
    return "".join(out)


def main() -> None:
    cmds = sys.argv[1:]
    if not cmds:
        print("usage: nsh.py <cmd> [<cmd> ...]", file=sys.stderr)
        sys.exit(2)

    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    master.wait_heartbeat(timeout=10)
    # nudge shell
    send_line(master, "")
    drain(master, 0.5)
    for line in cmds:
        send_line(master, line)
        sys.stdout.write(drain(master, 2.0))
    sys.stdout.flush()


if __name__ == "__main__":
    main()
