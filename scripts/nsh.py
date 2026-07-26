#!/usr/bin/env python3
"""Run PX4 NSH commands over MAVLink SERIAL_CONTROL and print the output.

Usage:
    nsh.py "param show EKF2_EV_CTRL"
    nsh.py "param set EKF2_EV_CTRL 11" "param save"
Each argument is one NSH command line, run in order.
"""
import copy
import sys
import time

from pymavlink import mavutil


def _patch_pymavlink_instances() -> None:
    """Fix the pymavlink 2.4.49 instanced-message crash at its root.

    `mavutil.add_message()` stores a message directly (leaving `_instances`
    at its class default of None) whenever that message's instance field is
    unset. If a later message of the *same* type does carry an instance value,
    it takes the "already seen this type" branch and does
    `messages[mtype]._instances[value] = msg` on None, raising
    `TypeError: 'NoneType' object does not support item assignment`.

    PX4 emits exactly this pattern, so any recv_match() could blow up. The
    upstream logic is fine apart from not re-initialising `_instances`, so
    patch that one condition rather than retrying around the symptom.
    """
    def add_message(messages, mtype, msg):
        if msg._instance_field is None or getattr(msg, msg._instance_field, None) is None:
            messages[mtype] = msg
            return
        instance_value = getattr(msg, msg._instance_field)
        if mtype not in messages or getattr(messages[mtype], "_instances", None) is None:
            messages[mtype] = copy.copy(msg)
            messages[mtype]._instances = {instance_value: msg}
            messages["%s[%s]" % (mtype, str(instance_value))] = copy.copy(msg)
            return
        messages[mtype]._instances[instance_value] = msg
        prev_instances = messages[mtype]._instances
        messages[mtype] = copy.copy(msg)
        messages[mtype]._instances = prev_instances
        messages["%s[%s]" % (mtype, str(instance_value))] = copy.copy(msg)

    mavutil.add_message = add_message


_patch_pymavlink_instances()

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
    # pymavlink 2.4.49 intermittently raises TypeError from add_message() when
    # an instanced message arrives before its sysid state exists; retry.
    deadline = time.time() + 30
    while True:
        try:
            if master.wait_heartbeat(timeout=10) is not None:
                break
        except TypeError:
            continue
        if time.time() > deadline:
            print("no heartbeat", file=sys.stderr)
            sys.exit(1)
    # nudge shell
    send_line(master, "")
    drain(master, 0.5)
    for line in cmds:
        send_line(master, line)
        sys.stdout.write(drain(master, 2.0))
    sys.stdout.flush()


if __name__ == "__main__":
    main()
