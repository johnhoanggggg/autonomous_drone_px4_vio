#!/usr/bin/env python3
"""Download a PX4 ULog off the SD card over MAVLink FTP on USB.

Usage:
    fetch_ulog.py --list
    fetch_ulog.py --latest [<dest_dir>]
    fetch_ulog.py /fs/microsd/log/2026-07-26/03_30_57.ulg ros2_ws/flight_logs/px4_03_30_57.ulg

`--latest` picks the newest date directory and the newest log inside it, and
writes it to <dest_dir> (default `ros2_ws/flight_logs`) as `px4_<name>.ulg`,
matching the naming used by the existing logs.

Remember the PX4 clock runs tens of seconds behind the Pi (78 s on 2026-07-26,
84 s on 2026-07-25), so a ULog named HH_MM_SS pairs with the rosbag stamped
slightly *later* than it. Match on duration/content, not on the name alone.
"""
import os
import sys
import time

from pymavlink import mavutil, mavftp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nsh import send_line, drain  # noqa: E402

LOG_DIR = "/fs/microsd/log"
DEFAULT_DEST = "ros2_ws/flight_logs"


def connect():
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    # Same pymavlink 2.4.49 wait_heartbeat TypeError race that nsh.py works around.
    deadline = time.time() + 30
    while True:
        try:
            if master.wait_heartbeat(timeout=10) is not None:
                return master
        except TypeError:
            pass
        if time.time() > deadline:
            print("no heartbeat on /dev/ttyACM0", file=sys.stderr)
            sys.exit(1)


def nsh_ls(master, path):
    """List `path` via NSH. MAVFTP's own list is flakier on this build."""
    send_line(master, "")
    drain(master, 0.5)
    send_line(master, f"ls -l {path}")
    return drain(master, 3.0)


def parse_entries(text):
    """Pull (name, size) out of NSH `ls -l` output. Directories have no size."""
    entries = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0].startswith(("-", "d")):
            name = parts[-1]
            size = None
            for p in parts[1:-1]:
                if p.isdigit():
                    size = int(p)
            entries.append((name, size))
    return entries


def find_latest(master):
    dirs = sorted(n for n, _ in parse_entries(nsh_ls(master, LOG_DIR)) if n.endswith("/"))
    if not dirs:
        sys.exit(f"no log directories under {LOG_DIR}")
    newest_dir = dirs[-1].rstrip("/")
    logs = sorted(
        (n, s) for n, s in parse_entries(nsh_ls(master, f"{LOG_DIR}/{newest_dir}"))
        if n.endswith(".ulg")
    )
    if not logs:
        sys.exit(f"no .ulg files under {LOG_DIR}/{newest_dir}")
    name, size = logs[-1]
    return f"{LOG_DIR}/{newest_dir}/{name}", name, size


def download(master, remote, local, expect_size=None):
    os.makedirs(os.path.dirname(os.path.abspath(local)), exist_ok=True)
    ftp = mavftp.MAVFTP(master, master.target_system, master.target_component)
    ftp.ftp_settings.debug = 0
    ftp.ftp_settings.retry_time = 0.5

    last = [time.time()]

    def progress(pct):
        if time.time() - last[0] > 3:
            print(f"  {pct:.0f}%", flush=True)
            last[0] = time.time()

    print(f"fetching {remote} -> {local}")
    ftp.cmd_get([remote, local], progress_callback=progress)
    # process_ftp_reply() runs the whole transfer itself; it takes a timeout,
    # NOT a message (passing a message raises a confusing TypeError).
    ftp.process_ftp_reply("OpenFileRO", timeout=900)

    if not os.path.exists(local):
        sys.exit("transfer produced no file")
    got = os.path.getsize(local)
    if expect_size is not None and got != expect_size:
        sys.exit(f"size mismatch: got {got} bytes, SD card reports {expect_size}")
    print(f"wrote {local} ({got} bytes){' — matches SD card' if expect_size else ''}")


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__.strip(), file=sys.stderr)
        sys.exit(2)

    master = connect()

    if args[0] == "--list":
        text = nsh_ls(master, LOG_DIR)
        for name, _ in parse_entries(text):
            if name.endswith("/"):
                print(f"{LOG_DIR}/{name}")
                for n, s in parse_entries(nsh_ls(master, f"{LOG_DIR}/{name.rstrip('/')}")):
                    print(f"    {n:20s} {s if s is not None else '':>10}")
        return

    if args[0] == "--latest":
        dest_dir = args[1] if len(args) > 1 else DEFAULT_DEST
        remote, name, size = find_latest(master)
        local = os.path.join(dest_dir, f"px4_{name}")
        download(master, remote, local, size)
        return

    if len(args) != 2:
        print(__doc__.strip(), file=sys.stderr)
        sys.exit(2)
    download(master, args[0], args[1])


if __name__ == "__main__":
    main()
