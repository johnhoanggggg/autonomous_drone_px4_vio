#!/usr/bin/env python3
"""Pre-flight level check over USB/NSH (the FC sits on soft mounts and shifts when handled).

Put the drone on a KNOWN-LEVEL surface, on its legs, USB routed as flown, hands off.
    .venv-mavlink/bin/python3 scripts/level_check.py          # report only
    .venv-mavlink/bin/python3 scripts/level_check.py --fix    # re-level if outside tolerance

Reports resting roll/pitch (the mean of a few samples) and the SENS_BOARD offsets.
With --fix, if |roll| or |pitch| exceeds --tol it runs `commander calibrate level`
(disarmed only), then verifies the new resting attitude. Previous offsets are
printed so they can be restored with `param set SENS_BOARD_X_OFF/Y_OFF`.
"""
import argparse
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def nsh(*cmds, timeout=90):
    r = subprocess.run([sys.executable, os.path.join(HERE, "nsh.py"), *cmds],
                       capture_output=True, text=True, timeout=timeout, cwd=ROOT)
    return r.stdout + r.stderr


def attitude(samples=4):
    rolls, pitches = [], []
    for _ in range(samples):
        m = re.search(r"Roll: (-?[\d.]+) deg, Pitch: (-?[\d.]+) deg", nsh("listener vehicle_attitude -n 1"))
        if m:
            rolls.append(float(m.group(1)))
            pitches.append(float(m.group(2)))
    if not rolls:
        sys.exit("could not read vehicle_attitude")
    return sum(rolls) / len(rolls), sum(pitches) / len(pitches), max(rolls) - min(rolls)


def offsets():
    text = nsh("param show SENS_BOARD_X_OFF", "param show SENS_BOARD_Y_OFF")
    get = lambda n: float(re.search(rf"{n} \[\d+,\d+\] : (-?[\d.]+)", text).group(1))
    return get("SENS_BOARD_X_OFF"), get("SENS_BOARD_Y_OFF")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--fix", action="store_true", help="run level calibration if outside tolerance")
    p.add_argument("--tol", type=float, default=0.5, help="allowed |roll|/|pitch| at rest (deg)")
    a = p.parse_args()

    if not os.path.exists("/dev/ttyACM0"):
        sys.exit("FC not on USB")
    if "INFO  [commander] Disarmed" not in nsh("commander status"):
        sys.exit("FC not Disarmed; refusing")
    x, y = offsets()
    roll, pitch, spread = attitude()
    print(f"offsets: SENS_BOARD_X_OFF {x:.2f}  SENS_BOARD_Y_OFF {y:.2f}")
    print(f"at rest: roll {roll:+.2f} deg  pitch {pitch:+.2f} deg  (sample spread {spread:.2f})")
    if spread > 0.3:
        print("attitude still changing; is the drone being touched or still settling?")
    ok = abs(roll) <= a.tol and abs(pitch) <= a.tol
    print("LEVEL OK" if ok else f"OUT OF LEVEL (tolerance ±{a.tol} deg)")
    if ok or not a.fix:
        return 0 if ok else 1

    print("running `commander calibrate level`; hands off...")
    nsh("commander calibrate level")
    time.sleep(12)
    log = nsh("dmesg")
    for line in log.splitlines():
        if "SENS_BOARD" in line or "calibration done: level" in line or "calibration failed" in line:
            print("  PX4: " + line.strip())
    x2, y2 = offsets()
    roll, pitch, _ = attitude()
    print(f"new offsets: X {x:.2f} -> {x2:.2f}   Y {y:.2f} -> {y2:.2f}")
    print(f"at rest now: roll {roll:+.2f}  pitch {pitch:+.2f}")
    print(f"restore previous: scripts/nsh.py 'param set SENS_BOARD_X_OFF {x:.4f}' "
          f"'param set SENS_BOARD_Y_OFF {y:.4f}' 'param save'")
    return 0


if __name__ == "__main__":
    sys.exit(main())
