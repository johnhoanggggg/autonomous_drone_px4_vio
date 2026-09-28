#!/usr/bin/env python3
"""PX4 IMU thermal calibration from a cold start, run over the USB/NSH link.

Procedure (see THERMAL_CAL notes in memory 2026-09-27):
  1. Power everything off and let the drone cool to room temperature (30-45 min).
  2. Put it on a level surface, props off, where nothing will bump it.
  3. Plug in USB only (no battery needed), wait ~20 s for the FC to boot, then run:
        cd /home/john/autonomous_drone_px4_vio
        .venv-mavlink/bin/python3 scripts/thermal_cal.py --check   # preconditions only
        .venv-mavlink/bin/python3 scripts/thermal_cal.py           # run it (30-60 min)
  4. Do not touch the drone until the script says it has finished.

What it does:
  * refuses unless the FC is disarmed and both IMUs are cool (<= --tmax);
  * backs up all parameters to params_backup_<utc>_pre_thermal.params;
  * sets SYS_CAL_TMIN/TMAX/TDEL so PX4's own calibration can start at room
    temperature and finish within the ~20 C that USB self-heating gives
    (defaults: 5 / 32 / 16);
  * starts `temperature_compensation calibrate -a` (accelerometer only). PX4 records
    the warm-up, fits the correction curves, writes TC_A* params, sets
    TC_A_ENABLE=1 and saves. ACCEL ONLY ON PURPOSE: this firmware (d6f12ad) calls
    run_temperature_calibration(accel, baro, gyro, mag) against the declared
    (accel, gyro, mag, baro), so `-g` actually starts a MAGNETOMETER calibration,
    which never reaches the temperature rise and hangs the whole run (seen
    2026-09-27 15:13). Gyro thermal drift is left to the EKF's gyro-bias estimate;
  * logs both IMUs' temperature and mean accel to reports/thermal_cal_<utc>.csv;
  * when PX4 reports completion: turns off SENS_IMU_AUTOCAL (so in-flight offset
    learning does not fight the compensation; --keep-autocal to skip), saves, and
    reboots the FC so the compensation module starts from boot (rc.thermal_cal).

PX4 only finishes after BOTH IMUs have risen more than SYS_CAL_TDEL (this
firmware has no "stopped warming" exit). If the warm-up stalls short of it, the
script times out, nothing is saved, and it tells you to rerun with a lower --tdel.
"""
import argparse
import csv
import datetime as dt
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PY = sys.executable


def nsh(*commands, timeout=90):
    out = subprocess.run([PY, os.path.join(HERE, "nsh.py"), *commands],
                         capture_output=True, text=True, timeout=timeout, cwd=ROOT)
    return out.stdout + out.stderr


def param(name):
    m = re.search(rf"{name} \[\d+,\d+\] : (-?[\d.]+)", nsh(f"param show {name}"))
    return float(m.group(1)) if m else None


def imu_status():
    """[(temperature_accel, (ax, ay, az)) for instance 0, 1]."""
    text = nsh("listener vehicle_imu_status -n 1 -i 0", "listener vehicle_imu_status -n 1 -i 1")
    out = []
    for block in text.split("TOPIC: vehicle_imu_status")[1:]:
        t = re.search(r"temperature_accel: (-?[\d.]+)", block)
        a = re.search(r"mean_accel: \[(-?[\d.]+), (-?[\d.]+), (-?[\d.]+)\]", block)
        if t and a:
            out.append((float(t.group(1)), tuple(float(a.group(i)) for i in (1, 2, 3))))
    return out


def disarmed():
    return "INFO  [commander] Disarmed" in nsh("commander status")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--check", action="store_true", help="preconditions only; change nothing")
    p.add_argument("--tmin", type=int, default=5, help="SYS_CAL_TMIN (C)")
    p.add_argument("--tmax", type=int, default=32, help="SYS_CAL_TMAX: IMUs must start at or below this (C)")
    p.add_argument("--tdel", type=int, default=16, help="SYS_CAL_TDEL: required rise on BOTH IMUs (C)")
    p.add_argument("--timeout-min", type=float, default=75.0)
    p.add_argument("--poll-s", type=float, default=20.0)
    p.add_argument("--keep-autocal", action="store_true", help="leave SENS_IMU_AUTOCAL unchanged")
    p.add_argument("--no-reboot", action="store_true")
    a = p.parse_args()

    if not os.path.exists("/dev/ttyACM0"):
        sys.exit("FC not on USB (/dev/ttyACM0 missing)")
    if not disarmed():
        sys.exit("FC is not reporting Disarmed; refusing")
    imus = imu_status()
    if len(imus) < 2:
        sys.exit("could not read both IMUs' vehicle_imu_status")
    temps = [t for t, _ in imus]
    print("IMU temperatures now: " + ", ".join(f"IMU{i} {t:.1f} C" for i, t in enumerate(temps)))
    if max(temps) > a.tmax:
        sys.exit(f"too warm: an IMU is above {a.tmax} C. Power off and let it cool longer.")
    if min(temps) < a.tmin:
        sys.exit(f"too cold: an IMU is below {a.tmin} C (PX4 would wait for it)")
    if param("TC_A_ENABLE") == 1:
        print("note: thermal compensation is already enabled; this run will replace it")
    if a.check:
        print(f"preconditions OK. Would set SYS_CAL_TMIN {a.tmin}, TMAX {a.tmax}, TDEL {a.tdel} and start.")
        return 0

    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d_%H%M%S")
    backup = os.path.join(ROOT, f"params_backup_{stamp}_pre_thermal.params")
    print(f"backing up parameters -> {backup}")
    for attempt in range(3):
        r = subprocess.run([PY, os.path.join(HERE, "dump_params.py"), backup], capture_output=True, text=True, cwd=ROOT)
        if r.returncode == 0 and os.path.exists(backup):
            break
        time.sleep(2)
    else:
        sys.exit("parameter backup failed; not starting")

    nsh(f"param set SYS_CAL_TMIN {a.tmin}", f"param set SYS_CAL_TMAX {a.tmax}", f"param set SYS_CAL_TDEL {a.tdel}")
    got = (param("SYS_CAL_TMIN"), param("SYS_CAL_TMAX"), param("SYS_CAL_TDEL"))
    if got != (a.tmin, a.tmax, a.tdel):
        sys.exit(f"SYS_CAL_* readback mismatch: {got}")
    print(f"SYS_CAL_TMIN/TMAX/TDEL = {got}")

    print("starting PX4 thermal calibration (accelerometer). DO NOT MOVE THE DRONE.")
    print(nsh("temperature_compensation calibrate -a").strip().splitlines()[-3:])
    started = False
    start = time.time()
    t0 = temps[:]
    csv_path = os.path.join(ROOT, "reports", f"thermal_cal_{stamp}.csv")
    with open(csv_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["elapsed_s", "imu0_temp", "imu0_ax", "imu0_ay", "imu0_az", "imu1_temp", "imu1_ax", "imu1_ay", "imu1_az"])
        done = False
        while time.time() - start < a.timeout_min * 60:
            time.sleep(a.poll_s)
            imus = imu_status()
            if len(imus) >= 2:
                w.writerow([round(time.time() - start, 1)] + [v for t, acc in imus[:2] for v in (t, *acc)])
                fh.flush()
                rises = [imus[i][0] - t0[i] for i in range(2)]
                print(f"  {(time.time()-start)/60:5.1f} min  IMU0 {imus[0][0]:5.1f} C (+{rises[0]:4.1f})  "
                      f"IMU1 {imus[1][0]:5.1f} C (+{rises[1]:4.1f})  target +{a.tdel}", flush=True)
            log = nsh("dmesg")
            if not started:
                m = re.search(r"Starting temperature calibration task \(([^)]*)\)", log)
                if m:
                    started = True
                    print("  PX4: calibrating " + m.group(1))
                    if "mag=1" in m.group(1) or "accel=1" not in m.group(1):
                        print("  WARNING: PX4 is not calibrating accel only; this run may never finish")
            for line in log.splitlines()[-40:]:
                if "won't start" in line or "Failed" in line or "too high" in line:
                    print("  PX4: " + line.strip())
            if param("TC_A_ENABLE") == 1:
                done = True
                break
    if not done:
        print(f"TIMED OUT after {a.timeout_min} min without PX4 finishing. Nothing was saved.")
        print("If the rise stalled short of the target, cool down again and rerun with a lower --tdel.")
        return 1

    print(f"PX4 calibration COMPLETE after {(time.time()-start)/60:.1f} min. Data: {csv_path}")
    print(nsh("param show TC_A0_*", "param show TC_A1_*"))
    if not a.keep_autocal:
        nsh("param set SENS_IMU_AUTOCAL 0")
        print(f"SENS_IMU_AUTOCAL -> {param('SENS_IMU_AUTOCAL')}")
    nsh("param save")
    if not a.no_reboot and disarmed():
        nsh("reboot")
        print("FC rebooting so temperature_compensation starts from boot (rc.thermal_cal).")
        time.sleep(25)
        print(nsh("temperature_compensation status").strip().splitlines()[-3:])
    print(f"rollback if needed: load {backup}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
