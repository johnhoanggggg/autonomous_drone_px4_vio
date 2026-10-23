#!/usr/bin/env python3
"""Offline, bag-aligned control-chain audit for the 2026-09-13 lateral stop."""
from pathlib import Path
import json
import sys

import numpy as np
from pyulog import ULog

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
FLIGHT = ROOT / "ros2_ws/flight_logs/offboard_lateral_stop_20260913T121110Z"
ULOG = FLIGHT / "px4_03_37_03.ulg"
BAG = FLIGHT / "render/lateral_stop.data.json"


def latest(source_t, target_t, max_age=.05):
    idx = np.searchsorted(source_t, target_t, side="right") - 1
    idx = np.maximum(idx, 0)
    return idx, (target_t - source_t[idx]) <= max_age


def moving_average(t, x, seconds=.08):
    dt = np.median(np.diff(t))
    n = max(1, int(round(seconds / dt)))
    kernel = np.ones(n) / n
    return np.convolve(x, kernel, mode="same")


def euler(q):
    q = q / np.linalg.norm(q, axis=0)
    w, x, y, z = q
    return np.array([
        np.arctan2(2*(w*x + y*z), 1-2*(x*x+y*y)),
        np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
        np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)),
    ])


def tilt_accel(q, lateral):
    q = q / np.linalg.norm(q, axis=0)
    w, x, y, z = q
    b3 = np.array([2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)])
    return -9.81 * (lateral @ b3[:2]) / b3[2]


def rotate_body_to_ned(q, vector):
    """Rotate FRD vector columns into NED using PX4's body-to-earth quaternion."""
    q = q / np.linalg.norm(q, axis=0)
    w, x, y, z = q
    r = np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])
    return np.einsum("ijn,jn->in", r, vector)


def crossing_after(t, x, start, direction, threshold=0.0, end=None):
    keep = np.flatnonzero((t >= start) & (t <= (t[-1] if end is None else end)))
    if direction > 0:
        hits = keep[x[keep] >= threshold]
    else:
        hits = keep[x[keep] <= threshold]
    return None if not len(hits) else float(t[hits[0]])


def extrema_from_velocity(t, v, lo, hi, deadband=.008):
    """Hysteretic zero crossings of smoothed velocity, i.e. position extrema."""
    mask = (t >= lo) & (t <= hi)
    ii = np.flatnonzero(mask)
    state = 0
    out = []
    for i in ii:
        new = 1 if v[i] > deadband else (-1 if v[i] < -deadband else state)
        if state and new != state:
            out.append((float(t[i]), "minimum" if new > 0 else "maximum"))
        state = new
    return out


bag = json.loads(BAG.read_text())
offset = bag["boot_to_bag_offset"]
yaw0 = bag["debug"][0]["yaw0"]
lateral = np.array([-np.sin(yaw0), np.cos(yaw0)])
origin = np.array(bag["debug"][0]["origin"])
u = ULog(str(ULOG))
d = {x.name: x.data for x in u.data_list if x.multi_id == 0}

# Use local position's 100 Hz estimator clock for outer-loop quantities.
lp = d["vehicle_local_position"]
t = lp["timestamp"] / 1e6 + offset
pos = lateral @ (np.array([lp["x"], lp["y"]]).T - origin).T
vel = lateral @ np.array([lp["vx"], lp["vy"]])
est_accel = lateral @ np.array([lp["ax"], lp["ay"]])
sp = d["vehicle_local_position_setpoint"]
st = sp["timestamp"] / 1e6 + offset
si, sv = latest(st, t, .2)
accel_d = lateral @ np.array([sp["acceleration[0]"][si], sp["acceleration[1]"][si]])
vel_d = lateral @ np.array([sp["vx"][si], sp["vy"][si]])

att = d["vehicle_attitude"]
at = att["timestamp"] / 1e6 + offset
q = np.array([att[f"q[{i}]"] for i in range(4)])
actual_tilt_accel = tilt_accel(q, lateral)
asp = d["vehicle_attitude_setpoint"]
ast = asp["timestamp"] / 1e6 + offset
qi, qv = latest(ast, at, .05)
qd = np.array([asp[f"q_d[{i}]"][qi] for i in range(4)])
desired_tilt_accel = tilt_accel(qd, lateral)

# Body roll is the relevant lateral-tilt rate at yaw ~= 90 degrees.
gyro = d["vehicle_angular_velocity"]
gt = gyro["timestamp"] / 1e6 + offset
roll_rate = np.degrees(gyro["xyz[0]"])
rsp = d["vehicle_rates_setpoint"]
rst = rsp["timestamp"] / 1e6 + offset
ri, rv = latest(rst, gt, .05)
roll_rate_d = np.degrees(rsp["roll"][ri])
torque = d["vehicle_torque_setpoint"]
tt = torque["timestamp"] / 1e6 + offset
ti, tv = latest(tt, gt, .02)
roll_torque = torque["xyz[0]"][ti]
status = d["rate_ctrl_status"]
sst = status["timestamp"] / 1e6 + offset
ssi, ssv = latest(sst, gt, .1)
roll_i = status["rollspeed_integ"][ssi]
roll_error_rate = rsp["roll"][ri] - gyro["xyz[0]"]
roll_p = u.initial_parameters["MC_ROLLRATE_P"] * roll_error_rate
roll_d = -u.initial_parameters["MC_ROLLRATE_D"] * gyro["xyz_derivative[0]"]

# vehicle_acceleration contains bias-corrected specific force in body FRD. Rotate
# it to NED, add gravity, then project horizontally. This remains IMU-derived.
va = d["vehicle_acceleration"]
vat = va["timestamp"] / 1e6 + offset
vai, vav = latest(at, vat, .05)
va_body = np.array([va[f"xyz[{i}]"] for i in range(3)])
va_ned = rotate_body_to_ned(q[:, vai], va_body)
va_ned[2] += 9.81
imu_accel = lateral @ va_ned[:2]

lo, hi = 25.88, 38.73
hold = 28.678268611
brake = next(row["time"] for row in bag["debug"]
             if row["leg"] == 2 and row["phase"] == "BRAKE")
target = -.35

# Smooth only for event finding; all error/RMS metrics below use native samples.
vsm = moving_average(t, vel, .10)
asm = moving_average(t, accel_d, .10)
tilt_d_sm = moving_average(at, desired_tilt_accel, .06)
tilt_a_sm = moving_average(at, actual_tilt_accel, .06)
imu_sm = moving_average(vat, imu_accel, .10)
rate_d_sm = moving_average(gt, roll_rate_d, .04)
rate_sm = moving_average(gt, roll_rate, .04)

# Identify sustained outer-loop command lobes by crossings of +/-0.08 m/s^2.
command_events = []
state = 0
for i in np.flatnonzero((t >= brake) & (t <= hi)):
    new = 1 if asm[i] > .08 else (-1 if asm[i] < -.08 else state)
    if new and new != state:
        tc = float(t[i])
        # Desired and achieved tilt must reach the same modest threshold.
        td = crossing_after(at, tilt_d_sm, tc, new, .05*new, min(tc+1, hi))
        ta = crossing_after(at, tilt_a_sm, tc, new, .05*new, min(tc+1, hi))
        ia = crossing_after(vat, imu_sm, tc, new, .05*new, min(tc+1, hi))
        command_events.append({
            "accel_command_threshold_s": tc,
            "sign": "positive/right" if new > 0 else "negative/left",
            "desired_tilt_threshold_s": td,
            "actual_tilt_threshold_s": ta,
            "imu_accel_threshold_s": ia,
            "command_to_actual_tilt_ms": None if ta is None else round(1000*(ta-tc), 1),
            "desired_to_actual_tilt_ms": None if td is None or ta is None else round(1000*(ta-td), 1),
            "command_to_imu_accel_ms": None if ia is None else round(1000*(ia-tc), 1),
        })
        state = new

# Per-lobe agreement, robust to small timing mismatch and high-frequency vibration.
mask_att = (at >= lo) & (at <= hi) & qv
angle_actual = euler(q)[0]
angle_target = euler(qd)[0]
roll_error = np.degrees((angle_actual-angle_target+np.pi)%(2*np.pi)-np.pi)
mask_g = (gt >= lo) & (gt <= hi) & rv & tv
same_torque_rate_error = np.mean(
    np.sign(roll_torque[mask_g]) == np.sign((roll_rate_d-roll_rate)[mask_g]))


def episode(a, b):
    ma = (at >= a) & (at < b)
    mg = (gt >= a) & (gt < b) & rv & tv & ssv
    ml = (t >= a) & (t < b)
    mv = (vat >= a) & (vat < b) & vav
    target_minus_actual = -roll_error[ma]
    return {
        "window_s": [a, b],
        "accel_d_mean_m_s2": float(np.mean(accel_d[ml])),
        "desired_tilt_equiv_mean_m_s2": float(np.mean(desired_tilt_accel[ma])),
        "actual_tilt_equiv_mean_m_s2": float(np.mean(actual_tilt_accel[ma])),
        "imu_accel_mean_m_s2": float(np.mean(imu_accel[mv])),
        "velocity_mean_m_s": float(np.mean(vel[ml])),
        "target_minus_actual_roll_mean_deg": float(np.mean(target_minus_actual)),
        "roll_rate_target_mean_deg_s": float(np.mean(roll_rate_d[mg])),
        "roll_rate_actual_mean_deg_s": float(np.mean(roll_rate[mg])),
        "attitude_p_observed_ratio_per_s": float(np.mean(roll_rate_d[mg]) / np.mean(target_minus_actual)),
        "rate_torque_terms_mean": {
            "P": float(np.mean(roll_p[mg])),
            "I": float(np.mean(roll_i[mg])),
            "D": float(np.mean(roll_d[mg])),
            "sum": float(np.mean((roll_p+roll_i+roll_d)[mg])),
            "logged": float(np.mean(roll_torque[mg])),
        },
        "integral_opposes_p_fraction": float(np.mean(np.sign(roll_i[mg]) == -np.sign(roll_p[mg]))),
    }


def integral_update(a, b):
    # The status field is the accumulated I torque contribution. Compare its
    # measured change with integral(gain * rate_error * dt), excluding dropouts.
    dt = np.diff(sst)
    di = np.diff(status["rollspeed_integ"])
    ge, gev = latest(gt, sst, .05)
    re, rev = latest(rst, sst, .05)
    error = rsp["roll"][re] - gyro["xyz[0]"][ge]
    m = ((sst[:-1] >= a) & (sst[:-1] < b) & (dt < .03)
         & gev[:-1] & rev[:-1])
    predicted = u.initial_parameters["MC_ROLLRATE_I"] * error[:-1] * dt
    return {
        "window_s": [a, b],
        "measured_delta_I": float(np.interp(b, sst, status["rollspeed_integ"])
                                  - np.interp(a, sst, status["rollspeed_integ"])),
        "predicted_delta_I": float(np.sum(predicted[m])),
        "derivative_error_correlation": float(np.corrcoef(
            di[m] / dt[m], u.initial_parameters["MC_ROLLRATE_I"] * error[:-1][m])[0, 1]),
        "update_sign_matches_rate_error_fraction": float(np.mean(
            np.sign(di[m]) == np.sign(error[:-1][m]))),
    }


# A coarse physical-polarity check. Binning suppresses prop-frequency vibration;
# it is deliberately reported as weak evidence rather than a plant fit.
physical_sign = []
for a in np.arange(lo, hi-.1, .1):
    m = (gt >= a) & (gt < a+.1)
    if np.count_nonzero(m) > 2:
        tq_mean = float(np.mean(roll_torque[m]))
        angular_accel = float((roll_rate[m][-1]-roll_rate[m][0]) / (gt[m][-1]-gt[m][0]))
        if abs(tq_mean) > .002 and abs(angular_accel) > 2:
            physical_sign.append(np.sign(tq_mean) == np.sign(angular_accel))

# Evaluate actual acceleration response without treating estimator cross-correlation as causal.
# The median is robust to prop vibration; compare it with tilt direction in 100 ms bins.
bins = np.arange(lo, hi, .1)
bin_rows = []
for a, b in zip(bins[:-1], bins[1:]):
    def med(tx, x):
        m = (tx >= a) & (tx < b)
        return float(np.median(x[m])) if np.any(m) else None
    bin_rows.append({"time_s": round((a+b)/2, 3), "accel_d": med(t, accel_d),
                     "tilt_d": med(at, desired_tilt_accel),
                     "tilt_actual": med(at, actual_tilt_accel),
                     "imu_accel": med(vat, imu_accel), "vio_est_accel": med(t, est_accel),
                     "velocity": med(t, vel), "position_error": None})
    bin_rows[-1]["position_error"] = med(t, pos-target)

hm = (t >= hold) & (t <= hi)
summary = {
    "flight": str(ULOG.relative_to(ROOT)),
    "bag_offset_s": offset,
    "window_s": [lo, hi],
    "planned_brake_start_s": brake,
    "fixed_hold_start_s": hold,
    "parameters": {k: u.initial_parameters[k] for k in [
        "EKF2_EV_DELAY", "MC_ROLLRATE_P", "MPC_XY_P", "MPC_XY_VEL_P_ACC",
        "MPC_XY_VEL_I_ACC", "MPC_XY_VEL_D_ACC"]},
    "hold": {
        "position_error_range_m": [float(np.min(pos[hm]-target)), float(np.max(pos[hm]-target))],
        "position_span_m": float(np.ptp(pos[hm])),
        "velocity_range_m_s": [float(np.min(vel[hm])), float(np.max(vel[hm]))],
        "accel_d_range_m_s2": [float(np.min(accel_d[hm])), float(np.max(accel_d[hm]))],
        "tilt_equiv_actual_range_m_s2": [float(np.min(actual_tilt_accel[mask_att & (at>=hold)])),
                                                   float(np.max(actual_tilt_accel[mask_att & (at>=hold)]))],
        "position_extrema": extrema_from_velocity(t, vsm, hold, hi),
    },
    "tracking": {
        "roll_angle_error_rms_deg": float(np.sqrt(np.mean(roll_error[mask_att]**2))),
        "roll_angle_error_p95_abs_deg": float(np.percentile(np.abs(roll_error[mask_att]), 95)),
        "roll_rate_error_rms_deg_s": float(np.sqrt(np.mean((roll_rate_d[mask_g]-roll_rate[mask_g])**2))),
        "torque_same_sign_as_rate_error_fraction": float(same_torque_rate_error),
        "torque_same_sign_as_roll_accel_100ms_active_bins": float(np.mean(physical_sign)),
        "torque_roll_accel_bins": len(physical_sign),
        "note": "PX4 torque should share the sign of (rate_target-rate_actual); this directly checks feedback polarity rather than inferring it from translational cross-correlation.",
    },
    "episodes": {
        "braking_build": episode(27.975, 28.678),
        "first_hold_braking": episode(28.678, 29.733),
        "large_negative_correction_onset": episode(32.769, 32.953),
        "large_negative_correction_early": episode(32.953, 33.453),
        "large_negative_correction_continues": episode(33.453, 35.706),
        "last_positive_correction": episode(37.783, 38.73),
    },
    "integrator_update_checks": {
        "braking_build": integral_update(27.975, 28.678),
        "negative_correction_early": integral_update(32.953, 33.453),
        "full_window": integral_update(25.88, 38.73),
    },
    "command_reversals": command_events,
    "limits": [
        "Local position/velocity/acceleration are EKF estimates influenced by delayed external vision.",
        "vehicle_acceleration is IMU-derived body specific force rotated here into NED, but vibration, filtering, attitude error, drag, and accelerometer bias limit its use as ground-truth displacement.",
        "Threshold delays describe these recorded reversals; they are not a fitted plant delay or a closed-loop cross-correlation estimate.",
    ],
}

OUT.mkdir(parents=True, exist_ok=True)
(OUT / "chain_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
(OUT / "chain_100ms.json").write_text(json.dumps(bin_rows, indent=2) + "\n")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
fig, ax = plt.subplots(4, 1, figsize=(13, 11), sharex=True, layout="constrained")
ax[0].plot(t, (pos-target)*100, label="EKF position error")
ax[0].plot(t, vel*100, label="EKF lateral velocity (cm/s)")
ax[0].set_ylabel("cm, cm/s"); ax[0].legend()
ax[1].plot(t, accel_d, label="PX4 acceleration demand")
ax[1].plot(at, desired_tilt_accel, label="desired tilt equivalent", alpha=.8)
ax[1].plot(at, actual_tilt_accel, label="actual tilt equivalent", alpha=.8)
ax[1].set_ylabel("m/s²"); ax[1].legend()
ax[2].plot(vat, moving_average(vat, imu_accel, .10), label="IMU-derived acceleration, 100 ms mean")
ax[2].plot(t, est_accel, label="EKF reported acceleration", alpha=.7)
ax[2].set_ylabel("m/s²"); ax[2].legend()
ax[3].plot(gt, rate_d_sm, label="roll-rate target")
ax[3].plot(gt, rate_sm, label="roll rate")
ax[3].plot(gt, roll_torque*100, label="roll torque ×100", alpha=.6)
ax[3].set_ylabel("deg/s, scaled torque"); ax[3].set_xlabel("bag-aligned seconds"); ax[3].legend()
for a in ax:
    a.set_xlim(lo, hi); a.grid(alpha=.25); a.axvline(brake, color="orange", ls="--")
    a.axvline(hold, color="black", ls=":")
fig.suptitle("Third lateral leg/hold: onboard demand → attitude/rates → translation")
fig.savefig(OUT / "chain.png", dpi=150)
print(json.dumps(summary, indent=2))
