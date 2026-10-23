#!/usr/bin/env python3
"""Figures and numbers for the report's controls section (2026-10-05 flights).

Run with /home/john/sai_venv/bin/python3 after extract_controls.py.
  fig_c1_architecture.png   cascaded control loops
  fig_c2_hover.png          7-min hover hold (offboard_hover, ULog 17_42_03)
  fig_c3_disturbance.png    bump-test response (ULogs 19_46_26, 19_47_20)
  fig_c4_tracking.png       Square B tracking error relative to the nose
  fig_c5_inner_loop.png     Square B attitude tracking and motor commands
"""
import json
import pickle
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from scipy.spatial.transform import Rotation as R

HERE = Path(__file__).resolve().parent
VIC = HERE / "../vicon_vs_px4_20261005"
MASS = 1.8
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.3})
metrics = {}


def save(fig, name):
    fig.tight_layout()
    fig.savefig(HERE / name, dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------- C1 architecture
fig, ax = plt.subplots(figsize=(13, 4.2))
ax.set_xlim(0, 13); ax.set_ylim(0, 4.2); ax.axis("off"); ax.grid(False)


def box(x, y, w, h, text, fc):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.04", fc=fc, ec="0.3", lw=1.2))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=9)


def arrow(x0, y0, x1, y1, text=None, dy=0.13):
    ax.annotate("", (x1, y1), (x0, y0), arrowprops=dict(arrowstyle="-|>", color="0.25", lw=1.2))
    if text:
        ax.text((x0 + x1) / 2, (y0 + y1) / 2 + dy, text, ha="center", va="bottom", fontsize=7.5, color="0.2")


pi_c, fc_c, hw_c, est_c = "#dbe9f6", "#fde9d4", "#e4e4e4", "#e2f2e0"
y, h = 2.5, 0.9
box(0.1, y, 1.5, h, "Offboard node\n(Raspberry Pi 5)", pi_c)
box(2.45, y, 1.2, h, "Position\nP", fc_c)
box(4.5, y, 1.2, h, "Velocity\nPID", fc_c)
box(6.55, y, 1.2, h, "Attitude\nP", fc_c)
box(8.6, y, 1.2, h, "Body rate\nPID", fc_c)
box(10.65, y, 1.0, h, "Mixer", fc_c)
box(12.0, y, 0.95, h, "ESCs +\nmotors", hw_c)
arrow(1.6, y + h / 2, 2.45, y + h / 2, "position\nsetpoint\n(50 Hz)")
arrow(3.65, y + h / 2, 4.5, y + h / 2, "velocity\nsetpoint")
arrow(5.7, y + h / 2, 6.55, y + h / 2, "attitude +\nthrust")
arrow(7.75, y + h / 2, 8.6, y + h / 2, "rate\nsetpoint")
arrow(9.8, y + h / 2, 10.65, y + h / 2, "torque +\nthrust")
arrow(11.65, y + h / 2, 12.0, y + h / 2)
box(3.4, 0.35, 3.6, 0.9, "EKF2 estimator\n(VIO position, height, heading + IMU)", est_c)
box(8.6, 0.35, 1.2, 0.9, "Gyro\n(IMU)", est_c)
box(0.1, 0.35, 2.4, 0.9, "OAK-D Lite VIO\n(15 Hz, uXRCE-DDS)", hw_c)
arrow(2.5, 0.8, 3.4, 0.8)
for x_to, x_from in ((3.05, 4.0), (5.1, 5.2), (7.15, 6.4)):
    ax.annotate("", (x_to, y), (x_from, 1.25), arrowprops=dict(arrowstyle="-|>", color="#3a7d34", lw=1.0, ls="--"))
ax.annotate("", (9.2, y), (9.2, 1.25), arrowprops=dict(arrowstyle="-|>", color="#3a7d34", lw=1.0, ls="--"))
ax.text(5.2, 1.75, "position / velocity / attitude feedback", ha="center", fontsize=7.5, color="#3a7d34",
        bbox=dict(fc="white", ec="none", pad=1))
ax.text(9.3, 1.75, "rate feedback", ha="left", fontsize=7.5, color="#3a7d34")
ax.add_patch(FancyBboxPatch((2.3, 2.25), 9.5, 1.55, boxstyle="round,pad=0.02", fc="none", ec="#c77b2a",
                            lw=1.0, ls=":"))
ax.text(7.05, 3.88, "PX4 on Pixhawk 4", ha="center", fontsize=9, color="#c77b2a")
save(fig, "fig_c1_architecture.png")

# ---------------------------------------------------------------- C2 hover hold
H = np.load(HERE / "hover.npz")
lp, sp, st = H["lp"], H["sp"], H["st"]
t_arm = st[st[:, 1] == 2, 0][0]
t_off_end = st[(st[:, 1] == 2) & (st[:, 2] == 14), 0][-1]
idx = np.clip(np.searchsorted(sp[:, 0], lp[:, 0], side="right") - 1, 0, len(sp) - 1)
e = lp[:, 1:4] - sp[idx, 1:4]
t = lp[:, 0] - t_arm
spz = sp[idx, 3]
hold = (t > 20) & (lp[:, 0] < t_off_end) & (np.abs(spz - np.median(spz[(t > 30) & (t < 40)])) < 0.01)
eh = 100 * np.hypot(e[hold, 0], e[hold, 1]); ez = -100 * e[hold, 2]; th = t[hold]
metrics["hover"] = dict(duration_s=float(th[-1] - th[0]), height_m=float(-spz[hold].mean()),
                        horiz_rms_cm=float(np.sqrt(np.mean(eh ** 2))), horiz_p95_cm=float(np.percentile(eh, 95)),
                        horiz_max_cm=float(eh.max()), height_rms_cm=float(np.sqrt(np.mean(ez ** 2))),
                        height_p95_cm=float(np.percentile(np.abs(ez), 95)), height_max_cm=float(np.abs(ez).max()))
fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 5.2), sharex=True)
a1.plot(th, eh, "C0", lw=0.7)
a1.axhline(metrics["hover"]["horiz_p95_cm"], color="C3", ls="--", lw=1, label=f"p95 {metrics['hover']['horiz_p95_cm']:.1f} cm")
a1.set_ylabel("horizontal error [cm]"); a1.legend(loc="upper right", fontsize=9)
a1.set_title(f"(a) Horizontal distance from hold point")
a2.plot(th, ez, "C2", lw=0.7); a2.axhline(0, color="0.5", lw=0.6)
a2.set_ylabel("height error [cm]"); a2.set_xlabel("time since arming [s]")
a2.set_title(f"(b) Height error (setpoint {metrics['hover']['height_m']:.1f} m)")
fig.suptitle(f"Hover hold, {metrics['hover']['duration_s']:.0f} s at 1.0 m (PX4 estimate vs setpoint)", fontsize=12)
save(fig, "fig_c2_hover.png")

# ---------------------------------------------------------------- C3 disturbance rejection
runs = pickle.load(open(HERE / "bump.pkl", "rb"))
fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.6))
rows = []
k = 0
for r in runs:
    starts = [row["t"] for row in r["rows"]]
    for row in r["rows"]:
        s = row["t"]; dv = row["J"] / MASS
        nxt = [x for x in starts if x > s]
        t_end = min(s + 2.5, nxt[0] if nxt else np.inf)
        w = (r["tl"] >= s - 0.2) & (r["tl"] < t_end)
        if not row["cut"]:          # pushes cut short by AUTO.LAND are left out
            k += 1
            a1.plot(r["tl"][w] - s, r["dh"][w], lw=1.3, label=f"push {k}: Δv {dv:.2f} m/s")
        rows.append(dict(dv=dv, J=row["J"], peak_cm=row["dmax"], t_peak=row["tmax"],
                         recover_s=row["rec"], overshoot_cm=row["over"], cut=bool(row["cut"])))
a1.axhline(5, color="0.4", ls="--", lw=0.8)
a1.text(-0.18, 6.0, "5 cm", ha="left", fontsize=8, color="0.3")
a1.set_xlabel("time since push [s]"); a1.set_ylabel("horizontal displacement [cm]")
a1.set_title("(a) Response to each push"); a1.legend(fontsize=8)
full = [x for x in rows if not x["cut"]]
dvs = np.array([x["dv"] for x in full]); pk = np.array([x["peak_cm"] for x in full])
gain = float(np.sum(dvs * pk) / np.sum(dvs ** 2))       # cm per m/s, least squares through origin
a2.plot(dvs, pk, "o", color="C0", ms=7, label="pushes")
xx = np.linspace(0, dvs.max() * 1.08, 10)
a2.plot(xx, gain * xx, "--", color="0.4", lw=1, label=f"fit: {gain/100:.2f} s × Δv")
a2.set_xlabel("velocity change from push, Δv [m/s]"); a2.set_ylabel("peak displacement [cm]")
a2.set_title("(b) Peak displacement vs push size"); a2.legend(fontsize=9)
fig.suptitle("Disturbance rejection: hand pushes during a 0.3 m hover", fontsize=12)
save(fig, "fig_c3_disturbance.png")
rec = [x["recover_s"] for x in full if x["recover_s"] is not None]
metrics["bump"] = dict(n=len(full), dv_range=[float(dvs.min()), float(dvs.max())],
                       peak_range_cm=[float(pk.min()), float(pk.max())], s_per_dv=gain / 100,
                       t_peak_range=[float(min(x["t_peak"] for x in full)), float(max(x["t_peak"] for x in full))],
                       recover_range_s=[float(min(rec)), float(max(rec))],
                       overshoot_range_cm=[float(min(x["overshoot_cm"] for x in full)),
                                           float(max(x["overshoot_cm"] for x in full))])

# ---------------------------------------------------------------- Square B (bag + ULog)
B = np.load(VIC / "sq_20261005T034924Z.npz")
blp = B["fmu__out__vehicle_local_position_v1"]; tsp = B["fmu__in__trajectory_setpoint"]
tb0 = blp[0, 0]
T_TAKEOFF, W0, W1 = 8.04, 9.04, 55.35        # bag s: liftoff, window start, final turn starts
tl = blp[:, 0] - tb0; ts = tsp[:, 0] - tb0
i = np.clip(np.searchsorted(ts, tl, side="right") - 1, 0, len(ts) - 1)
err = blp[:, 2:4] - tsp[i, 2:4]
hdg = blp[:, 5]
fwd = np.c_[np.cos(hdg), np.sin(hdg)]; right = np.c_[-np.sin(hdg), np.cos(hdg)]
ef = 100 * np.sum(err * fwd, 1); er = 100 * np.sum(err * right, 1)
spd = np.r_[0, np.linalg.norm(np.diff(tsp[:, 2:4], axis=0), axis=1) / np.maximum(np.diff(ts), 1e-3)]
moving = spd[i] > 0.02
yaw_sp = np.unwrap(tsp[:, 5]); yawrate_sp = np.r_[0, np.abs(np.diff(yaw_sp)) / np.maximum(np.diff(ts), 1e-3)]
turning = yawrate_sp[i] > 0.05
win = (tl >= W0) & (tl <= W1)
tt = tl - T_TAKEOFF


def shade(ax):
    for mask, color, lab in ((moving & win, "#cfe3f7", "leg (setpoint moving)"), (turning & win & ~moving, "#fbe3c8", "turn")):
        edges = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
        for n, (a, b) in enumerate(zip(edges[::2], edges[1::2])):
            ax.axvspan(tt[a], tt[b - 1], color=color, lw=0, label=lab if n == 0 else None)


def phase_stats(mask):
    m = mask & win
    return dict(fwd_mean_cm=float(ef[m].mean()), right_mean_cm=float(er[m].mean()))


metrics["squareB_tracking"] = dict(legs=phase_stats(moving), holds=phase_stats(~moving & ~turning),
                                   turns=phase_stats(turning & ~moving))
fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 5.6), sharex=True)
shade(a1); a1.plot(tt[win], ef[win], "C0", lw=1.2)
a1.axhline(0, color="0.5", lw=0.6); a1.set_ylabel("forward error [cm]")
a1.set_title("(a) Along the nose (negative = behind the setpoint)"); a1.legend(loc="lower left", fontsize=8)
shade(a2); a2.plot(tt[win], er[win], "C3", lw=1.2)
a2.axhline(0, color="0.5", lw=0.6)
a2.axhline(metrics["squareB_tracking"]["holds"]["right_mean_cm"], color="C3", ls="--", lw=0.8)
a2.set_ylabel("sideways error [cm]"); a2.set_xlabel("time since takeoff [s]")
a2.set_title("(b) Sideways (negative = left of the setpoint)")
fig.suptitle("Square B: position tracking error relative to the vehicle's heading (PX4 estimate vs setpoint)", fontsize=12)
save(fig, "fig_c4_tracking.png")

U = np.load(VIC / "ulog_squareB.npz")
bag2px4 = lambda tb: np.interp(tb, tl, blp[:, 1])
p0, p1, pto = bag2px4(W0), bag2px4(W1), bag2px4(T_TAKEOFF)


def cut(a):
    return a[(a[:, 0] >= p0) & (a[:, 0] <= p1)]


def eul(q):
    return R.from_quat(np.c_[q[:, 1], q[:, 2], q[:, 3], q[:, 0]]).as_euler("ZYX")[:, ::-1]


att, asp, mot = cut(U["att"]), cut(U["att_sp"]), cut(U["mot"])
ea, es = eul(att[:, 1:5]), eul(asp[:, 1:5])
ea[:, 2] = np.unwrap(ea[:, 2]); es[:, 2] = np.unwrap(es[:, 2])
es_i = np.stack([np.interp(att[:, 0], asp[:, 0], es[:, k]) for k in range(3)], 1)
aerr = np.degrees(es_i - ea)
metrics["squareB_inner"] = dict(
    roll_err_rms_deg=float(np.sqrt(np.mean(aerr[:, 0] ** 2))), pitch_err_rms_deg=float(np.sqrt(np.mean(aerr[:, 1] ** 2))),
    roll_err_max_deg=float(np.abs(aerr[:, 0]).max()), pitch_err_max_deg=float(np.abs(aerr[:, 1]).max()),
    yaw_err_rms_deg=float(np.sqrt(np.mean(aerr[:, 2] ** 2))), mean_roll_deg=float(np.degrees(ea[:, 0].mean())),
    mean_pitch_deg=float(np.degrees(ea[:, 1].mean())),
    motor_mean=mot[:, 1:].mean(0).tolist(), motor_min=float(mot[:, 1:].min()), motor_max=float(mot[:, 1:].max()),
    thrust_mean=float(-cut(U["thrust"])[:, 1].mean()))
ta, tsp_ = att[:, 0] - pto, asp[:, 0] - pto
fig, ax = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
for k, (name, lim) in enumerate((("Roll", None), ("Pitch", None), ("Yaw", None))):
    ax[k].plot(tsp_, np.degrees(es[:, k]), color="0.55", lw=1.6, label="setpoint")
    ax[k].plot(ta, np.degrees(ea[:, k]), "C0", lw=0.9, label="actual")
    ax[k].set_ylabel(f"{name.lower()} [deg]")
    ax[k].set_title(f"({'abc'[k]}) {name}: tracking error RMS {np.sqrt(np.mean(aerr[:, k] ** 2)):.2f}°")
ax[0].legend(loc="upper right", fontsize=8)
ax[0].axhline(metrics["squareB_inner"]["mean_roll_deg"], color="C3", ls="--", lw=0.8)
ax[0].text(ta[5], metrics["squareB_inner"]["mean_roll_deg"] + 0.25, f"mean {metrics['squareB_inner']['mean_roll_deg']:+.1f}°",
           color="C3", fontsize=8)
names = ["1 front-right", "2 rear-left", "3 front-left", "4 rear-right"]
for j in range(4):
    ax[3].plot(mot[:, 0] - pto, mot[:, j + 1], lw=0.7, label=f"motor {names[j]}")
ax[3].set_ylim(0, 1); ax[3].set_ylabel("command [0–1]"); ax[3].set_xlabel("time since takeoff [s]")
ax[3].set_title("(d) Motor commands"); ax[3].legend(ncol=4, fontsize=8, loc="upper right")
fig.suptitle("Square B: attitude tracking and motor commands (takeoff to final turn)", fontsize=12)
save(fig, "fig_c5_inner_loop.png")

json.dump(metrics, open(HERE / "controls_metrics.json", "w"), indent=1)
print(json.dumps(metrics, indent=1))
