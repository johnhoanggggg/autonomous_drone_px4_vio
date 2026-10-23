#!/usr/bin/env python3
"""Square B (offboard_square_20261005T034924Z) only: Vicon vs PX4 EKF and VIO input.

Report figure: horizontal path with the commanded square, horizontal error,
height above takeoff, heading error, and a metrics table.
"""
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from compare import analyse, load_vicon, interp_valid, rz, HERE, VICON_DIR

VICON, BAG = "john_1.csv", "sq_20261005T034924Z.npz"
# Measurement ends when the final 90 deg turn back to the start heading begins
# (rosout "[offboard_square] -> TURN" for side 4/4, bag time 55.35 s): the drift during that
# turn and the landing are excluded.
T_FINAL_TURN = 55.35
figs = []
r = analyse(VICON, BAG, "square B (034924Z)", figs, t_end=T_FINAL_TURN)
f = figs[0]
json.dump(r, open(HERE / "metrics_square_b.json", "w"), indent=1)

xg = np.array(r["fit_params"]); off = r["time_offset_s"]; lever = xg[4:7]
t_to, t_td = r["airborne_bag_s"]
tv, pv, rv = load_vicon(VICON_DIR / VICON)
d = np.load(HERE / BAG)
lp = d["fmu__out__vehicle_local_position_v1"]; sp = d["fmu__in__trajectory_setpoint"]
tb0 = lp[0, 0]


def vicon_fc(t):
    t = np.atleast_1d(t)
    p = interp_valid(tv, t + off, pv)
    a = interp_valid(tv, t + off, rv)
    return p + np.einsum("nij,j->ni", R.from_euler("xyz", a).as_matrix(), lever)


def ned_to_vicon(ned):
    return np.c_[ned[:, 0], -ned[:, 1], -ned[:, 2]] @ rz(xg[0]).T + xg[1:4]


# at rest before spool-up and after the post-touchdown skid (the drone slides on the floor at both ends)
start_rest = vicon_fc(t_to - 2.0)[0]
liftoff = vicon_fc(t_to)[0]
touchdown = vicon_fc(t_td + 0.5)[0]
end_rest = vicon_fc(t_td + 2.0)[0]
tg = np.arange(t_to - 2.0, t_td + 2.0, 0.01)
ground = vicon_fc(tg)
w0, w1 = r["window_bag_s"]
o = start_rest[:2]
G = ground - [*o, 0]
V = f["fc_v"] - [*o, 0]
E = f["est"] - [*o, 0]
t = f["t"] + 1.0                      # analysis window starts 1 s after takeoff
te = f["ev"]["t"] + 1.0
# VIO input is the camera point; its error was fitted with its own lever arm, so draw it at the FC point
EV = np.stack([np.interp(te, t, V[:, i]) for i in range(3)], 1) + f["ev"]["err"]
ts = sp[:, 0] - tb0
air = (ts > t_to) & (ts < T_FINAL_TURN)
SP = ned_to_vicon(sp[air, 2:5]) - [*o, 0]

ev = r["ev_input"]
rows = [
    ("Horizontal RMSE", f"{100*r['horiz']['rmse']:.1f} cm", f"{100*ev['horiz']['rmse']:.1f} cm"),
    ("Horizontal p95 / max", f"{100*r['horiz']['p95']:.1f} / {100*r['horiz']['max']:.1f} cm",
     f"{100*ev['horiz']['p95']:.1f} / {100*ev['horiz']['max']:.1f} cm"),
    ("Vertical RMSE / max", f"{100*r['vert']['rmse']:.1f} / {100*r['vert']['max']:.1f} cm",
     f"{100*ev['vert']['rmse']:.1f} / {100*ev['vert']['max']:.1f} cm"),
    ("Heading RMSE / max", f"{r['heading_deg']['rmse']:.2f} / {r['heading_deg']['max']:.2f}°", "–"),
    ("Horizontal scale", f"{r['scale_xy']:.3f}", "–"),
    ("Horizontal drift at end", f"{100*r['drift_firstalign']['final_horiz_m']:.1f} cm "
     f"({r['drift_firstalign']['pct_of_path']:.1f}% of path)", "–"),
    ("Height drift at end", f"{100*r['drift_firstalign']['final_vert_m']:+.0f} cm", "–"),
    ("Update rate", "50 Hz", f"{ev['rate_hz']:.0f} Hz"),
]

def draw_path(a):
    a.plot(SP[:, 0], SP[:, 1], color="0.6", ls="--", lw=1.2, label="Commanded path")
    a.plot(V[:, 0], V[:, 1], "k-", lw=2.2, label="Vicon (ground truth)")
    a.plot(E[:, 0], E[:, 1], "C0-", lw=1.3, label="PX4 EKF estimate")
    a.set_aspect("equal"); a.grid(alpha=.3)
    a.set_xlabel("x (Vicon frame) [m]"); a.set_ylabel("y (Vicon frame) [m]")


plt.rcParams.update({"font.size": 10})
fig = plt.figure(figsize=(13, 10))
gs = fig.add_gridspec(4, 2, width_ratios=[1.0, 1.25], height_ratios=[1, 1, 1, 0.08])

a = fig.add_subplot(gs[0:2, 0])
draw_path(a)
a.set_title("(a) Horizontal path")
a.legend(fontsize=8.5, loc="center")

tb = fig.add_subplot(gs[2, 0]); tb.axis("off")
tab = tb.table(cellText=[[k, e, v] for k, e, v in rows], colLabels=["Metric", "PX4 EKF", "VIO input"],
               colWidths=[0.40, 0.36, 0.24], loc="center", cellLoc="left")
tab.auto_set_font_size(False); tab.set_fontsize(9); tab.scale(1, 1.45)
for (i, j), c in tab.get_celld().items():
    c.set_edgecolor("0.75")
    if i == 0:
        c.set_text_props(weight="bold"); c.set_facecolor("0.92")
tb.set_title("(e) Accuracy against Vicon", pad=2)

b = fig.add_subplot(gs[0, 1])
b.plot(t, 100 * f["eh"], "C0", label="PX4 EKF")
b.plot(te, 100 * f["ev"]["eh"], "C1:", label="VIO input")
b.set_ylabel("error [cm]"); b.grid(alpha=.3); b.legend(fontsize=8.5, loc="upper left")
b.set_title("(b) Horizontal position error")

c = fig.add_subplot(gs[1, 1], sharex=b)
c.plot(t, 100 * f["h_vicon"], "k", lw=2, label="Vicon")
c.plot(t, 100 * f["h_px4"], "C0", label="PX4 EKF")
c.plot(t, 100 * (f["h_px4"] - f["h_vicon"]), "C3", lw=1, label="PX4 − Vicon")
c.axhline(0, color="0.5", lw=.6)
c.set_ylabel("height [cm]"); c.grid(alpha=.3); c.legend(fontsize=8.5, loc="center right")
c.set_title("(c) Height above takeoff")

dd = fig.add_subplot(gs[2, 1], sharex=b)
dd.plot(t, f["hdg"], "C2")
dd.axhline(0, color="0.5", lw=.6)
dd.set_ylabel("error [deg]"); dd.set_xlabel("time since takeoff [s] (measurement ends at start of final turn)"); dd.grid(alpha=.3)
dd.set_title("(d) Heading error")
for ax in (b, c):
    plt.setp(ax.get_xticklabels(), visible=False)

fig.suptitle("Square B (2026-10-05 03:49Z): PX4 internal estimate vs Vicon motion capture, takeoff to final turn", fontsize=13)
fig.tight_layout()
fig.savefig(HERE / "square_b_panels.png", dpi=150)

# square_b.png: horizontal path only
fig2, ax = plt.subplots(figsize=(7.5, 7.5))
draw_path(ax)
ax.legend(fontsize=9, loc="center")
ax.set_title("Square B: horizontal path, PX4 estimate vs Vicon")
fig2.tight_layout()
fig2.savefig(HERE / "square_b.png", dpi=150)
print(dict(window=r["window_bag_s"], seconds=r["window_bag_s"][1]-r["window_bag_s"][0], path=r["path_len_m"]))
cm = lambda u, v: round(float(100 * np.linalg.norm(u[:2] - v[:2])), 1)
print(dict(spoolup_skid_cm=cm(liftoff, start_rest), liftoff_to_touchdown_cm=cm(touchdown, liftoff),
           landing_skid_cm=cm(end_rest, touchdown), rest_to_rest_cm=cm(end_rest, start_rest),
           touchdown_vs_start_cm=cm(touchdown, start_rest)))
