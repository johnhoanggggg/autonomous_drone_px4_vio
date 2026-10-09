"""Bump test: external horizontal force (N) and displacement from setpoint (cm).

External force = mass x body-frame horizontal specific force. Thrust acts along
body z only, so body x/y of the accelerometer is contact force + drag, whatever
the tilt. Rotated into NED with the attitude quaternion for plotting.
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pyulog import ULog

MASS = 1.8
EVENT_THRESH = 3.0 * MASS      # N
OUT = sys.argv[1]
LOGS = sys.argv[2:]


def qrot(q, v):
    w, x, y, z = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    return np.einsum("ijn,jn->in", R, v)


def load(path):
    u = ULog(path, ["vehicle_status", "vehicle_acceleration", "vehicle_attitude",
                    "vehicle_local_position", "trajectory_setpoint",
                    "vehicle_angular_velocity"])
    d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
    vs = d["vehicle_status"]
    armed = vs["timestamp"][vs["arming_state"] == 2]
    t0 = armed[0]
    land = vs["timestamp"][(vs["nav_state"] == 18) & (vs["arming_state"] == 2)]
    t_land = (land[0] - t0) / 1e6 if len(land) else None
    t_end = (armed[-1] - t0) / 1e6

    a = d["vehicle_acceleration"]
    ta = (a["timestamp"] - t0) / 1e6
    att = d["vehicle_attitude"]
    q = np.vstack([np.interp(a["timestamp"], att["timestamp"], att[f"q[{i}]"]) for i in range(4)])
    fb = np.vstack([a["xyz[0]"], a["xyz[1]"], np.zeros_like(ta)])
    fw = qrot(q, fb) * MASS                    # N, NED
    F = np.vstack([fw[0], fw[1]])

    lp = d["vehicle_local_position"]
    tl = (lp["timestamp"] - t0) / 1e6
    ts = d["trajectory_setpoint"]
    sp = [np.interp(lp["timestamp"], ts["timestamp"], ts[f"position[{i}]"]) for i in range(3)]
    D = np.vstack([lp["x"] - sp[0], lp["y"] - sp[1], -(lp["z"] - sp[2])]) * 100  # cm, N/E/Up
    # The offboard setpoint is stale once AUTO.LAND takes over; stop there.
    if t_land is not None:
        D[:, tl > t_land] = np.nan
    av = d["vehicle_angular_velocity"]
    tw = (av["timestamp"] - t0) / 1e6
    yr = np.degrees(av["xyz[2]"])

    # Events: cluster samples above threshold, only before the land switch.
    mag = np.hypot(*F)
    t_cut = t_land if t_land is not None else t_end
    idx = np.where((mag > EVENT_THRESH) & (ta < t_cut))[0]
    events = []
    for i in idx:
        if events and ta[i] - events[-1][1] < 0.4:
            events[-1][1] = ta[i]
        else:
            events.append([ta[i], ta[i]])
    rows = []
    dh = np.hypot(D[0], D[1])
    for s, e in events:
        w = (ta >= s - 0.05) & (ta <= e + 0.05)
        dt = np.gradient(ta[w])
        J = np.array([np.sum(F[0][w] * dt), np.sum(F[1][w] * dt)])   # N*s
        pk = mag[w].max()
        nxt = [ev[0] for ev in events if ev[0] > e]
        win_end = min(nxt[0] if nxt else t_cut, t_cut)
        wl = (tl >= s) & (tl < win_end) & np.isfinite(dh)
        k = np.argmax(dh[wl])
        dmax, tmax = dh[wl][k], tl[wl][k]
        back = tl[wl][(tl[wl] > tmax) & (dh[wl] < 5.0)]
        rec = back[0] - s if len(back) else None
        along = (D[0][wl] * J[0] + D[1][wl] * J[1]) / np.hypot(*J)
        overshoot = max(0.0, -np.nanmin(along[tl[wl] > tmax])) if np.any(tl[wl] > tmax) else 0.0
        ww = (tw >= s) & (tw < s + 0.6)
        yrmax = np.abs(yr[ww]).max()
        cut = win_end == t_cut and t_land is not None and t_land - s < 1.0
        dirn = np.degrees(np.arctan2(J[1], J[0])) % 360
        rows.append(dict(t=s, peak=pk, J=np.hypot(*J), dirn=dirn, dmax=dmax,
                         tmax=tmax - s, rec=rec, Jvec=J, over=overshoot, yr=yrmax, cut=cut))
    return dict(path=path, ta=ta, F=F, tl=tl, D=D, t_land=t_land, t_end=t_end,
                rows=rows, dh=dh, tw=tw, yr=yr)


runs = [load(p) for p in LOGS]
fig, axes = plt.subplots(4, len(runs), figsize=(8 * len(runs), 12.5), sharex="col",
                         squeeze=False)
for c, r in enumerate(runs):
    name = r["path"].split("/")[-1].replace(".ulg", "")
    ax = axes[0][c]
    ax.plot(r["ta"], r["F"][0], lw=0.8, label="North")
    ax.plot(r["ta"], r["F"][1], lw=0.8, label="East")
    ax.plot(r["ta"], np.hypot(*r["F"]), lw=0.8, color="k", alpha=0.5, label="|F|")
    for i, row in enumerate(r["rows"]):
        ax.annotate(f"#{i+1}\n{row['J']:.2f} N·s", (row["t"], row["peak"]),
                    textcoords="offset points", xytext=(4, -2), fontsize=8)
    ax.set_ylabel("external horizontal force (N)")
    ax.set_title(f"{name}  (1.8 kg; peaks low-passed at 30 Hz, impulse is exact)")
    ax.legend(loc="upper left", fontsize=8)

    ax = axes[1][c]
    ax.plot(r["tl"], r["D"][0], label="North")
    ax.plot(r["tl"], r["D"][1], label="East")
    ax.plot(r["tl"], r["dh"], color="k", alpha=0.6, label="|horizontal|")
    for row in r["rows"]:
        ax.annotate(f"{row['dmax']:.0f} cm" + (" (cut by LAND)" if row["cut"] else ""),
                    (row["t"] + row["tmax"], row["dmax"]),
                    textcoords="offset points", xytext=(2, 3), fontsize=8)
    ax.set_ylabel("displacement from setpoint (cm)")
    ax.legend(loc="upper left", fontsize=8)

    ax = axes[2][c]
    ax.plot(r["tl"], r["D"][2], color="tab:green")
    ax.set_ylabel("height error (cm, +up)")
    ax = axes[3][c]
    ax.plot(r["tw"], r["yr"], color="tab:purple", lw=0.8)
    ax.axhline(60, color="r", lw=0.8, ls="--"); ax.axhline(-60, color="r", lw=0.8, ls="--")
    ax.text(0.2, 63, "offboard_hover abort: 60 deg/s for 0.1 s", color="r", fontsize=8)
    ax.set_ylabel("yaw rate (deg/s)")
    ax.set_ylim(-170, 170)
    ax.set_xlabel("time since arm (s)")
    for row in axes[:, c]:
        if r["t_land"] is not None:
            row.axvspan(r["t_land"], r["t_end"], color="0.85", zorder=0)
            row.axvline(r["t_land"], color="r", ls="--", lw=0.8)
        for ev in r["rows"]:
            row.axvline(ev["t"], color="0.6", lw=0.5, ls=":")
        row.grid(alpha=0.3)
    axes[0][c].text(r["t_land"], axes[0][c].get_ylim()[1] * 0.9, " AUTO.LAND",
                    color="r", fontsize=8) if r["t_land"] is not None else None
    axes[1][c].set_ylim(-30, 70)
fig.tight_layout()
fig.savefig(OUT + "_timeline.png", dpi=110)

# Response overlay + impulse vs displacement
fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 5))
for r in runs:
    tag = r["path"].split("_")[-1].replace(".ulg", "")
    for i, row in enumerate(r["rows"]):
        w = (r["tl"] >= row["t"] - 0.2) & (r["tl"] <= row["t"] + 2.5)
        a1.plot(r["tl"][w] - row["t"], r["dh"][w], label=f"{tag} #{i+1}  {row['J']:.2f} N·s")
        a2.scatter(row["J"], row["dmax"], s=40, marker="x" if row["cut"] else "o")
        a2.annotate(f"{tag} #{i+1}" + (" (cut by LAND)" if row["cut"] else ""), (row["J"], row["dmax"]), fontsize=8,
                    textcoords="offset points", xytext=(4, 2))
a1.axhline(5, color="0.5", ls="--", lw=0.8)
a1.set_xlabel("time since bump (s)"); a1.set_ylabel("horizontal displacement (cm)")
a1.set_title("response to each bump"); a1.legend(fontsize=7); a1.grid(alpha=0.3)
a2.set_xlabel("impulse (N·s)"); a2.set_ylabel("peak displacement (cm)")
Js = np.linspace(0, 3.8, 2)
a2.plot(Js, Js / MASS * 0.30 * 100, color="0.5", ls="--", lw=0.8, label="0.30 s x delta-v")
a2.legend(fontsize=8)
a2.set_title("push size vs how far it moved"); a2.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT + "_response.png", dpi=110)

for r in runs:
    print(r["path"], "land at", r["t_land"])
    for i, row in enumerate(r["rows"]):
        print(f"  #{i+1} t={row['t']:.2f}s peak={row['peak']:.0f}N impulse={row['J']:.3f}N*s "
              f"dir={row['dirn']:.0f}deg(NED) dmax={row['dmax']:.1f}cm at +{row['tmax']:.2f}s "
              f"back<5cm={'%.2fs' % row['rec'] if row['rec'] is not None else 'never'} "
              f"overshoot={row['over']:.1f}cm yawrate={row['yr']:.0f} dv={row['J']/MASS:.2f}m/s"
              f"{' CUT' if row['cut'] else ''}")
