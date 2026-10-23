#!/usr/bin/env python3
"""Vicon (external) vs PX4 EKF2 / VIO-input (internal) tracking, 2026-10-05 flights.

Inputs: Vicon CSV exports (100 Hz, object "johnh") and npz extracts of the matching
rosbags made with extract_bag.py.  PX4 NED is mapped into a z-up frame
(x=N, y=-E, z=-D) and fitted to Vicon with yaw + translation + a body lever arm
(no scale), after a time-offset search.
"""
import csv, json, sys
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

HERE = Path(__file__).resolve().parent
FLIGHTS = [
    ("john.csv", "sq_20261005T034525Z.npz", "square A (034525Z)"),
    ("john_1.csv", "sq_20261005T034924Z.npz", "square B (034924Z)"),
    ("john_2.csv", "p3_20261005T053837Z.npz", "planner3d (053837Z)"),
]
VICON_DIR = Path("/home/john")


def load_vicon(path, obj="johnh"):
    rows = list(csv.reader(open(path, encoding="utf-8-sig")))
    col = [i for i, h in enumerate(rows[2]) if h.endswith(obj + ":" + obj)][0]
    f, d = [], []
    for r in rows[5:]:
        if not r or not r[0].strip():
            continue
        v = r[col:col + 6]
        f.append(int(r[0]))
        d.append([float(x) if x.strip() else np.nan for x in v] if len(v) == 6 else [np.nan] * 6)
    f = np.array(f); d = np.array(d)
    return (f - f[0]) / 100.0, d[:, 3:6] / 1000.0, d[:, 0:3]


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rz(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def interp_valid(t, tq, y):
    ok = ~np.isnan(y[:, 0])
    out = np.stack([np.interp(tq, t[ok], y[ok, i]) for i in range(y.shape[1])], 1)
    # mask query times that fall inside a Vicon gap > 0.05 s
    idx = np.clip(np.searchsorted(t[ok], tq), 1, ok.sum() - 1)
    gap = t[ok][idx] - t[ok][idx - 1]
    out[(gap > 0.05) | (tq < t[ok][0]) | (tq > t[ok][-1])] = np.nan
    return out


def residual(params, pv, Rv, q):
    th, tx, ty, tz, lx, ly, lz = params
    fc_v = pv + np.einsum("nij,j->ni", Rv, [lx, ly, lz])
    return (fc_v - (q @ rz(th).T + [tx, ty, tz])).ravel()


def fit(pv, Rv, q, x0=None, lever=True, lever_fixed=None):
    x0 = np.zeros(7) if x0 is None else x0
    if not lever:
        lf = np.zeros(3) if lever_fixed is None else lever_fixed
        f = lambda p: residual(np.r_[p, lf], pv, Rv, q)
        best = None
        for th0 in np.linspace(-np.pi, np.pi, 12, endpoint=False):
            c = (pv - q @ rz(th0).T).mean(0)
            s = least_squares(f, np.r_[th0, c])
            if best is None or s.cost < best.cost:
                best = s
        return np.r_[best.x, lf]
    best = None
    for th0 in np.linspace(-np.pi, np.pi, 12, endpoint=False):
        c = (pv - q @ rz(th0).T).mean(0)
        # lever arm z is not observable in near-level flight (same as tz): pin it to 0
        s = least_squares(lambda p: residual(np.r_[p, 0.0], pv, Rv, q), np.r_[th0, c, 0, 0])
        if best is None or s.cost < best.cost:
            best = s
    return np.r_[best.x, 0.0]


def stats(e):
    e = e[~np.isnan(e)]
    return dict(rmse=float(np.sqrt(np.mean(e ** 2))), p95=float(np.percentile(np.abs(e), 95)),
                max=float(np.max(np.abs(e))), mean=float(np.mean(e)))


def analyse(vfile, npz, label, figs, t_end=None):
    tv, pv_all, rv_all = load_vicon(VICON_DIR / vfile)
    d = np.load(HERE / npz)
    lp = d["fmu__out__vehicle_local_position_v1"]
    ev = d["fmu__in__vehicle_visual_odometry"]
    st = d["fmu__out__vehicle_status_v1"]
    tb0 = lp[0, 0]
    tl = lp[:, 0] - tb0
    q_lp = np.c_[lp[:, 2], -lp[:, 3], -lp[:, 4]]
    yaw_lp = -lp[:, 5]
    armed = st[st[:, 2] == 2, 0] - tb0
    arm_t = (armed.min(), armed.max())

    # ---- time offset: Vicon time = bag time + off.  Coarse on height, fine on 3D fit.
    zg = np.nanmedian(pv_all[:200, 2]) if not np.isnan(pv_all[:200, 2]).all() else np.nanmin(pv_all[:, 2])
    zv_ground = np.nanpercentile(pv_all[:, 2], 2)
    best = (np.inf, 0)
    for off in np.arange(-40, 40, 0.05):
        zv = interp_valid(tv, tl + off, pv_all[:, 2:3])[:, 0]
        ok = ~np.isnan(zv)
        if ok.sum() < 0.5 * len(tl):
            continue
        a = zv[ok] - zv[ok].mean(); b = q_lp[ok, 2] - q_lp[ok, 2].mean()
        c = np.mean((a - b) ** 2)
        if c < best[0]:
            best = (c, off)
    off0 = best[1]

    # airborne window from Vicon height (> 8 cm above ground), in bag time
    zv_on_lp = interp_valid(tv, tl + off0, pv_all[:, 2:3])[:, 0]
    air = (zv_on_lp > zv_ground + 0.08) & (tl > arm_t[0]) & (tl < arm_t[1])
    ia = np.where(air)[0]
    t_air = (tl[ia[0]], tl[ia[-1]])
    # t_end (bag time) optionally cuts the measurement window before the end of the airborne segment
    t_hi = t_air[1] - 1.0 if t_end is None else min(t_air[1] - 1.0, t_end)
    sel = (tl >= t_air[0] + 1.0) & (tl <= t_hi)

    def at(off, mask):
        pv = interp_valid(tv, tl[mask] + off, pv_all)
        rv = interp_valid(tv, tl[mask] + off, rv_all)
        ok = ~np.isnan(pv[:, 0])
        Rv = R.from_euler("xyz", rv[ok]).as_matrix()
        return pv[ok], Rv, q_lp[mask][ok], ok, rv[ok]

    fine = []
    x = None
    for off in np.arange(off0 - 0.6, off0 + 0.6, 0.02):
        pv, Rv, q, ok, _ = at(off, sel)
        x = fit(pv, Rv, q, lever=False)
        fine.append((np.mean(residual(x, pv, Rv, q) ** 2), off))
    off = min(fine)[1]
    pv, Rv, q, ok, rv = at(off, sel)
    xg = fit(pv, Rv, q)               # yaw, t, lever arm on whole airborne segment
    lever = xg[4:7]

    def errors(params, pv, Rv, q):
        r = residual(params, pv, Rv, q).reshape(-1, 3)
        return -r  # estimate - truth

    e = errors(xg, pv, Rv, q)
    eh = np.linalg.norm(e[:, :2], axis=1)
    tsel = tl[sel][ok]
    fc_v = pv + np.einsum("nij,j->ni", Rv, lever)
    path_len = float(np.sum(np.linalg.norm(np.diff(fc_v[:, :2], axis=0), axis=1)))

    # heading: Vicon yaw vs PX4 heading rotated into Vicon frame, constant mount offset removed
    yv = rv[:, 2]
    yp = yaw_lp[sel][ok] + xg[0]
    dy = wrap(yp - yv)
    mount = np.angle(np.mean(np.exp(1j * dy)))
    hdg_err = np.degrees(wrap(dy - mount))

    # drift: align on first 3 s of the airborne segment only (lever from global fit)
    first = tsel < tsel[0] + 3.0
    x3 = fit(pv[first], Rv[first], q[first], lever=False, lever_fixed=lever)
    e3 = errors(x3, pv, Rv, q)
    eh3 = np.linalg.norm(e3[:, :2], axis=1)

    # scale diagnostic: horizontal and vertical extents (FC point)
    def ext(a):
        return np.percentile(a, 98) - np.percentile(a, 2)
    q_rot = q @ rz(xg[0]).T
    scale_xy = float(np.hypot(ext(q_rot[:, 0]), ext(q_rot[:, 1])) / np.hypot(ext(fc_v[:, 0]), ext(fc_v[:, 1])))
    climb_v = float(np.percentile(fc_v[:, 2], 95) - zv_ground)
    # vertical error model: e_z = k*(height above ground) + tilt plane in x,y + c
    A = np.c_[fc_v[:, 2] - zv_ground, fc_v[:, 0] - fc_v[:, 0].mean(), fc_v[:, 1] - fc_v[:, 1].mean(),
              np.ones(len(fc_v))]
    coef, *_ = np.linalg.lstsq(A, e[:, 2], rcond=None)
    vres = e[:, 2] - A @ coef
    vmodel = dict(height_scale_err=float(coef[0]), tilt_deg=float(np.degrees(np.arctan(np.hypot(coef[1], coef[2])))),
                  rmse_after_model=float(np.sqrt(np.mean(vres ** 2))))
    A2 = A[:, [1, 2, 3]]
    c2, *_ = np.linalg.lstsq(A2, e[:, 2], rcond=None)
    vmodel["tilt_only_deg"] = float(np.degrees(np.arctan(np.hypot(c2[0], c2[1]))))
    vmodel["rmse_after_tilt_only"] = float(np.sqrt(np.mean((e[:, 2] - A2 @ c2) ** 2)))
    A3 = A[:, [0, 3]]
    c3, *_ = np.linalg.lstsq(A3, e[:, 2], rcond=None)
    vmodel["scale_only"] = float(c3[0])
    vmodel["rmse_after_scale_only"] = float(np.sqrt(np.mean((e[:, 2] - A3 @ c3) ** 2)))
    z_lp_ground = float(np.median(q_lp[tl < arm_t[0], 2]))
    climb_px4 = float(np.percentile(q[:, 2], 95) - z_lp_ground)

    # velocity: PX4 vx,vy vs Vicon derivative (FC point, 0.1 s central difference)
    tv_ok = ~np.isnan(pv_all[:, 0])
    tq = tl[sel][ok] + off
    h = 0.05
    p1 = interp_valid(tv, tq + h, pv_all); p0 = interp_valid(tv, tq - h, pv_all)
    r1 = interp_valid(tv, tq + h, rv_all); r0 = interp_valid(tv, tq - h, rv_all)
    good = ~np.isnan(p1[:, 0]) & ~np.isnan(p0[:, 0])
    f1 = p1[good] + np.einsum("nij,j->ni", R.from_euler("xyz", r1[good]).as_matrix(), lever)
    f0 = p0[good] + np.einsum("nij,j->ni", R.from_euler("xyz", r0[good]).as_matrix(), lever)
    vv = (f1 - f0) / (2 * h)
    vl = np.c_[lp[sel][ok][good, 8], -lp[sel][ok][good, 9], -lp[sel][ok][good, 10]] @ rz(xg[0]).T
    ve = vl - vv
    speed = np.linalg.norm(vv[:, :2], axis=1)

    # post-landing: Vicon stationary, how far does the PX4 estimate move
    post = {}
    land_lp = tl > t_air[1] + 0.5
    if land_lp.sum() > 10:
        pvp = interp_valid(tv, tl[land_lp] + off, pv_all)
        okp = ~np.isnan(pvp[:, 0])
        qp = q_lp[land_lp][okp] @ rz(xg[0]).T
        post = dict(duration_s=float(tl[land_lp][-1] - tl[land_lp][0]),
                    vicon_move_m=float(np.linalg.norm(np.ptp(pvp[okp, :2], axis=0))),
                    px4_move_xy_m=float(np.linalg.norm(qp[-1, :2] - qp[0, :2])),
                    px4_move_z_m=float(qp[-1, 2] - qp[0, 2]))

    # ---- VIO position fed to PX4 (EV input), same frame as PX4 local position
    evr = {}
    ev_fig = None
    if len(ev):
        te = ev[:, 0] - tb0
        qe = np.c_[ev[:, 3], -ev[:, 4], -ev[:, 5]]
        lag_sample = np.median(ev[:, 1] - ev[:, 2])
        msk = (te >= t_air[0] + 1.0) & (te <= t_hi)
        best_e = []
        for o in np.arange(off - 0.6, off + 0.4, 0.02):
            pve = interp_valid(tv, te[msk] + o, pv_all)
            oke = ~np.isnan(pve[:, 0])
            Rve = R.from_euler("xyz", interp_valid(tv, te[msk] + o, rv_all)[oke]).as_matrix()
            xe = fit(pve[oke], Rve, qe[msk][oke], lever=False, lever_fixed=lever)
            best_e.append((np.mean(residual(xe, pve[oke], Rve, qe[msk][oke]) ** 2), o, xe))
        _, oe, xe = min(best_e, key=lambda b: b[0])
        pve = interp_valid(tv, te[msk] + oe, pv_all); oke = ~np.isnan(pve[:, 0])
        Rve = R.from_euler("xyz", interp_valid(tv, te[msk] + oe, rv_all)[oke]).as_matrix()
        # EV is the camera point (EKF2_EV_POS_X 0.10), not the FC: give it its own lever arm
        xe = fit(pve[oke], Rve, qe[msk][oke])
        ee = errors(xe, pve[oke], Rve, qe[msk][oke])
        ev_fig = dict(t=te[msk][oke] - tsel[0], err=ee,
                      eh=np.linalg.norm(ee[:, :2], axis=1), ez=ee[:, 2])
        rate = len(te) / (te[-1] - te[0])
        evr = dict(rate_hz=float(rate), lag_vs_ekf_s=float(off - oe), sample_age_s=float(lag_sample),
                   horiz=stats(np.linalg.norm(ee[:, :2], axis=1)), vert=stats(ee[:, 2]),
                   yaw_vs_ekf_fit_deg=float(np.degrees(wrap(xe[0] - xg[0]))),
                   lever_minus_ekf_lever_m=(xe[4:6] - lever[:2]).tolist())

    res = dict(
        label=label, vicon_file=vfile, bag=npz,
        time_offset_s=float(off), airborne_bag_s=[float(t_air[0]), float(t_air[1])],
        airborne_duration_s=float(t_air[1] - t_air[0]), path_len_m=path_len,
        window_bag_s=[float(t_air[0] + 1.0), float(t_hi)],
        frame_yaw_deg=float(np.degrees(xg[0])), lever_arm_m=lever.tolist(), fit_params=xg.tolist(),
        horiz=stats(eh), vert=stats(e[:, 2]), err3d_rmse=float(np.sqrt(np.mean(np.sum(e ** 2, 1)))),
        heading_deg=stats(hdg_err), heading_mount_deg=float(np.degrees(mount)),
        drift_firstalign=dict(final_horiz_m=float(np.mean(eh3[-50:])), max_horiz_m=float(eh3.max()),
                              final_vert_m=float(np.mean(e3[-50:, 2])),
                              pct_of_path=float(100 * np.mean(eh3[-50:]) / path_len)),
        scale_xy=scale_xy, climb_vicon_m=climb_v, vert_model=vmodel, climb_px4_m=climb_px4,
        vel_err=dict(horiz=stats(np.linalg.norm(ve[:, :2], axis=1)), vert=stats(ve[:, 2]),
                     mean_speed=float(speed.mean()), max_speed=float(speed.max())),
        post_landing=post, ev_input=evr,
    )
    figs.append(dict(label=label, t=tsel - tsel[0], fc_v=fc_v, est=q @ rz(xg[0]).T + xg[1:4],
                     est3=q @ rz(x3[0]).T + x3[1:4], eh=eh, ez=e[:, 2], eh3=eh3, hdg=hdg_err,
                     ex=e[:, 0], ey=e[:, 1], ev=ev_fig,
                     h_vicon=fc_v[:, 2] - zv_ground, h_px4=q[:, 2] - z_lp_ground))
    return res


def plot(figs, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = len(figs)
    fig, ax = plt.subplots(3, n, figsize=(5.2 * n, 12))
    for j, f in enumerate(figs):
        a = ax[0, j]
        a.plot(f["fc_v"][:, 0], f["fc_v"][:, 1], "k-", lw=2, label="Vicon")
        a.plot(f["est"][:, 0], f["est"][:, 1], "C0-", lw=1.2, label="PX4 EKF (best fit)")
        a.plot(f["est3"][:, 0], f["est3"][:, 1], "C3--", lw=1, label="PX4 EKF (aligned first 3 s)")
        a.set_aspect("equal"); a.set_title(f["label"]); a.set_xlabel("x [m]"); a.set_ylabel("y [m]"); a.grid(alpha=.3)
        if j == 0:
            a.legend(fontsize=8)
        a = ax[1, j]
        a.plot(f["t"], 100 * f["eh"], "C0", label="horizontal (best fit)")
        a.plot(f["t"], 100 * f["eh3"], "C3--", label="horizontal (first-3 s align)")
        a.plot(f["t"], 100 * f["ez"], "C2", label="vertical")
        a.set_ylabel("error [cm]"); a.set_xlabel("time airborne [s]"); a.grid(alpha=.3)
        if j == 0:
            a.legend(fontsize=8)
        a = ax[2, j]
        a.plot(f["t"], f["hdg"], "C1")
        a.set_ylabel("heading error [deg]"); a.set_xlabel("time airborne [s]"); a.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(out, dpi=110)


if __name__ == "__main__":
    figs, out = [], []
    for v, n, l in FLIGHTS:
        out.append(analyse(v, n, l, figs))
    json.dump(out, open(HERE / "metrics.json", "w"), indent=1)
    plot(figs, HERE / "vicon_vs_px4.png")
    print(json.dumps(out, indent=1))
