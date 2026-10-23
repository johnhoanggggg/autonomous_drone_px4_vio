#!/usr/bin/env python3
"""Compute load and memory for the latest planner3d flight (offboard_planner_3d_unknown_20261009T050943Z).

Run with /home/john/sai_venv/bin/python3 after extract_perf.py. Writes compute_metrics.json and
fig_compute.png. CPU per process is psutil-style (100% = one core); machine CPU is 0-100% of 4 cores.
"""
import json
import pickle
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
RAM_MB = 7937            # Pi 5, `free -m` total
d = pickle.load(open(HERE / "perf.pkl", "rb"))
st = d["status"]
armed = st[st[:, 1] == 2, 0]
a0, a1 = armed[0], armed[-1]
tp = np.array([t for t, _ in d["procs"]])
t0 = tp[0]
m = (tp >= a0) & (tp <= a1)

NAMES = {
    "oak_container": "Camera driver + RTAB-Map VIO",
    "rtabmap_octomap_node": "OctoMap (3D occupancy map)",
    "cpp_astar_planner_3d": "3D A* planner",
    "mapper_container": "RTAB-Map mapper",
    "cpp_flight_adapter_3d": "Flight adapter (PX4 setpoints)",
    "bag_record": "Rosbag recorder",
    "vio_bridge_3d": "VIO → PX4 bridge",
    "foxglove": "Foxglove bridge",
    "cpp_ros_planner_pose": "Planner pose relay",
}
rows = []
for key, name in NAMES.items():
    cpu = np.array([p["processes"][key]["cpu_percent"] for _, p in d["procs"]])
    mem = np.array([p["processes"][key]["mem_mb"] for _, p in d["procs"]])
    rows.append(dict(key=key, name=name, cpu_mean=float(cpu[m].mean()), cpu_max=float(cpu[m].max()),
                     mem_mean=float(mem[m].mean()), mem_max=float(mem[m].max()), cpu=cpu))
rows.sort(key=lambda r: -r["cpu_mean"])
machine = np.array([p["cpu_percent"] for _, p in d["procs"]])
per_core = np.array([p["per_core"] for _, p in d["procs"]])
mem_pct = np.array([p["mem_percent"] for _, p in d["procs"]])
temp = np.array([p["cpu_temp_c"] for _, p in d["procs"]])
thr = np.array([p["throttled"] for _, p in d["procs"]])
load1 = np.array([p["load1"] for _, p in d["procs"]])
tracked_cpu = sum(r["cpu"] for r in rows)

stamps = d["stamps"]
def rate(topic):
    s = stamps[topic]; s = s[(s >= a0) & (s <= a1)]
    return dict(hz=float(len(s) / (a1 - a0)), max_gap_s=float(np.diff(s).max()))

metrics = dict(
    bag=Path(d["bag"]).name, armed_s=float(a1 - a0), samples=int(m.sum()),
    machine_cpu=dict(mean=float(machine[m].mean()), p95=float(np.percentile(machine[m], 95)), max=float(machine[m].max())),
    cores_used_mean=float(machine[m].mean() * 4 / 100),
    per_core_mean=per_core[m].mean(0).tolist(), per_core_max=per_core[m].max(0).tolist(),
    busiest_core_mean=float(per_core[m].max(1).mean()),
    tracked_cpu_mean_pct_core=float(tracked_cpu[m].mean()),
    load1_mean=float(load1[m].mean()),
    mem=dict(mean_pct=float(mem_pct[m].mean()), max_pct=float(mem_pct[m].max()),
             mean_mb=float(mem_pct[m].mean() * RAM_MB / 100), max_mb=float(mem_pct[m].max() * RAM_MB / 100),
             tracked_rss_mb=float(sum(r["mem_mean"] for r in rows))),
    temp_c=dict(mean=float(temp[m].mean()), max=float(temp[m].max()), start=float(temp[m][0]), end=float(temp[m][-1])),
    throttled_any=bool(np.any(thr[m] != 0)),
    rates=dict(vio=rate("/rtabmap/vio_odometry"), ev_to_px4=rate("/fmu/in/vehicle_visual_odometry"),
               camera=rate("/oak/left/image_rect/compressed"), setpoints=rate("/fmu/in/trajectory_setpoint"),
               octomap=rate("/rtabmap/octomap"), planner_path=rate("/planner3d/reference_path")),
    processes=[{k: v for k, v in r.items() if k != "cpu"} for r in rows],
)
json.dump(metrics, open(HERE / "compute_metrics.json", "w"), indent=1)

# ---- figure
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.3})
t = tp - a0
fig, ax = plt.subplots(3, 1, figsize=(12.5, 10), sharex=True,
                       gridspec_kw=dict(height_ratios=[1.4, 1, 0.8]))
stack = [r["cpu"] / 4 for r in rows if r["cpu_mean"] > 0.5]       # % of machine
labels = [r["name"] for r in rows if r["cpu_mean"] > 0.5]
colors = plt.cm.tab10(np.arange(len(stack)))
ax[0].stackplot(t, *stack, labels=labels, colors=colors, alpha=0.85, lw=0)
ax[0].plot(t, machine, "k-", lw=1.4, label="Whole machine (incl. untracked)")
ax[0].axhline(100, color="0.4", ls=":", lw=1)
ax[0].set_ylim(0, 105); ax[0].set_ylabel("CPU [% of 4 cores]")
ax[0].set_title("(a) CPU load by process")
ax[0].legend(fontsize=8, loc="upper left", bbox_to_anchor=(1.01, 1.0), framealpha=0.9)
for k in range(4):
    ax[1].plot(t, per_core[:, k], lw=1.1, label=f"core {k}")
ax[1].axhline(100, color="0.4", ls=":", lw=1)
ax[1].set_ylim(0, 105); ax[1].set_ylabel("core load [%]"); ax[1].set_title("(b) Per-core load")
ax[1].legend(fontsize=8, loc="upper left", bbox_to_anchor=(1.01, 1.0))
ax[2].plot(t, temp, "C3", lw=1.4, label="CPU temperature")
ax[2].set_ylabel("temperature [°C]", color="C3")
ax2 = ax[2].twinx(); ax2.grid(False)
ax2.plot(t, mem_pct * RAM_MB / 100 / 1024, "C0", lw=1.4, label="RAM used")
ax2.set_ylabel("RAM used [GB of 7.8]", color="C0"); ax2.set_ylim(0, 7.9)
ax[2].set_title("(c) CPU temperature and memory"); ax[2].set_xlabel("time since arming [s]")
for a in ax:
    # not armed: shaded underneath the data
    a.axvspan(t.min(), 0, color="0.92", lw=0, zorder=0); a.axvspan(a1 - a0, t.max(), color="0.92", lw=0, zorder=0)
fig.suptitle("Companion computer load during the 3D planner flight (2026-10-09 05:09Z, Raspberry Pi 5)", fontsize=12)
fig.tight_layout(); fig.savefig(HERE / "fig_compute.png", dpi=150)
print(json.dumps({k: v for k, v in metrics.items() if k != "processes"}, indent=1))
for r in metrics["processes"]:
    print(f"{r['name']:48s} cpu mean {r['cpu_mean']:6.1f}% max {r['cpu_max']:6.1f}%  mem {r['mem_mean']:6.1f} MB")
