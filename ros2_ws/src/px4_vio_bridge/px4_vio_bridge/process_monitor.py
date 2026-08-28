"""Publish per-process CPU and memory load so flight performance is probeable.

The flight stack is several processes sharing four Pi cores, and until now the
only record of how hard they were working was absent from the bag entirely. A
late setpoint, a VIO frame drop, or a planner replan that misses its deadline
all look like control problems in the log when they may be scheduling problems.
This node puts the load next to the flight data on the same time base, so a bag
can answer "was the Pi saturated when that happened?" after the fact.

Written after the 2026-08-28 flight, where VIO arrived at 13.8 Hz against a
nominal 10-15 Hz and there was no way to tell from the bag whether the camera,
RTAB-Map, or CPU contention was responsible.

Per label it publishes:
    /perf/process/<label>/cpu_percent   Float32, summed over matching processes
    /perf/process/<label>/mem_mb        Float32, RSS
plus machine-wide `/perf/cpu_percent`, `/perf/mem_percent`, `/perf/cpu_temp_c`,
`/perf/load1`, `/perf/throttled` and a `/perf/processes` JSON table carrying
everything at once (pids, thread counts, per-label totals) for a Foxglove Raw
Message or Table panel.

CPU percent follows the psutil convention: 100% is one fully-used core, so a
threaded process such as RTAB-Map legitimately exceeds 100 on this 4-core Pi.
`/perf/cpu_percent` is machine-wide and does saturate at 100.

`/perf/throttled` is the Raspberry Pi throttle word from `vcgencmd`, which is
the signal that separates "our code is too slow" from "the board browned out or
overheated and the clock was cut". Bit 0 = under-voltage now, bit 1 = ARM
frequency capped now, bit 2 = throttled now, bit 3 = soft temperature limit now;
bits 16-19 are the same four latched since boot. -1 means unavailable.
"""
import json
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int32, String

try:
    import psutil
except ImportError:  # pragma: no cover - exercised only on a host without psutil
    psutil = None

# label -> substrings that must all appear in the process command line. Chosen
# to match the flight stack in rtabmap_slam_px4.launch.py and
# offboard_global_planner.launch.py without also matching this monitor.
DEFAULT_TARGETS = {
    "slam": "basalt_rtabmap_slam_ros2",
    "xrce_agent": "MicroXRCEAgent",
    "vio_bridge": "vio_to_px4_odometry",
    "planner": "offboard_global_planner",
    # The A* planner and the follower are the monitor-launch nodes, not the
    # grid_planner/path_follower modules they import — matching on the module
    # names finds no process at all.
    "astar": "global_planner_monitor",
    "follower": "route_follower_monitor",
    "planner_sim": "global_planner_sim",
    "bag_record": "ros2 bag record",
    "foxglove": "foxglove_bridge",
    "battery": "battery_to_ros",
    "px4_pos": "px4_local_position_to_ros",
}

THERMAL_PATH = "/sys/class/thermal/thermal_zone0/temp"


def parse_throttled(output):
    """Parse `vcgencmd get_throttled` ("throttled=0x50005") into an int.

    Returns -1 when the output is missing or unparseable, which is the same
    thing the topic reports on a non-Pi host.
    """
    if not output:
        return -1
    text = output.strip()
    _, _, value = text.partition("=")
    try:
        return int(value, 0)
    except ValueError:
        return -1


def match_label(cmdline, pattern):
    """True when every whitespace-separated token of pattern is in cmdline.

    Tokens rather than a plain substring so "ros2 bag record" matches the real
    command line, where the words are separated by argv boundaries and may pick
    up intervening arguments.
    """
    return all(token in cmdline for token in pattern.split())


class ProcessMonitor(Node):
    def __init__(self):
        super().__init__("process_monitor")

        self.declare_parameter("publish_period", 1.0)
        # Enumerating every process is itself work; do it far less often than
        # we publish. Between scans the cached handles are reused, which is
        # also what makes psutil's per-object cpu_percent() meaningful.
        self.declare_parameter("scan_period", 5.0)
        self.declare_parameter("labels", sorted(DEFAULT_TARGETS))
        self.declare_parameter("patterns", [DEFAULT_TARGETS[k] for k in sorted(DEFAULT_TARGETS)])
        self.declare_parameter("include_self", False)
        self.declare_parameter("vcgencmd", True)

        self.publish_period = float(self.get_parameter("publish_period").value)
        self.scan_period = float(self.get_parameter("scan_period").value)
        labels = [str(x) for x in self.get_parameter("labels").value]
        patterns = [str(x) for x in self.get_parameter("patterns").value]
        if len(labels) != len(patterns):
            raise ValueError("labels and patterns must have the same length")
        self.targets = dict(zip(labels, patterns))
        self.include_self = bool(self.get_parameter("include_self").value)
        self.use_vcgencmd = bool(self.get_parameter("vcgencmd").value)

        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cpu_pubs = {
            label: self.create_publisher(
                Float32, f"/perf/process/{label}/cpu_percent", out_qos
            )
            for label in self.targets
        }
        self.mem_pubs = {
            label: self.create_publisher(
                Float32, f"/perf/process/{label}/mem_mb", out_qos
            )
            for label in self.targets
        }
        self.cpu_pub = self.create_publisher(Float32, "/perf/cpu_percent", out_qos)
        self.mem_pub = self.create_publisher(Float32, "/perf/mem_percent", out_qos)
        self.temp_pub = self.create_publisher(Float32, "/perf/cpu_temp_c", out_qos)
        self.load_pub = self.create_publisher(Float32, "/perf/load1", out_qos)
        self.throttled_pub = self.create_publisher(Int32, "/perf/throttled", out_qos)
        self.table_pub = self.create_publisher(String, "/perf/processes", out_qos)

        self.tracked = {}          # label -> {pid: psutil.Process}
        self.last_scan = 0.0
        self.throttled = -1
        self.last_throttle_poll = 0.0
        self.cores = os.cpu_count() or 1

        if psutil is None:
            self.get_logger().error(
                "psutil is not installed; /perf topics will publish nothing. "
                "Install with: sudo apt install python3-psutil"
            )
        else:
            psutil.cpu_percent(None)  # prime the machine-wide counter
            self.scan()

        self.create_timer(self.publish_period, self.tick)
        self.get_logger().info(
            f"process_monitor: {len(self.targets)} labels "
            f"({', '.join(sorted(self.targets))}) on {self.cores} cores, "
            f"publish {self.publish_period:.1f}s / scan {self.scan_period:.1f}s"
        )

    def scan(self):
        """Refresh the pid set for every label, keeping live handles alive.

        Handles must survive across scans: psutil computes a process's CPU
        percent against the previous call *on that object*, so replacing it
        every scan would reset the measurement to 0.
        """
        if psutil is None:
            return
        self.last_scan = time.monotonic()
        own = os.getpid()
        found = {label: {} for label in self.targets}
        for proc in psutil.process_iter(["pid", "cmdline"]):
            try:
                pid = proc.info["pid"]
                if pid == own and not self.include_self:
                    continue
                cmdline = " ".join(proc.info["cmdline"] or ())
                if not cmdline:
                    continue
                for label, pattern in self.targets.items():
                    if not match_label(cmdline, pattern):
                        continue
                    existing = self.tracked.get(label, {}).get(pid)
                    if existing is not None:
                        found[label][pid] = existing
                    else:
                        proc.cpu_percent(None)  # prime; first read is always 0
                        found[label][pid] = proc
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        self.tracked = found

    def sample(self):
        """Per-label totals. Dead processes are dropped as they are noticed."""
        out = {}
        for label, procs in self.tracked.items():
            cpu = 0.0
            mem = 0.0
            threads = 0
            pids = []
            for pid, proc in list(procs.items()):
                try:
                    cpu += proc.cpu_percent(None)
                    mem += proc.memory_info().rss / (1024.0 * 1024.0)
                    threads += proc.num_threads()
                    pids.append(pid)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    procs.pop(pid, None)
            out[label] = {
                "cpu_percent": round(cpu, 1),
                "cpu_percent_of_machine": round(cpu / self.cores, 1),
                "mem_mb": round(mem, 1),
                "threads": threads,
                "count": len(pids),
                "pids": sorted(pids),
                "running": bool(pids),
            }
        return out

    def cpu_temp(self):
        try:
            with open(THERMAL_PATH) as handle:
                return int(handle.read().strip()) / 1000.0
        except (OSError, ValueError):
            return float("nan")

    def poll_throttled(self):
        """Read the Pi throttle word, at its own slow cadence.

        This forks a process, so it deliberately runs far less often than the
        publish timer; the value is latched between polls.
        """
        if not self.use_vcgencmd:
            return -1
        now = time.monotonic()
        if now - self.last_throttle_poll < max(self.publish_period, 5.0):
            return self.throttled
        self.last_throttle_poll = now
        try:
            result = subprocess.run(
                ["vcgencmd", "get_throttled"],
                capture_output=True, text=True, timeout=2.0, check=False,
            )
            self.throttled = parse_throttled(result.stdout)
        except (OSError, subprocess.SubprocessError):
            self.throttled = -1
        return self.throttled

    def tick(self):
        if psutil is None:
            return
        if time.monotonic() - self.last_scan >= self.scan_period:
            self.scan()

        per_process = self.sample()
        for label, stats in per_process.items():
            self.cpu_pubs[label].publish(Float32(data=float(stats["cpu_percent"])))
            self.mem_pubs[label].publish(Float32(data=float(stats["mem_mb"])))

        cpu = psutil.cpu_percent(None)
        mem = psutil.virtual_memory().percent
        temp = self.cpu_temp()
        load1 = os.getloadavg()[0]
        throttled = self.poll_throttled()

        self.cpu_pub.publish(Float32(data=float(cpu)))
        self.mem_pub.publish(Float32(data=float(mem)))
        self.temp_pub.publish(Float32(data=float(temp)))
        self.load_pub.publish(Float32(data=float(load1)))
        self.throttled_pub.publish(Int32(data=int(throttled)))
        self.table_pub.publish(String(data=json.dumps({
            "cpu_percent": round(cpu, 1),
            "mem_percent": round(mem, 1),
            "cpu_temp_c": None if temp != temp else round(temp, 1),
            "load1": round(load1, 2),
            "cores": self.cores,
            "throttled": throttled,
            "per_core": [round(x, 1) for x in psutil.cpu_percent(None, percpu=True)],
            "processes": per_process,
        }, sort_keys=True)))

        missing = [label for label, s in per_process.items() if not s["running"]]
        if missing:
            self.get_logger().debug(
                f"not running: {', '.join(sorted(missing))}", throttle_duration_sec=30.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = ProcessMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
