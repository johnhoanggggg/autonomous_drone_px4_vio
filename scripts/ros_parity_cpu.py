#!/usr/bin/env python3
"""Measure selected live processes and ROS output rates; 100% CPU is one core."""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import statistics
import subprocess
import time

from depthai_rtabmap_bag import pose_is_valid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--process", action="append", required=True, metavar="LABEL=PID")
    parser.add_argument("--topic", action="append", default=[])
    parser.add_argument("--native-pose-topic", action="append", default=["/rtabmap/vio_pose"],
                        help="Count the verified SDK 3.5 null-pose sentinel as invalid on this topic")
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--label", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists() or args.seconds < 2:
        parser.error("use a new output path and a duration of at least two seconds")
    import psutil
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message

    processes = {label: psutil.Process(int(pid)) for label, pid in
                 (value.rsplit("=", 1) for value in args.process)}
    metadata = {label: dict(pid=p.pid, command=p.cmdline(), created=p.create_time())
                for label, p in processes.items()}
    rclpy.init()
    node = rclpy.create_node("parity_cpu_measurement")
    counts, lost = Counter(), Counter()
    arrivals = defaultdict(list)
    sensor_to_receive = defaultdict(list)
    positions = defaultdict(list)
    last_data = {}
    invalid_poses = Counter()
    status_topics = set()
    subscriptions = []
    deadline = time.monotonic() + 15
    pending = set(args.topic)
    while pending and time.monotonic() < deadline:
        topics = dict(node.get_topic_names_and_types())
        for topic in list(pending):
            if topic not in topics:
                continue
            def receive(msg, topic=topic):
                counts[topic] += 1
                arrivals[topic].append(time.monotonic())
                header = getattr(msg, "header", None)
                stamp = getattr(header, "stamp", None)
                if stamp is not None:
                    stamp_value = stamp.sec * 1_000_000_000 + stamp.nanosec
                    age = (node.get_clock().now().nanoseconds - stamp_value) / 1e9
                    if stamp_value > 0 and 0 <= age < 60:
                        sensor_to_receive[topic].append(age)
                if hasattr(msg, "lost"):
                    status_topics.add(topic)
                    if msg.lost:
                        lost[topic] += 1
                if isinstance(getattr(msg, "data", None), str):
                    try:
                        last_data[topic] = json.loads(msg.data)
                    except json.JSONDecodeError:
                        last_data[topic] = msg.data
                pose = getattr(msg, "pose", None)
                if pose is not None:
                    pose = getattr(pose, "pose", pose)
                    if hasattr(pose, "position"):
                        p = pose.position
                        q = pose.orientation
                        if topic in args.native_pose_topic and not pose_is_valid(
                                (p.x,p.y,p.z), (q.x,q.y,q.z,q.w)):
                            invalid_poses[topic] += 1
                        else:
                            positions[topic].append((p.x, p.y, p.z))
            subscriptions.append(node.create_subscription(
                get_message(topics[topic][0]), topic, receive, qos_profile_sensor_data))
            pending.remove(topic)
        rclpy.spin_once(node, timeout_sec=0.1)
    if pending:
        raise RuntimeError(f"Missing requested topics: {sorted(pending)}")

    def cpu_time(process):
        value = process.cpu_times()
        return value.user + value.system

    def hardware():
        try:
            temp = float(Path('/sys/class/thermal/thermal_zone0/temp').read_text()) / 1000
        except OSError:
            temp = None
        try:
            throttle = subprocess.check_output(['vcgencmd', 'get_throttled'], text=True, timeout=2).strip()
        except (OSError, subprocess.SubprocessError):
            throttle = None
        return dict(temperature_c=temp, throttled=throttle)

    counts.clear(); lost.clear(); arrivals.clear(); sensor_to_receive.clear()
    positions.clear(); invalid_poses.clear(); last_data.clear()
    before = {label: cpu_time(p) for label, p in processes.items()}
    thread_start = {label: {t.id: t.user_time+t.system_time for t in p.threads()}
                    for label, p in processes.items()}
    initial_hardware = hardware()
    psutil.cpu_percent()
    samples = []
    start = previous = time.monotonic()
    while time.monotonic()-start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if now-previous >= 1:
            totals = {label: cpu_time(p) for label, p in processes.items()}
            cpu = {label: 100*(totals[label]-before[label])/(now-previous) for label in processes}
            samples.append(dict(t_s=now-start, interval_s=now-previous, cpu_percent=cpu,
                                rss_mb={label: p.memory_info().rss/1e6 for label,p in processes.items()},
                                machine_cpu_percent=psutil.cpu_percent()))
            before, previous = totals, now
    duration = time.monotonic()-start
    threads = {}
    for label, process in processes.items():
        threads[label] = []
        for thread in process.threads():
            try:
                name = Path(f'/proc/{process.pid}/task/{thread.id}/comm').read_text().strip()
            except OSError:
                name = "exited"
            used = thread.user_time+thread.system_time-thread_start[label].get(thread.id, 0)
            threads[label].append(dict(tid=thread.id, name=name, cpu_percent=100*used/duration))
        threads[label].sort(key=lambda item: -item['cpu_percent'])
    measured = sum(s['interval_s'] for s in samples)
    report = dict(label=args.label, duration_s=duration, cpu_percent_convention="100% = one core",
                  cpu_count=psutil.cpu_count(), processes=metadata, hardware_start=initial_hardware,
                  hardware_end=hardware(), samples=samples, threads=threads,
                  mean_cpu_percent={label: sum(s['cpu_percent'][label]*s['interval_s'] for s in samples)/measured
                                    for label in processes},
                  mean_rss_mb={label: statistics.mean(s['rss_mb'][label] for s in samples) for label in processes},
                  topics={topic: dict(count=counts[topic], rate_hz=counts[topic]/duration,
                                     lost=lost[topic] if topic in status_topics else None,
                                     invalid_native_poses=invalid_poses[topic] if topic in args.native_pose_topic else None,
                                     sensor_to_receive_median_s=(statistics.median(sensor_to_receive[topic])
                                                                 if sensor_to_receive[topic] else None),
                                     sensor_to_receive_p90_s=(statistics.quantiles(
                                         sensor_to_receive[topic], n=10, method="inclusive")[8]
                                         if len(sensor_to_receive[topic]) >= 2 else None),
                                     sensor_to_receive_max_s=max(sensor_to_receive[topic], default=None),
                                     longest_receive_gap_s=max((b-a for a,b in zip(arrivals[topic],arrivals[topic][1:])), default=None))
                          for topic in args.topic})
    for topic, points in positions.items():
        center = [statistics.mean(p[i] for p in points) for i in range(3)]
        report['topics'][topic]['pose_rms_m'] = math.sqrt(statistics.mean(math.dist(p, center)**2 for p in points))
        report['topics'][topic]['net_displacement_m'] = math.dist(points[0], points[-1])
    for topic, data in last_data.items():
        report['topics'][topic]['last_data'] = data
    args.out.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({key:report[key] for key in ['label','mean_cpu_percent','topics','hardware_end']}, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
