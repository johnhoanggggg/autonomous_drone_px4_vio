"""Real ROS graph with synthetic keyframes/odometry; no camera or PX4.

Runs in a separate DDS domain, exercises the installed production executables,
and always terminates its own process groups. No replayed /fmu input exists.
"""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import MapData
from std_msgs.msg import String, Bool
from octomap_msgs.msg import Octomap


def test_ros_octomap_navigation_faults(tmp_path):
    domain = 170 + os.getpid() % 40
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node("navigation_integration_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    binary_dir = Path(get_package_prefix("px4_vio_bridge")) / "lib/px4_vio_bridge"
    processes, logs, subscriptions = [], [], []
    received = {}

    def start(executable, params=None):
        log = (tmp_path / f"{executable}.log").open("w")
        logs.append(log)
        args = [str(binary_dir / executable), "--ros-args"]
        for key, value in (params or {}).items():
            args += ["-p", f"{key}:={str(value).lower() if isinstance(value, bool) else value}"]
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                   env={**os.environ, "ROS_DOMAIN_ID": str(domain)},
                                   start_new_session=True)
        processes.append(process)
        return process

    def subscribe(kind, topic, qos=10):
        received[topic] = []
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg: received[topic].append((time.monotonic(), msg)), qos))

    def latest(topic):
        return received[topic][-1][1] if received[topic] else None

    def value(topic):
        msg = latest(topic)
        return json.loads(msg.data) if msg else {}

    def wait_for(predicate, seconds=10, pump=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if pump:
                pump()
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        states = {topic: value(topic) for topic in (
            "/planner3d/status", "/planner3d/follower/status", "/planner3d/pose_status")}
        raise AssertionError(f"navigation condition timed out: {states}; logs: {tmp_path}")

    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscribe(MapData, "/rtabmap/mapData", durable)
    for topic in ("/planner3d/pose_snapshot", "/planner3d/path_snapshot", "/planner3d/status",
                  "/planner3d/follower/status", "/planner3d/pose_status"):
        subscribe(String, topic)
    subscribe(String, "/rtabmap/octomap_metadata", durable)
    subscribe(Octomap, "/rtabmap/octomap", durable)
    subscribe(Bool, "/planner3d/follower/valid")
    subscribe(Bool, "/planner3d/follower/goal_reached")
    subscribe(Vector3Stamped, "/planner3d/follower/velocity")
    subscribe(PoseStamped, "/rtabmap/vio_pose")
    subscribe(PoseStamped, "/rtabmap/pose")
    odom_pub = node.create_publisher(Odometry, "/rtabmap/vio_odometry", qos_profile_sensor_data)
    map_pub = node.create_publisher(MapData, "/rtabmap/mapData", durable)
    goal_pub = node.create_publisher(PointStamped, "/waypoint/clicked", 10)
    path_pub = node.create_publisher(String, "/planner3d/path_snapshot", 10)
    try:
        # Harvest the production deterministic keyframe fixture, then take over
        # its inputs so each fault controls sensor time and map generation.
        fixture = start("rtabmap_3d_fixture", {"loop_correction_after": 10000.0})
        wait_for(lambda: latest("/rtabmap/mapData") is not None)
        template = copy.deepcopy(latest("/rtabmap/mapData"))
        os.killpg(fixture.pid, signal.SIGINT)
        fixture.wait(timeout=10)
        record_log = (tmp_path / "record.log").open("w")
        logs.append(record_log)
        recorder = subprocess.Popen(
            ["ros2", "bag", "record", "--storage", "mcap", "--storage-preset-profile", "fastwrite",
             "--disable-keyboard-controls", "--output", str(tmp_path / "bag"), "--topics",
             "/rtabmap/octomap", "/rtabmap/octomap_metadata", "/rtabmap/mapData",
             "/rtabmap/vio_odometry", "/planner3d/pose_snapshot", "/planner3d/path_snapshot",
             "/planner3d/follower/status"], stdout=record_log, stderr=subprocess.STDOUT,
            env={**os.environ, "ROS_DOMAIN_ID": str(domain)}, start_new_session=True)
        processes.append(recorder)
        start("rtabmap_octomap_node")
        start("cpp_ros_planner_pose")
        common = {"pose_snapshot_topic": "/planner3d/pose_snapshot", "min_z": 0.2,
                  "max_z": 2.0, "planning_radius_xy": 3.0}
        planner_process = start("cpp_astar_planner_3d", {**common, "voxel_size": 0.1, "planning_timeout_ms": 1000.0})
        start("cpp_route_follower_3d", {**common, "vio_frame_id": "rtabmap_odom"})
        last_odom = last_map = last_goal = 0.0
        mode, shift, goal_z = "normal", 0.0, 1.1
        map_enabled = True

        def pump():
            nonlocal last_odom, last_map, last_goal
            clock = time.monotonic()
            if clock - last_map >= 0.8 and map_enabled:
                msg = copy.deepcopy(template)
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.graph.map_to_odom.translation.y = shift
                msg.graph.map_to_odom.rotation.w = 1.0
                for pose in msg.graph.poses:
                    pose.position.y += shift
                map_pub.publish(msg)
                last_map = clock
            if clock - last_goal >= 1.0:
                goal = PointStamped()
                goal.header.frame_id = "world"
                goal.header.stamp = node.get_clock().now().to_msg()
                goal.point.x, goal.point.y, goal.point.z = 1.0, shift, goal_z
                goal_pub.publish(goal)
                last_goal = clock
            if clock - last_odom >= 0.05:
                odom = Odometry()
                stamp = node.get_clock().now().nanoseconds
                if mode == "stale":
                    stamp -= 2_000_000_000
                elif mode == "future":
                    stamp += 2_000_000_000
                odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(stamp, 10**9)
                odom.header.frame_id = "wrong" if mode == "frame" else "rtabmap_odom"
                odom.child_frame_id = "oak"
                odom.pose.pose.position.x = -1.0
                odom.pose.pose.position.z = 0.3 if mode == "low_start" else 0.8
                odom.pose.pose.orientation.w = 1.0
                if mode == "nan":
                    odom.pose.pose.position.z = math.nan
                if mode == "lost":
                    odom.pose.covariance[0] = 9999.0
                odom_pub.publish(odom)
                last_odom = clock

        def following():
            msg = latest("/planner3d/follower/valid")
            return msg is not None and msg.data

        wait_for(following, seconds=20, pump=pump)
        snapshot = value("/planner3d/pose_snapshot")
        assert snapshot["raw_pose"][:3] == pytest.approx([-1.1, -0.036, 0.856])
        path = value("/planner3d/path_snapshot")
        assert path["points"][-1][2] > 1.0, "clicked height must survive planning"
        assert latest("/rtabmap/vio_pose").header.frame_id == "rtabmap_odom"

        # A free center cannot fit the robot envelope through the floor bound.
        # Report that configuration problem instead of generic START_BLOCKED.
        mode = "low_start"
        wait_for(lambda: value("/planner3d/status").get("state") ==
                 "START_ENVELOPE_OUTSIDE_GEOFENCE", pump=pump)
        boundary = value("/planner3d/status")
        assert boundary["start"][2] == pytest.approx(0.356)
        assert boundary["geofence"]["min_center_z"] == pytest.approx(0.55)
        wait_for(lambda: value("/planner3d/path_snapshot")["points"] == [], pump=pump)
        mode = "normal"
        wait_for(following, pump=pump)

        # A real graph correction shifts map and corrected body pose together;
        # raw odometry remains continuous. Follower resumes on the matched map.
        shift = 0.2
        wait_for(lambda: value("/planner3d/pose_snapshot").get("correction", [0, 0])[1] == shift,
                 pump=pump)
        wait_for(following, pump=pump)
        assert abs(value("/planner3d/pose_snapshot")["pose"][1] - 0.164) < 1e-6
        assert latest("/rtabmap/vio_pose").pose.position.y == 0.0

        for fault in ("stale", "future", "frame", "nan", "lost"):
            wait_for(lambda: following() and latest("/planner3d/follower/velocity") is not None and
                     math.hypot(latest("/planner3d/follower/velocity").vector.x,
                                latest("/planner3d/follower/velocity").vector.y) >= 0.06, pump=pump)
            map_enabled = False
            mode = fault
            # Consume already-in-flight data before asserting no pose refresh.
            end = time.monotonic() + 0.3
            while time.monotonic() < end:
                pump(); executor.spin_once(timeout_sec=0.02)
            count = len(received["/planner3d/pose_snapshot"])
            wait_for(lambda: value("/planner3d/follower/status").get("state") in
                     ("VIO_STALE", "POSE_STALE"), seconds=4, pump=pump)
            wait_for(lambda: not following() and latest("/planner3d/follower/velocity") is not None and
                     all(v == 0.0 for v in (latest("/planner3d/follower/velocity").vector.x,
                         latest("/planner3d/follower/velocity").vector.y,
                         latest("/planner3d/follower/velocity").vector.z)), seconds=2, pump=pump)
            assert len(received["/planner3d/pose_snapshot"]) == count, fault
            velocity = latest("/planner3d/follower/velocity").vector
            assert (velocity.x, velocity.y, velocity.z) == (0.0, 0.0, 0.0)
            recovered_at = time.monotonic()
            mode = "normal"
            wait_for(following, pump=pump)
            # DDS does not promise cross-topic callback order: the valid flag
            # can arrive before its matching velocity sample at this observer.
            wait_for(lambda: any(t >= recovered_at and math.hypot(msg.vector.x, msg.vector.y) > 1e-8
                                 for t, msg in received["/planner3d/follower/velocity"]), pump=pump)
            resumed = [msg.vector for t, msg in received["/planner3d/follower/velocity"]
                       if t >= recovered_at and math.hypot(msg.vector.x, msg.vector.y) > 1e-8]
            assert resumed and math.hypot(resumed[0].x, resumed[0].y) <= 0.04
            assert abs(resumed[0].z) <= 0.025
            map_enabled = True

        # Unknown/outside altitude must withdraw the route immediately and a
        # subsequent valid goal must recover without restarting the processes.
        goal_z = 3.0
        wait_for(lambda: value("/planner3d/status").get("state") == "GOAL_OUTSIDE_GEOFENCE",
                 pump=pump)
        wait_for(lambda: not following(), pump=pump)
        assert value("/planner3d/path_snapshot")["points"] == []
        goal_z = 1.1
        wait_for(following, pump=pump)

        map_enabled = False
        wait_for(lambda: value("/planner3d/follower/status").get("state") == "MAP_STALE",
                 seconds=6, pump=pump)
        wait_for(lambda: not following(), seconds=1, pump=pump)
        map_enabled = True
        wait_for(following, pump=pump)
        topics = [name for name, _ in node.get_topic_names_and_types()]
        assert not any(name.startswith("/fmu/in/") for name in topics)
        report = {"passed": True, "faults": ["stale", "future", "frame", "nan", "lost",
                  "goal_outside_geofence", "map_stale"], "loop_correction_m": shift,
                  "pose_snapshots": len(received["/planner3d/pose_snapshot"]),
                  "path_snapshots": len(received["/planner3d/path_snapshot"])}
        os.killpg(recorder.pid, signal.SIGINT)
        recorder.wait(timeout=15)
        # Rejected snapshots must not keep an earlier route active. These
        # deliberately invalid publisher messages are injected after recording.
        for path_fault in ("generation", "frame", "future"):
            os.killpg(planner_process.pid, signal.SIGSTOP)
            try:
                map_enabled = False
                invalid_path = value("/planner3d/path_snapshot")
                invalid_path["stamp_ns"] = node.get_clock().now().nanoseconds
                if path_fault == "generation":
                    invalid_path["map_generation"] = -1
                elif path_fault == "frame":
                    invalid_path["frame_id"] = "wrong"
                else:
                    invalid_path["stamp_ns"] += 2_000_000_000
                path_pub.publish(String(data=json.dumps(invalid_path)))
                wait_for(lambda: value("/planner3d/follower/status").get("state") == "PATH_STALE",
                         seconds=2, pump=pump)
                wait_for(lambda: not following(), seconds=1, pump=pump)
            finally:
                os.killpg(planner_process.pid, signal.SIGCONT)
                map_enabled = True
            wait_for(following, pump=pump)
        report["path_faults"] = ["generation", "frame", "future"]

        # Identical route geometry may be a partial endpoint or the actual
        # waypoint. Its atomic goal metadata must update without a path reset.
        map_enabled = False
        drain_until = time.monotonic() + 0.2
        wait_for(lambda: time.monotonic() >= drain_until, pump=pump)
        wait_for(following, pump=pump)
        os.killpg(planner_process.pid, signal.SIGSTOP)
        try:
            endpoint_path = value("/planner3d/path_snapshot")
            body = value("/planner3d/pose_snapshot")["pose"][:3]
            endpoint_path["points"] = [body, [body[0] + 0.01, body[1], body[2]]]
            for exact, terminal, state in (
                    (False, False, "EXPLORATION_ENDPOINT_REACHED"),
                    (True, True, "GOAL_REACHED"),
                    (False, True, "SAFE_APPROACH_REACHED")):
                endpoint_path["stamp_ns"] = node.get_clock().now().nanoseconds
                endpoint_path["goal_exact"], endpoint_path["goal_terminal"] = exact, terminal
                path_pub.publish(String(data=json.dumps(endpoint_path)))
                wait_for(lambda: value("/planner3d/follower/status").get("state") == state,
                         seconds=2, pump=pump)
                wait_for(lambda: latest("/planner3d/follower/goal_reached").data == exact,
                         seconds=1, pump=pump)
                assert value("/planner3d/follower/status")["requested_goal_reached"] == exact
        finally:
            os.killpg(planner_process.pid, signal.SIGCONT)
            map_enabled = True
        wait_for(following, pump=pump)


        audit_script = Path(__file__).resolve().parents[4] / "scripts/audit_planner_3d_bag.py"
        audit_result = subprocess.run(
            ["python3", str(audit_script), str(tmp_path / "bag"), "--out", str(tmp_path / "audit.json")],
            capture_output=True, text=True, timeout=30)
        assert audit_result.returncode == 0, audit_result.stdout + audit_result.stderr
        # Verify that the auditor itself rejects an occupied crossing, unknown
        # space, and a generation tag with no corresponding map.
        tree = latest("/rtabmap/octomap")
        meta = value("/rtabmap/octomap_metadata")
        map_record = {"kind": "map", "generation": meta["generation"], "stamp_ns": meta["stamp_ns"],
                      "id": tree.id, "binary": tree.binary, "resolution": tree.resolution,
                      "data": list(tree.data)}
        crossing = {"kind": "check", "generation": meta["generation"], "stamp_ns": meta["stamp_ns"],
                    "radius": 0.35, "points": [[-1.0, shift, 0.8], [1.0, shift, 0.8]]}
        unknown = {**crossing, "points": [[-3.0, shift, 0.8], [-2.5, shift, 0.8]]}
        mismatch = {**crossing, "generation": -1}
        bad_result = subprocess.run([str(binary_dir / "octomap_path_audit")],
            input="".join(json.dumps(item) + "\n" for item in [map_record, crossing, unknown, mismatch]),
            capture_output=True, text=True, timeout=10)
        outcomes = [json.loads(line) for line in bad_result.stdout.splitlines()]
        assert [item["valid"] for item in outcomes] == [True, False, False, False]
        report["audit"] = json.loads((tmp_path / "audit.json").read_text())
        (tmp_path / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        for log in logs:
            log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_start_clear_accepts_earlier_map_stamp_and_keeps_raw_anchor(tmp_path):
    domain = 170 + os.getpid() % 40
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node('start_clear_anchor_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    binary_dir = Path(get_package_prefix('px4_vio_bridge')) / 'lib/px4_vio_bridge'
    processes, logs = [], []
    maps, metadata = [], []
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(MapData, '/rtabmap/mapData', maps.append, durable),
        node.create_subscription(String, '/rtabmap/octomap_metadata',
                                 lambda msg: metadata.append(json.loads(msg.data)), durable),
    ]
    odom_pub = node.create_publisher(Odometry, '/rtabmap/vio_odometry', qos_profile_sensor_data)
    map_pub = node.create_publisher(MapData, '/rtabmap/mapData', durable)
    goal_pub = node.create_publisher(PointStamped, '/waypoint/clicked', 10)

    def start(executable, *params):
        log = (tmp_path / f'{executable}.log').open('w')
        logs.append(log)
        process = subprocess.Popen([str(binary_dir / executable), '--ros-args', *params],
            stdout=log, stderr=subprocess.STDOUT,
            env={**os.environ, 'ROS_DOMAIN_ID': str(domain)}, start_new_session=True)
        processes.append(process)
        return process

    def wait_for(predicate, pump=None):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if pump:
                pump()
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError(f'start clear condition timed out; logs: {tmp_path}')

    try:
        fixture = start('rtabmap_3d_fixture', '-p', 'loop_correction_after:=10000.0')
        wait_for(lambda: bool(maps))
        template = copy.deepcopy(maps[-1])
        os.killpg(fixture.pid, signal.SIGINT)
        fixture.wait(timeout=10)
        start('rtabmap_octomap_node', '-p', 'test_start_clear_radius:=0.5')
        wait_for(lambda: odom_pub.get_subscription_count() == 1)
        odom = Odometry()
        initial_stamp = node.get_clock().now().nanoseconds
        odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(initial_stamp, 10**9)
        odom.header.frame_id, odom.child_frame_id = 'rtabmap_odom', 'oak'
        odom.pose.pose.position.x, odom.pose.pose.position.z = -1.0, 0.8
        odom.pose.pose.orientation.w = 1.0
        deadline = time.monotonic() + 0.15
        wait_for(lambda: time.monotonic() >= deadline, lambda: odom_pub.publish(odom))
        first = copy.deepcopy(template)
        first_stamp = initial_stamp - 200_000
        first.header.stamp.sec, first.header.stamp.nanosec = divmod(first_stamp, 10**9)
        wait_for(lambda: bool(metadata), lambda: map_pub.publish(first))
        clear = metadata[-1]['test_start_clear']
        assert metadata[-1]['stamp_ns'] == first_stamp
        assert clear['anchor_stamp_ns'] == initial_stamp
        assert clear['center'] == pytest.approx([-1.1, -0.036, 0.856])
        assert clear['enabled'] and clear['radius'] == 0.5 and clear['cleared_voxels'] > 0
        assert clear['observed_free'] is False
        anchor = clear['raw_body_pose']
        odom.header.stamp = node.get_clock().now().to_msg()
        odom.pose.pose.position.x = -0.5
        odom.pose.pose.position.y = 0.2
        deadline = time.monotonic() + 0.15
        wait_for(lambda: time.monotonic() >= deadline, lambda: odom_pub.publish(odom))
        second = copy.deepcopy(template)
        second.header.stamp = node.get_clock().now().to_msg()
        second.graph.map_to_odom.translation.y = 0.2
        for pose in second.graph.poses:
            pose.position.y += 0.2
        wait_for(lambda: len(metadata) >= 2, lambda: map_pub.publish(second))
        corrected = metadata[-1]['test_start_clear']
        assert corrected['raw_body_pose'] == anchor
        assert corrected['anchor_stamp_ns'] == initial_stamp
        assert corrected['center'] == pytest.approx([-1.1, 0.164, 0.856])
        assert corrected['anchor_sequence'] == 1
        assert corrected['anchor_reason'] == 'startup'

        def publish_body():
            odom.header.stamp = node.get_clock().now().to_msg()
            odom_pub.publish(odom)

        def new_map():
            expected_count = len(metadata) + 1
            second.header.stamp = node.get_clock().now().to_msg()
            wait_for(lambda: len(metadata) >= expected_count, lambda: map_pub.publish(second))
            return metadata[-1]['test_start_clear']

        deadline = time.monotonic() + 0.1
        wait_for(lambda: time.monotonic() >= deadline, publish_body)
        goal = PointStamped()
        goal.header.frame_id = 'world'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.point.x, goal.point.z = 1.0, 0.8
        deadline = time.monotonic() + 0.1
        wait_for(lambda: time.monotonic() >= deadline, lambda: goal_pub.publish(goal))
        clicked = new_map()
        assert clicked['anchor_sequence'] == 2
        assert clicked['anchor_reason'] == 'waypoint'
        assert clicked['raw_body_pose'][:3] == pytest.approx([-0.6, 0.164, 0.856])
        assert clicked['center'] == pytest.approx([-0.6, 0.364, 0.856])

        # Body movement alone, or resending the same click, cannot clear a trail.
        odom.pose.pose.position.x = -0.2
        deadline = time.monotonic() + 0.1
        wait_for(lambda: time.monotonic() >= deadline, publish_body)
        goal_pub.publish(goal)
        unchanged = new_map()
        assert unchanged['anchor_sequence'] == 2
        assert unchanged['raw_body_pose'] == clicked['raw_body_pose']

        # Fresh clicks with bad frame/time or invalid tracking cannot move it.
        for fault in ('frame', 'future', 'lost'):
            rejected = copy.deepcopy(goal)
            rejected.header.stamp = node.get_clock().now().to_msg()
            if fault == 'frame':
                rejected.header.frame_id = 'wrong'
            elif fault == 'future':
                rejected.header.stamp.sec += 2
            else:
                odom.pose.covariance[0] = 9999.0
                deadline = time.monotonic() + 0.1
                wait_for(lambda: time.monotonic() >= deadline, publish_body)
            deadline = time.monotonic() + 0.1
            wait_for(lambda: time.monotonic() >= deadline, lambda: goal_pub.publish(rejected))
            assert new_map()['anchor_sequence'] == 2, fault

        odom.pose.covariance[0] = 0.0
        deadline = time.monotonic() + 0.1
        wait_for(lambda: time.monotonic() >= deadline, publish_body)
        goal.header.stamp = node.get_clock().now().to_msg()
        deadline = time.monotonic() + 0.1
        wait_for(lambda: time.monotonic() >= deadline, lambda: goal_pub.publish(goal))
        reclicked = new_map()
        assert reclicked['anchor_sequence'] == 3
        assert reclicked['center'] == pytest.approx([-0.3, 0.364, 0.856])

    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        for log in logs:
            log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
