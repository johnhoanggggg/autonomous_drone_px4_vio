#!/usr/bin/env python3
"""Replay saved OAK RGB-D through ROS-native 3D navigation, without camera/PX4.

By default only images, depth, calibration and static TF are played. --mapdata
instead replays MapData/raw odometry, optionally with recorded waypoint goals.
--hold-start-pose is a separately labelled synthetic stationary-start test,
not a replay of the recorded carry motion.
The recorded output includes exact OctoMaps and atomic navigation snapshots.
This is a graph/continuity diagnostic; a stationary bag may legitimately have
no known-free volume around the airframe and therefore no accepted route.
"""
import argparse
import copy
import os
import signal
import subprocess
import time
import json
import math
from pathlib import Path
from collections import Counter
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_prefix

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=178)
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--min-z', type=float, default=-1.5)
    parser.add_argument('--max-z', type=float, default=1.5)
    parser.add_argument('--mapdata', action='store_true', help='Replay recorded MapData and raw odometry instead of running the RGB-D estimator')
    parser.add_argument('--goal-height-mode', choices=['clicked', 'current'], default='clicked')
    parser.add_argument('--strict-route', action='store_true', help='Disable the exploration reference route')
    parser.add_argument('--max-marker-voxels', type=int, default=0)
    parser.add_argument('--disable-checkpoints', action='store_true', help='Use full OctoMap rebuilds as a timing baseline')
    parser.add_argument('--test-start-clear-radius', type=float, default=0.0)
    parser.add_argument('--goal-offset', type=float, default=0.75, help='Diagnostic goal offset in world X from first body snapshot')
    parser.add_argument('--hold-start-pose', action='store_true', help='Controlled startup diagnostic: publish the first recorded raw pose with fresh stamps instead of replaying carry motion')
    parser.add_argument('--recorded-goals', action='store_true', help='Use recorded waypoint clicks instead of a diagnostic goal')
    args = parser.parse_args()
    if not 0.0 <= args.test_start_clear_radius <= 0.5 or not math.isfinite(args.goal_offset):
        parser.error('require test clear radius in [0, 0.5] and finite goal offset')
    if args.hold_start_pose and not args.mapdata:
        parser.error('--hold-start-pose requires --mapdata')
    if args.recorded_goals and not args.mapdata:
        parser.error('--recorded-goals requires --mapdata')
    if args.domain == 42 or not 0 <= args.domain <= 232:
        parser.error('use an isolated ROS domain (0..232, excluding flight domain 42)')
    if not args.bag.exists() or args.out.exists() or (not 0 < args.timeout <= 3600) or (not all((math.isfinite(v) for v in (args.min_z, args.max_z)))) or (args.min_z >= args.max_z):
        parser.error('require an existing bag, new output directory, valid timeout and altitude band')
    out = args.out
    out.mkdir(exist_ok=False)
    env = {**os.environ, 'ROS_DOMAIN_ID': str(args.domain)}
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    rclpy.init()
    node = Node('planner3d_saved_replay_audit', parameter_overrides=[Parameter('use_sim_time', value=True)])
    processes = []
    logs = []
    rows = []
    counts = Counter()
    states = Counter()
    latest_pose = None
    sent_goal = False
    prefix = Path(get_package_prefix('px4_vio_bridge'))
    share = prefix / 'share/px4_vio_bridge'

    def start(name, args):
        f = (out / f'{name}.log').open('w')
        logs.append(f)
        p = subprocess.Popen(args, env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p)
        return p

    def receive(topic, msg):
        nonlocal latest_pose
        counts[topic] += 1
        if isinstance(msg, String):
            row = json.loads(msg.data)
            rows.append({'topic': topic, 'wall': time.time(), **row})
            if 'state' in row:
                states[topic + ':' + row['state']] += 1
            if topic == '/planner3d/pose_snapshot':
                latest_pose = row
    subs = []
    for topic in ['/planner3d/pose_snapshot', '/planner3d/pose_status', '/planner3d/status', '/planner3d/follower/status', '/planner3d/path_snapshot', '/planner3d/reference_snapshot', '/rtabmap/octomap_metadata', '/perf/processes']:
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic.endswith('metadata') else 10
        if topic.startswith('/perf/'):
            qos = qos_profile_sensor_data
        subs.append(node.create_subscription(String, topic, lambda msg, t=topic: receive(t, msg), qos))
    subs.append(node.create_subscription(Odometry, '/rtabmap/vio_odometry', lambda msg: receive('/rtabmap/vio_odometry', msg), qos_profile_sensor_data))
    goal_pub = node.create_publisher(PointStamped, '/waypoint/clicked', 10)
    held_start = None
    held_pub = None
    last_held_stamp = 0
    try:
        if args.hold_start_pose:
            import rosbag2_py
            from rclpy.serialization import deserialize_message
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='mcap'),
                        rosbag2_py.ConverterOptions('', ''))
            reader.set_filter(rosbag2_py.StorageFilter(topics=['/rtabmap/vio_odometry']))
            if not reader.has_next():
                raise RuntimeError('source bag has no raw odometry for the held-start diagnostic')
            _, data, _ = reader.read_next()
            held_start = deserialize_message(data, Odometry)
            del reader
            held_pub = node.create_publisher(Odometry, '/rtabmap/vio_odometry', qos_profile_sensor_data)
        if args.mapdata:
            backend = start('backend', [str(prefix / 'lib/px4_vio_bridge/rtabmap_octomap_node'), '--ros-args',
                '-p', 'use_sim_time:=true', '-p', 'max_marker_voxels:=5000',
                '-p', f'test_start_clear_radius:={args.test_start_clear_radius}',
                '-p', f'octomap_use_checkpoints:={str(not args.disable_checkpoints).lower()}'])
        else:
            backend = start('backend', [str(prefix / 'lib/px4_vio_bridge/ros_rtabmap_backend'), 'start_camera:=false', 'use_sim_time:=true', 'foxglove:=false', 'use_imu:=false', 'odometry_rate_hz:=0.0', 'odom_reset_countdown:=0', 'keep_latest_frame:=true', 'odom_key_frame_thr:=0.3', 'odom_vis_keyframe_thr:=150', f'camera_config:={share}/config/oak_v3_rtabmap_tracked.yaml', f'odom_config_path:={share}/config/rtabmap_ros_parity.ini', 'max_marker_voxels:=0', f'test_start_clear_radius:={args.test_start_clear_radius}', f'octomap_use_checkpoints:={str(not args.disable_checkpoints).lower()}'])
        monitor = start('monitor', ['ros2', 'launch', 'px4_vio_bridge', 'ros_planner_3d_monitor.launch.py', 'use_sim_time:=true', f'min_z:={args.min_z}', f'max_z:={args.max_z}', f'goal_height_mode:={args.goal_height_mode}', f'exploration_route:={str(not args.strict_route).lower()}', f'max_marker_voxels:={args.max_marker_voxels}'])
        from px4_vio_bridge.process_monitor import NATIVE_3D_TARGETS
        perf_topics = ['/perf/cpu_percent', '/perf/mem_percent', '/perf/cpu_temp_c', '/perf/load1', '/perf/throttled', '/perf/processes']
        perf_topics += [f'/perf/process/{label}/{metric}' for label in NATIVE_3D_TARGETS for metric in ('cpu_percent', 'mem_mb')]
        recorder = start('recorder', ['ros2', 'bag', 'record', '--storage', 'mcap', '--storage-preset-profile', 'fastwrite', '--disable-keyboard-controls', '--output', str(out / 'bag'), '--topics', '/clock', '/tf', '/tf_static', '/rtabmap/mapData', '/rtabmap/octomap', '/rtabmap/octomap_metadata', '/rtabmap/vio_odometry', '/planner3d/pose_snapshot', '/planner3d/path_snapshot', '/planner3d/reference_path', '/planner3d/reference_snapshot', '/planner3d/status', '/planner3d/pose_status', '/planner3d/follower/status', '/planner3d/follower/velocity', '/planner3d/follower/acceleration', '/planner3d/follower/valid', '/waypoint/clicked', '/planner3d/test_start_clear', '/planner3d/markers', *perf_topics])
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            names = node.get_node_names()
            if ('rtabmap_octomap' if args.mapdata else 'rtabmap') in names and 'ros_planner_pose' in names and ('route_follower_3d_monitor' in names) and 'process_monitor' in names:
                break
            if backend.poll() is not None or monitor.poll() is not None:
                raise RuntimeError('graph exited before playback')
        else:
            raise RuntimeError('required navigation nodes were not discovered')
        input_topics = ['/rtabmap/mapData', '/rtabmap/vio_odometry', '/tf_static'] if args.mapdata else ['/oak/left/image_rect', '/oak/stereo/image_raw', '/oak/left/camera_info', '/tf_static']
        if args.hold_start_pose:
            input_topics.remove('/rtabmap/vio_odometry')
        if args.recorded_goals:
            input_topics.append('/waypoint/clicked')
        player = start('player', ['ros2', 'bag', 'play', str(args.bag), '--clock', '100', '--delay', '2', '--disable-keyboard-controls', '--topics', *input_topics])
        end = time.monotonic() + args.timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if held_pub is not None:
                stamp = node.get_clock().now().nanoseconds
                if stamp - last_held_stamp >= 66_666_667:
                    odom = copy.deepcopy(held_start)
                    odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(stamp, 10**9)
                    held_pub.publish(odom)
                    last_held_stamp = stamp
            if latest_pose and not sent_goal and not args.recorded_goals:
                goal = PointStamped()
                goal.header.frame_id = 'world'
                goal.header.stamp = node.get_clock().now().to_msg()
                goal.point.x = latest_pose['pose'][0] + args.goal_offset
                goal.point.y = latest_pose['pose'][1]
                goal.point.z = latest_pose['pose'][2]
                goal_pub.publish(goal)
                sent_goal = True
            if player.poll() is not None:
                break
            if backend.poll() is not None or monitor.poll() is not None:
                raise RuntimeError('graph exited during playback')
        if player.poll() is None:
            raise RuntimeError('playback deadline')
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        topics = [name for name, _ in node.get_topic_names_and_types()]
        assert not any((name.startswith('/fmu/in/') for name in topics)), topics
        summary = {'pose_interface_valid': counts['/planner3d/pose_snapshot'] > 0 and counts['/rtabmap/octomap_metadata'] > 0, 'source_bag': str(args.bag), 'domain': args.domain, 'counts': dict(counts), 'states': dict(states), 'goal_sent': sent_goal, 'no_fmu_inputs': True, 'player_returncode': player.returncode, 'latest_pose': latest_pose}
        summary.update({'mapdata_replay': args.mapdata, 'checkpoints_enabled': not args.disable_checkpoints,
                        'goal_height_mode': args.goal_height_mode, 'exploration_route': not args.strict_route, 'max_marker_voxels': args.max_marker_voxels,
                        'test_start_clear_radius': args.test_start_clear_radius, 'recorded_goals': args.recorded_goals,
                        'synthetic_held_start_pose': args.hold_start_pose})
        (out / 'report.json').write_text(json.dumps(summary, indent=2) + '\n')
        if not summary['pose_interface_valid']:
            raise RuntimeError('replay produced no valid map/pose interface')
    except Exception as error:
        (out / 'incomplete.json').write_text(json.dumps({'error': str(error)}, indent=2) + '\n')
        raise
    finally:
        (out / 'samples.jsonl').write_text(''.join((json.dumps(row) + '\n' for row in rows)))
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait(timeout=5)
        for f in logs:
            f.close()
        node.destroy_node()
        rclpy.shutdown()
    print(out)
if __name__ == '__main__':
    main()
