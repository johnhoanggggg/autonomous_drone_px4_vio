"""Exercise the production 2D adapter with split localization deliveries.

All FMU names are remapped in a separate DDS domain. No hardware is involved.
"""
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from px4_msgs.msg import (TrajectorySetpoint, VehicleCommand, VehicleControlMode,
                          VehicleLocalPosition, VehicleOdometry)
from std_msgs.msg import Bool, Int32, String


def test_adapter_pairs_localization_and_path_before_rebasing(tmp_path):
    domain = 190 + os.getpid() % 20
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node('localization_pair_mock', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    prefix = '/localization_test'
    kinds = [
        (VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', durable),
        (VehicleControlMode, '/fmu/out/vehicle_control_mode', durable),
        (VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile_sensor_data),
        (PoseStamped, '/rtabmap/vio_pose', 10),
        (Int32, '/rtabmap/vio_feature_count', 10),
        (PoseStamped, '/rtabmap/body_pose', 10),
        (PoseStamped, '/rtabmap/odom_correction', 10),
        (OccupancyGrid, '/rtabmap/grid', durable),
        (OccupancyGrid, '/planner/inflated_map', durable),
        (String, '/planner/status', 10),
        (RosPath, '/planner/path', 10),
        (String, '/planner/follower/config', durable),
        (Bool, '/planner/follower/valid', 10),
        (Bool, '/planner/follower/goal_reached', 10),
        (Vector3Stamped, '/planner/follower/vio_displacement', 10),
    ]
    pub = {topic: node.create_publisher(kind, prefix+topic if topic.startswith('/fmu/') else topic, qos)
           for kind, topic, qos in kinds}
    received = {'setpoints': [], 'status': [], 'commands': [], 'debug': []}
    subs = [node.create_subscription(kind, topic, lambda msg, key=key: received[key].append(msg), qos)
            for kind, topic, key, qos in [
                (TrajectorySetpoint, prefix+'/fmu/in/trajectory_setpoint', 'setpoints', qos_profile_sensor_data),
                (VehicleCommand, prefix+'/fmu/in/vehicle_command', 'commands', qos_profile_sensor_data),
                (String, '/planner/flight/status', 'status', 10),
                (String, '/planner/flight/heading_debug', 'debug', 10),
            ]]
    binary = Path(get_package_prefix('px4_vio_bridge'))/'lib/px4_vio_bridge/cpp_flight_adapter'
    args = [str(binary), '--ros-args']
    params = {'auto_arm': False, 'keyboard_kill': False, 'keyboard_land': False,
              'rate_hz': 20., 'stream_time': .1, 'climb_timeout': .3,
              'max_flight_time': 90., 'command_speed': .2, 'command_acceleration': .3,
              'min_vio_features': 20, 'yaw_follows_heading': False,
              'max_correction_m': 1., 'map_pose_timeout': .4, 'correction_timeout': .4,
              'geofence_radius': 3., 'transit_horizontal_error': 2.,
              'max_horizontal_error': 2.}
    for k, v in params.items():
        args += ['-p', f'{k}:={str(v).lower()}']
    for topic in ['/fmu/out/vehicle_local_position_v1', '/fmu/out/vehicle_control_mode',
                  '/fmu/in/vehicle_visual_odometry', '/fmu/in/vehicle_command',
                  '/fmu/in/trajectory_setpoint', '/fmu/in/offboard_control_mode',
                  '/fmu/out/sensor_combined']:
        args += ['-r', f'{topic}:={prefix}{topic}']
    log_path = tmp_path/'adapter.log'
    log = log_path.open('w')
    process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                               env={**os.environ, 'ROS_DOMAIN_ID': str(domain)}, start_new_session=True)
    last_pump = 0.
    correction_y = 0.
    split = None
    frozen = False
    last_pair = None

    def localization(stamp, offset):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'world'
        pose.pose.position.y = offset
        pose.pose.position.z = .3
        pose.pose.orientation.w = 1.
        correction = copy.deepcopy(pose)
        correction.pose.position.z = 0.
        path = RosPath()
        path.header = pose.header
        for x in [0., 2.]:
            point = copy.deepcopy(pose)
            point.pose.position.x = x
            path.poses.append(point)
        return pose, correction, path

    def pump():
        nonlocal last_pump, last_pair
        now = time.monotonic()
        if now-last_pump < .02:
            return
        last_pump = now
        stamp = node.get_clock().now().to_msg()
        ns = stamp.sec*10**9+stamp.nanosec
        lp = VehicleLocalPosition()
        lp.timestamp = ns//1000
        lp.xy_valid = lp.z_valid = True
        lp.z = -.3
        lp.heading = math.pi/2
        pub['/fmu/out/vehicle_local_position_v1'].publish(lp)
        mode = VehicleControlMode()
        mode.timestamp = ns//1000
        mode.flag_control_offboard_enabled = True
        pub['/fmu/out/vehicle_control_mode'].publish(mode)
        raw = PoseStamped()
        raw.header.stamp = stamp
        raw.header.frame_id = 'vio'
        raw.pose.position.z = .3
        raw.pose.orientation.w = 1.
        pub['/rtabmap/vio_pose'].publish(raw)
        pub['/rtabmap/vio_feature_count'].publish(Int32(data=100))
        ev = VehicleOdometry()
        ev.timestamp = ev.timestamp_sample = ns//1000
        ev.pose_frame = VehicleOdometry.POSE_FRAME_NED
        ev.position = [0., 0., -.3]
        ev.q = [math.cos(math.pi/4), 0., 0., math.sin(math.pi/4)]
        pub['/fmu/in/vehicle_visual_odometry'].publish(ev)
        grid = OccupancyGrid()
        grid.header = copy.deepcopy(raw.header)
        grid.header.frame_id = 'world'
        grid.info.resolution = .1
        grid.info.width = grid.info.height = 100
        grid.info.origin.position.x = grid.info.origin.position.y = -5.
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*10000
        # A wall outside the 28 cm envelope gives a finite clearance probe.
        for x in range(100):
            grid.data[58*100+x] = 100
        for topic in ['/rtabmap/grid', '/planner/inflated_map']:
            pub[topic].publish(grid)
        pub['/planner/status'].publish(String(data='PATH_VALID'))
        config = {'frame_id': 'world', 'robot_radius': .25, 'safety_margin': .03,
                  'occupied_threshold': 65, 'max_carrot_speed': .2}
        pub['/planner/follower/config'].publish(String(data=json.dumps(config)))
        pub['/planner/follower/valid'].publish(Bool(data=True))
        pub['/planner/follower/goal_reached'].publish(Bool(data=False))
        displacement = Vector3Stamped()
        displacement.header = raw.header
        displacement.vector.x = 1.
        pub['/planner/follower/vio_displacement'].publish(displacement)
        if split is None:
            if not frozen:
                last_pair = localization(stamp, correction_y)
            pose, correction, path = last_pair
            pub['/rtabmap/body_pose'].publish(pose)
            pub['/rtabmap/odom_correction'].publish(correction)
            pub['/planner/path'].publish(path)

    def wait(predicate, seconds=5.):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            pump()
            executor.spin_once(timeout_sec=.005)
            assert process.poll() is None, log_path.read_text()
            if predicate():
                return
        raise AssertionError(log_path.read_text()[-5000:])

    def run_for(seconds):
        end = time.monotonic()+seconds
        wait(lambda: time.monotonic() >= end, seconds+1.)

    try:
        wait(lambda: any(m.data.startswith('ROUTE') for m in received['status']))
        old_path = copy.deepcopy(last_pair[2])
        for order, offset in [('correction', .15), ('pose', -.10)]:
            split = localization(node.get_clock().now().to_msg(), offset)
            pose, correction, path = split
            first = '/rtabmap/odom_correction' if order == 'correction' else '/rtabmap/body_pose'
            pub[first].publish(correction if order == 'correction' else pose)
            # Path also arrives before its complete source pair.
            pub['/planner/path'].publish(path)
            start = len(received['setpoints'])
            run_for(.16)  # At least three 20 Hz control ticks between halves.
            assert len(received['setpoints'])-start >= 2
            second = '/rtabmap/body_pose' if order == 'correction' else '/rtabmap/odom_correction'
            pub[second].publish(pose if order == 'correction' else correction)
            run_for(.10)
            correction_y = offset
            split = None
            run_for(.12)
            assert max(abs(m.position[0]) for m in received['setpoints'][start:]) < .002
        # A delayed older path must not displace the latest accepted route.
        for point in old_path.poses:
            point.pose.position.y += .15
        pub['/planner/path'].publish(old_path)
        start = len(received['setpoints'])
        run_for(.16)
        assert max(abs(m.position[0]) for m in received['setpoints'][start:]) < .002
        # An unknown future source must not poison ordering of valid paths.
        future_path = copy.deepcopy(last_pair[2])
        future_path.header.stamp.sec += 10000
        split = last_pair  # Pause normal paths until the injection is processed.
        pub['/planner/path'].publish(future_path)
        run_for(.10)
        before = json.loads(received['debug'][-1].data)['latest_path_input']
        split = None
        run_for(.16)
        assert json.loads(received['debug'][-1].data)['latest_path_input'] > before
        # Repeated delivery must not keep an old source sample fresh forever.
        frozen = True
        start = len(received['status'])
        wait(lambda: any('stale' in m.data for m in received['status'][start:]), seconds=1.5)
        assert not any(m.command == 400 and m.param1 == 1 for m in received['commands'])
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        log.close()
        executor.remove_node(node)
        node.destroy_node()
        context.shutdown()


def test_planner_and_follower_share_path_localization_provenance(tmp_path):
    from geometry_msgs.msg import PointStamped

    domain = 210 + os.getpid() % 10
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node('paired_route_mock', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = {topic: node.create_publisher(kind, topic, qos) for kind, topic, qos in [
        (PoseStamped, '/rtabmap/body_pose', 10),
        (PoseStamped, '/rtabmap/body_vio_pose', 10),
        (PoseStamped, '/rtabmap/odom_correction', 10),
        (OccupancyGrid, '/rtabmap/grid', durable),
        (PointStamped, '/waypoint/clicked', 10),
    ]}
    paths, statuses = [], []
    delayed_path_pub = node.create_publisher(RosPath, '/planner/path', 10)
    subs = [node.create_subscription(RosPath, '/planner/path', paths.append, 10),
            node.create_subscription(String, '/planner/follower/status', statuses.append, 10)]
    folder = Path(get_package_prefix('px4_vio_bridge'))/'lib/px4_vio_bridge'
    processes, logs = [], []
    for binary in ['cpp_astar_planner', 'cpp_route_follower']:
        log = (tmp_path/f'{binary}.log').open('w')
        logs.append(log)
        processes.append(subprocess.Popen([str(folder/binary), '--ros-args', '-p', 'max_correction_m:=1.0'],
                         env={**os.environ, 'ROS_DOMAIN_ID': str(domain)}, stdout=log,
                         stderr=subprocess.STDOUT, start_new_session=True))
    stamps = set()
    start = time.monotonic()
    goal_sent = 0.
    next_map = 0.
    try:
        while time.monotonic()-start < 5.:
            now = time.monotonic()
            pose = PoseStamped()
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.header.frame_id = 'world'
            pose.pose.position.x = pose.pose.position.y = .05
            pose.pose.position.z = .3
            pose.pose.orientation.w = 1.
            source = pose.header.stamp.sec*10**9+pose.header.stamp.nanosec
            stamps.add(source)
            correction = copy.deepcopy(pose)
            correction.pose.position.x = correction.pose.position.y = correction.pose.position.z = 0.
            raw = copy.deepcopy(pose)
            raw.header.frame_id = 'vio'
            # Alternate delivery order: publication timestamps still identify
            # the same complete sample and must be carried onto every path.
            order = [('body_pose', pose), ('odom_correction', correction)]
            if len(stamps) % 2:
                order.reverse()
            for topic, msg in order:
                pub['/rtabmap/'+topic].publish(msg)
            pub['/rtabmap/body_vio_pose'].publish(raw)
            if now >= next_map:
                grid = OccupancyGrid()
                grid.header = copy.deepcopy(pose.header)
                grid.info.resolution = .1
                grid.info.width = grid.info.height = 50
                grid.info.origin.position.x = grid.info.origin.position.y = -2.5
                grid.info.origin.orientation.w = 1.
                grid.data = [0]*2500
                for x in range(50):
                    grid.data[33*50+x] = 100
                pub['/rtabmap/grid'].publish(grid)
                next_map = now+.8
            # Repeat until discovery; then stop changing goal generation.
            if now-start < 1. and now-goal_sent > .2:
                goal = PointStamped()
                goal.header = copy.deepcopy(pose.header)
                goal.point.x = 1.5
                pub['/waypoint/clicked'].publish(goal)
                goal_sent = now
            for process in processes:
                assert process.poll() is None
            executor.spin_once(timeout_sec=.02)
            if len(paths) >= 3 and any(m.data.startswith('FOLLOWING') for m in statuses):
                break
        assert len(paths) >= 3, [p.read_text() for p in tmp_path.glob('*.log')]
        assert any(m.data.startswith('FOLLOWING') for m in statuses), [m.data for m in statuses]
        assert all(p.header.stamp.sec*10**9+p.header.stamp.nanosec in stamps for p in paths if p.poses)
        assert len({p.header.stamp.sec*10**9+p.header.stamp.nanosec for p in paths if p.poses}) >= 2
        # An old empty veto must not erase the newer accepted route.
        empty = RosPath()
        empty.header.frame_id = 'world'
        before = len(statuses)
        delayed_path_pub.publish(empty)
        until = time.monotonic()+.25
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
        assert any(not p.poses for p in paths)  # Injection was actually received.
        assert not any(m.data.startswith('WAITING_FOR_PATH') for m in statuses[before:])
    finally:
        for process in processes:
            process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        for log in logs:
            log.close()
        executor.remove_node(node)
        node.destroy_node()
        context.shutdown()
