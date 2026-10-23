"""Production adapter HOLD/recovery regression, with all FMU names isolated."""
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
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from px4_msgs.msg import (TrajectorySetpoint, VehicleCommand, VehicleControlMode,
                          VehicleLocalPosition, VehicleOdometry)
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, Int32, String


class Adapter:
    def __init__(self, tmp_path):
        self.domain = 130 + os.getpid() % 20
        self.context = Context()
        rclpy.init(context=self.context, domain_id=self.domain)
        self.node = Node('fault_hold_mock', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.prefix = '/fault_hold_test'
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
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
        self.pub = {topic: self.node.create_publisher(kind, self.remap(topic), qos)
                    for kind, topic, qos in kinds}
        self.sp, self.status, self.commands, self.config = [], [], [], []
        self.subs = [self.node.create_subscription(kind, self.remap(topic), target.append, qos)
                     for kind, topic, target, qos in [
                         (TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.sp, qos_profile_sensor_data),
                         (VehicleCommand, '/fmu/in/vehicle_command', self.commands, qos_profile_sensor_data),
                         (String, '/planner/flight/status', self.status, 10),
                         (String, '/planner/flight/config', self.config, durable)]]
        binary = os.environ.get('PX4_HOLD_TEST_BINARY') or (
            Path(get_package_prefix('px4_vio_bridge'))/'lib/px4_vio_bridge/cpp_flight_adapter')
        args = [str(binary), '--ros-args']
        params = {'auto_arm': False, 'keyboard_kill': False, 'keyboard_land': False,
                  'rate_hz': 20., 'stream_time': .1, 'climb_timeout': .3,
                  'max_flight_time': 90., 'command_speed': .2, 'command_acceleration': .3,
                  'min_vio_features': 20, 'yaw_follows_heading': False,
                  'max_correction_m': 1., 'map_pose_timeout': .4, 'correction_timeout': .4,
                  'map_timeout': .4, 'geofence_radius': 3., 'transit_horizontal_error': 2.,
                  'max_horizontal_error': 2., 'follower_latch_grace': 0.,
                  'route_entry_grace': 0., 'planner_fault_land_time': 1.2,
                  'route_command_grace': 1.}
        for key, value in params.items():
            args += ['-p', f'{key}:={str(value).lower()}']
        for topic in ['/fmu/out/vehicle_local_position_v1', '/fmu/out/vehicle_control_mode',
                      '/fmu/in/vehicle_visual_odometry', '/fmu/in/vehicle_command',
                      '/fmu/in/trajectory_setpoint', '/fmu/in/offboard_control_mode',
                      '/fmu/out/sensor_combined']:
            args += ['-r', f'{topic}:={self.remap(topic)}']
        self.log_path = tmp_path/'adapter.log'
        self.log = self.log_path.open('w')
        self.process = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT,
                                        env={**os.environ, 'ROS_DOMAIN_ID': str(self.domain)},
                                        start_new_session=True)
        self.north = self.east = self.correction_y = self.path_y = 0.
        self.valid = True
        self.armed = self.stale_map = self.block_target = False
        self.last_pump = 0.

    def remap(self, topic):
        return self.prefix+topic if topic.startswith('/fmu/') else topic

    def pump(self):
        now = time.monotonic()
        if now-self.last_pump < .02:
            return
        self.last_pump = now
        stamp = self.node.get_clock().now().to_msg()
        us = (stamp.sec*10**9+stamp.nanosec)//1000
        lp = VehicleLocalPosition()
        lp.timestamp = us
        lp.xy_valid = lp.z_valid = lp.v_xy_valid = lp.v_z_valid = True
        lp.x, lp.y, lp.z, lp.heading = self.north, self.east, -.3, math.pi/2
        self.pub['/fmu/out/vehicle_local_position_v1'].publish(lp)
        mode = VehicleControlMode()
        mode.timestamp = us
        mode.flag_control_offboard_enabled = True
        mode.flag_armed = self.armed
        self.pub['/fmu/out/vehicle_control_mode'].publish(mode)
        raw = PoseStamped()
        raw.header.stamp = stamp
        raw.header.frame_id = 'vio'
        raw.pose.position.x, raw.pose.position.y = self.east, self.north
        raw.pose.position.z = .3
        raw.pose.orientation.w = 1.
        self.pub['/rtabmap/vio_pose'].publish(raw)
        self.pub['/rtabmap/vio_feature_count'].publish(Int32(data=100))
        ev = VehicleOdometry()
        ev.timestamp = ev.timestamp_sample = us
        ev.pose_frame = VehicleOdometry.POSE_FRAME_NED
        ev.position = [self.north, self.east, -.3]
        ev.q = [math.cos(math.pi/4), 0., 0., math.sin(math.pi/4)]
        self.pub['/fmu/in/vehicle_visual_odometry'].publish(ev)
        pose = copy.deepcopy(raw)
        pose.header.frame_id = 'world'
        pose.pose.position.y += self.correction_y
        correction = copy.deepcopy(pose)
        correction.pose.position.x = correction.pose.position.z = 0.
        correction.pose.position.y = self.correction_y
        self.pub['/rtabmap/body_pose'].publish(pose)
        self.pub['/rtabmap/odom_correction'].publish(correction)
        path = RosPath()
        path.header = copy.deepcopy(pose.header)
        for x in [0., 2.]:
            point = copy.deepcopy(pose)
            point.pose.position.x = x
            point.pose.position.y = self.correction_y+self.path_y
            path.poses.append(point)
        self.pub['/planner/path'].publish(path)
        if not self.stale_map:
            grid = OccupancyGrid()
            grid.header = copy.deepcopy(pose.header)
            grid.info.resolution = .05
            grid.info.width = grid.info.height = 200
            grid.info.origin.position.x = grid.info.origin.position.y = -5.
            grid.info.origin.orientation.w = 1.
            grid.data = [0]*40000
            for x in range(200):
                grid.data[116*200+x] = 100
            if self.block_target and self.sp:
                x = int((self.sp[-1].position[1]+5.)/.05)
                y = int((self.sp[-1].position[0]+self.correction_y+5.)/.05)
                grid.data[y*200+x] = 100
            self.pub['/rtabmap/grid'].publish(grid)
            self.pub['/planner/inflated_map'].publish(grid)
        self.pub['/planner/status'].publish(String(data='PATH_VALID'))
        config = {'frame_id': 'world', 'robot_radius': .25, 'safety_margin': .03,
                  'occupied_threshold': 65, 'max_carrot_speed': .2}
        self.pub['/planner/follower/config'].publish(String(data=json.dumps(config)))
        self.pub['/planner/follower/valid'].publish(Bool(data=self.valid))
        self.pub['/planner/follower/goal_reached'].publish(Bool(data=False))
        displacement = Vector3Stamped()
        displacement.header = raw.header
        displacement.vector.x = .5
        displacement.vector.y = self.path_y-self.north
        self.pub['/planner/follower/vio_displacement'].publish(displacement)

    def wait(self, predicate, timeout=5.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.pump()
            self.executor.spin_once(timeout_sec=.005)
            assert self.process.poll() is None, self.log_path.read_text()
            if predicate():
                return
        raise AssertionError(self.log_path.read_text()[-6000:] + '\nRecent status:\n' +
                             '\n'.join(m.data for m in self.status[-10:]))

    def run(self, seconds):
        end = time.monotonic()+seconds
        self.wait(lambda: time.monotonic() >= end, seconds+1.)

    def close(self):
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)
        self.log.close()
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.context.shutdown()


@pytest.fixture
def adapter(tmp_path):
    a = Adapter(tmp_path)
    try:
        a.wait(lambda: any(m.data.startswith('ROUTE') for m in a.status))
        a.run(.7)
        yield a
        assert not any(m.command == 400 and m.param1 == 1 for m in a.commands)
    finally:
        a.close()


def xy(msg):
    return tuple(float(v) for v in msg.position[:2])


def distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def test_hold_retains_reference_and_recovery_joins_without_jump(adapter):
    a = adapter
    for shift in [.06, -.04]:
        before = a.sp[-1]
        a.north = float(before.position[0])+.242
        a.east = float(before.position[1])
        a.valid = False
        start = len(a.status)
        a.wait(lambda: any(m.data.startswith('HOLD') for m in a.status[start:]))
        a.run(.2)
        held = xy(a.sp[-1])
        assert abs(held[0]-before.position[0]) < .003
        assert distance(held, (a.north, a.east)) > .23
        start = len(a.sp)
        a.correction_y += .15  # A new map solution while the NED reference holds.
        a.run(.25)
        assert all(distance(xy(m), held) < 1e-6 for m in a.sp[start:])
        assert all(math.isnan(m.velocity[0]) for m in a.sp[start:])
        a.path_y = held[0]+shift
        a.valid = True
        start = len(a.sp)
        a.wait(lambda: any(math.isfinite(m.velocity[0]) for m in a.sp[start:]))
        a.run(.35)
        previous = held
        first = True
        for msg in a.sp[start:]:
            assert distance(xy(msg), previous) <= .0101
            if math.isfinite(msg.velocity[0]) and first:
                assert math.hypot(*msg.velocity[:2]) <= .0151
                first = False
            previous = xy(msg)
        assert not first
    config = json.loads(a.config[-1].data)
    assert config['fault_hold_reference'] == 'retain_last_command'
    assert config['fault_recovery_reference'] == 'join_from_held_command'


def test_unjoinable_recovery_preserves_reference_through_retries(adapter):
    a = adapter
    a.valid = False
    a.run(.2)
    held = xy(a.sp[-1])
    a.path_y = .45  # Beyond the existing connector limit.
    a.valid = True
    start = len(a.sp)
    a.run(.4)
    a.north += .08
    a.correction_y += .10
    a.run(.4)
    assert all(distance(xy(m), held) < 1e-6 for m in a.sp[start:])
    a.path_y = .06
    start = len(a.sp)
    a.wait(lambda: any(math.isfinite(m.velocity[0]) for m in a.sp[start:]))
    assert distance(xy(a.sp[-1]), held) < .011


def test_stale_map_freezes_reference_and_keeps_land_timer(adapter):
    a = adapter
    a.stale_map = True
    a.run(.5)
    held = xy(a.sp[-1])
    a.valid = False
    a.north = .65  # Below wall clearance; stale map must not authorize escape.
    a.armed = True  # Mock state only; every FMU topic is remapped.
    start = len(a.sp)
    a.wait(lambda: any(m.command == VehicleCommand.VEHICLE_CMD_NAV_LAND
                      for m in a.commands), timeout=2.)
    assert all(distance(xy(m), held) < 1e-6 for m in a.sp[start:])


def test_newly_blocked_held_reference_requests_land(adapter):
    a = adapter
    a.valid = False
    a.run(.2)
    held = xy(a.sp[-1])
    a.armed = True
    a.block_target = True
    a.wait(lambda: any(m.command == VehicleCommand.VEHICLE_CMD_NAV_LAND
                      for m in a.commands), timeout=.9)
    assert distance(xy(a.sp[-1]), held) < 1e-6
    assert 'held position reference no longer has clearance' in a.log_path.read_text()


def test_clearance_escape_does_not_pull_back_an_existing_corrective_target(adapter):
    a = adapter
    held = xy(a.sp[-1])
    a.north = .62  # Vehicle is inside the wall margin; old reference leads out.
    a.east = held[1]
    a.valid = False
    a.armed = True
    start = len(a.sp)
    a.run(.45)
    assert all(distance(xy(m), held) < .001 for m in a.sp[start:])
    assert not any(m.command == VehicleCommand.VEHICLE_CMD_NAV_LAND for m in a.commands)
