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
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
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
            (Twist, '/planner/flight/teleop', 10),
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
                  'foxglove_teleop': True,
                  'rate_hz': 20., 'stream_time': .1, 'climb_timeout': .3,
                  'max_flight_time': 90., 'command_speed': .2, 'command_acceleration': .3,
                  'min_vio_features': 20, 'yaw_follows_heading': False,
                  'max_correction_m': 1., 'map_pose_timeout': .4, 'correction_timeout': .4,
                  'map_timeout': .4, 'geofence_radius': 3., 'transit_horizontal_error': 2.,
                  'max_horizontal_error': 2.,
                  'route_entry_grace': 0., 'planner_fault_land_time': 3.,
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
        self.vn = self.ve = self.correction_yaw = 0.
        self.model = False
        self.local_enabled = self.velocity_valid = True
        self.velocity_reset = 0
        self.valid = True
        self.armed = self.stale_map = self.block_target = False
        self.last_pump = 0.

    def remap(self, topic):
        return self.prefix+topic if topic.startswith('/fmu/') else topic

    def pump(self):
        now = time.monotonic()
        if now-self.last_pump < .02:
            return
        elapsed = min(.04, now-self.last_pump) if self.last_pump else .02
        self.last_pump = now
        if self.model and self.sp:
            command = self.sp[-1]
            desired = [float(v) if math.isfinite(v) else 0. for v in command.velocity[:2]]
            if all(math.isfinite(p) for p in command.position[:2]):
                desired[0] += 1.5*(float(command.position[0])-self.north)
                desired[1] += 1.5*(float(command.position[1])-self.east)
            # Delayed measured response: settling must follow the mock aircraft,
            # not merely the reference reaching zero. This is not an airframe model.
            self.vn += (desired[0]-self.vn)*elapsed/.25
            self.ve += (desired[1]-self.ve)*elapsed/.25
            self.north += self.vn*elapsed
            self.east += self.ve*elapsed
        stamp = self.node.get_clock().now().to_msg()
        us = (stamp.sec*10**9+stamp.nanosec)//1000
        lp = VehicleLocalPosition()
        lp.timestamp = us
        lp.xy_valid = lp.z_valid = lp.v_xy_valid = lp.v_z_valid = True
        lp.x, lp.y, lp.z, lp.heading = self.north, self.east, -.3, math.pi/2
        lp.vx, lp.vy = self.vn, self.ve
        lp.v_xy_valid = self.velocity_valid
        lp.vxy_reset_counter = self.velocity_reset
        if self.local_enabled:
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
        co, si = math.cos(self.correction_yaw), math.sin(self.correction_yaw)
        pose.pose.position.x = co*self.east-si*self.north
        pose.pose.position.y = si*self.east+co*self.north+self.correction_y
        correction = copy.deepcopy(pose)
        correction.pose.position.x = correction.pose.position.z = 0.
        correction.pose.position.y = self.correction_y
        correction.pose.orientation.z = math.sin(self.correction_yaw/2)
        correction.pose.orientation.w = math.cos(self.correction_yaw/2)
        self.pub['/rtabmap/body_pose'].publish(pose)
        self.pub['/rtabmap/odom_correction'].publish(correction)
        path = RosPath()
        path.header = copy.deepcopy(pose.header)
        for x in [0., 2.]:
            point = copy.deepcopy(pose)
            point.pose.position.x = co*x-si*self.path_y
            point.pose.position.y = si*x+co*self.path_y+self.correction_y
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


def braking(msg):
    return all(math.isnan(float(p)) for p in msg.position[:2])


def landed(a):
    return any(m.command == VehicleCommand.VEHICLE_CMD_NAV_LAND for m in a.commands)


def test_velocity_stop_waits_for_aircraft_then_rejoins_continuously(adapter):
    a = adapter
    a.north = .242
    a.east = xy(a.sp[-1])[1]
    a.vn = -.259  # Returning quickly, as in the failed flight.
    a.valid = False
    start = len(a.sp)
    a.wait(lambda: any(braking(m) for m in a.sp[start:]))
    first = next(m for m in a.sp[start:] if braking(m))
    assert first.velocity[0] == pytest.approx(-.259, abs=.001)
    assert math.isfinite(first.position[2])  # Altitude is still position-controlled.
    a.valid = True  # Recovery must remain inhibited despite an early valid follower.
    a.model = True
    a.path_y = .12
    a.run(.4)
    assert all(braking(m) for m in a.sp[start:] if m.timestamp >= first.timestamp)
    a.wait(lambda: not braking(a.sp[-1]), timeout=4.)
    capture = a.sp[-1]
    assert distance(xy(capture), (a.north, a.east)) < .005
    assert math.hypot(*capture.velocity[:2]) <= .017
    trace = [m for m in a.sp[start:] if braking(m)]
    for old, new in zip(trace, trace[1:]):
        elapsed = (new.timestamp-old.timestamp)/1e6
        assert distance(new.velocity[:2], old.velocity[:2]) <= .3*elapsed+.002
        assert new.velocity[0] <= .0001  # No commanded reversal while braking.
        assert math.hypot(*new.acceleration[:2]) <= .3001
    start = len(a.sp)
    previous = xy(capture)
    a.run(.35)
    for msg in a.sp[start:]:
        assert distance(xy(msg), previous) < .012
        previous = xy(msg)
    config = json.loads(a.config[-1].data)
    assert config['fault_hold_reference'] == 'measured_velocity_stop_then_capture'
    assert config['fault_recovery_reference'] == 'join_from_settled_capture'


def test_unjoinable_recovery_preserves_settled_capture_during_map_correction(adapter):
    a = adapter
    a.valid = False
    a.wait(lambda: braking(a.sp[-1]))
    a.wait(lambda: not braking(a.sp[-1]))
    captured = xy(a.sp[-1])
    a.path_y = .45
    a.valid = True
    start = len(a.sp)
    a.run(.4)
    a.correction_y += .1
    a.correction_yaw = .04
    a.run(.4)
    assert all(distance(xy(m), captured) < 1e-6 for m in a.sp[start:])
    a.path_y = .06
    start = len(a.sp)
    a.wait(lambda: any(math.hypot(*m.velocity[:2]) > .001 for m in a.sp[start:]))
    assert distance(xy(a.sp[-1]), captured) < .002


def test_measured_overshoot_cannot_resume_and_original_timeout_lands(adapter):
    a = adapter
    a.vn = .15
    a.valid = False
    a.armed = True
    start = len(a.sp)
    a.wait(lambda: any(braking(m) for m in a.sp[start:]))
    a.valid = True
    a.run(1.)
    assert braking(a.sp[-1])
    assert math.hypot(*a.sp[-1].velocity[:2]) < 1e-6
    assert not landed(a)  # No false capture just because commanded velocity is zero.
    a.wait(lambda: landed(a), timeout=2.5)
    assert 'planner fault persisted' in a.log_path.read_text()


def test_obstructed_predicted_stop_requests_land(adapter):
    a = adapter
    a.north = .35  # Body has clearance; predicted +north stop corridor does not.
    a.vn = .342
    a.valid = False
    a.armed = True
    a.wait(lambda: landed(a), timeout=1.)
    assert 'velocity stop measured corridor rejected:' in a.log_path.read_text()
    assert 'required=0.280m known=true' in a.log_path.read_text()


@pytest.mark.parametrize('failure', ['map', 'local_position', 'velocity', 'reset'])
def test_invalid_stop_inputs_request_land(adapter, failure):
    a = adapter
    a.vn = -.1
    a.valid = False
    a.armed = True
    a.wait(lambda: braking(a.sp[-1]))
    if failure == 'map':
        a.stale_map = True
    elif failure == 'local_position':
        a.local_enabled = False
    elif failure == 'velocity':
        a.velocity_valid = False
    else:
        a.velocity_reset += 1
    a.wait(lambda: landed(a), timeout=1.)


def test_disturbance_after_capture_removes_position_target_again(adapter):
    a = adapter
    a.valid = False
    a.wait(lambda: braking(a.sp[-1]))
    a.wait(lambda: not braking(a.sp[-1]))
    a.vn = -.16
    a.wait(lambda: braking(a.sp[-1]))
    assert a.sp[-1].velocity[0] < -.1


def test_operator_land_during_braking_cannot_restore_old_position(adapter):
    a = adapter
    a.vn = -.1
    a.valid = False
    a.armed = True
    a.wait(lambda: braking(a.sp[-1]))
    a.north -= .06
    a.run(.1)
    request = Twist()
    request.linear.z = -1.
    a.pub['/planner/flight/teleop'].publish(request)
    a.wait(lambda: landed(a), timeout=1.)
    a.run(.1)
    assert distance(xy(a.sp[-1]), (a.north, a.east)) < .001
    assert all(math.isnan(float(v)) for v in a.sp[-1].velocity[:2])


def test_default_follower_dropout_brakes_without_degraded_crawl(adapter):
    # Exercise production defaults, not a test-only grace override. This window
    # is shorter than the previous 1.5 s grace that delayed the recorded stops.
    a = adapter
    config = json.loads(a.config[-1].data)
    assert config['follower_latch_grace'] == 0.
    assert config['follower_drop_policy'] == 'immediate_velocity_stop'
    a.vn = -.10
    start = len(a.sp)
    status_start = len(a.status)
    a.valid = False
    a.wait(lambda: any(braking(m) for m in a.sp[start:]), timeout=.6)
    assert not any(m.data.startswith('DEGRADED') for m in a.status[status_start:])
    assert not landed(a)


def test_momentum_guard_brakes_before_valid_follower_cross_track_fault(adapter):
    a = adapter
    # Wall at north=.8: body clearance .45, nominal stopping chord .2833.
    # Hard .28 clearance still passes, but .05 reserve is almost consumed.
    a.north = .35
    a.vn = .25
    a.path_y = .35
    assert a.valid
    start = len(a.sp)
    a.wait(lambda: any(braking(m) for m in a.sp[start:]), timeout=.6)
    first = next(m for m in a.sp[start:] if braking(m))
    assert first.velocity[0] == pytest.approx(.25, abs=.001)
    assert 'route stopping reserve:' in a.log_path.read_text()
    assert not landed(a)
    # The valid follower cannot bypass measured settling, then can recover.
    a.vn = 0.
    a.run(.2)
    assert braking(a.sp[-1])
    a.wait(lambda: not braking(a.sp[-1]), timeout=2.)
    assert distance(xy(a.sp[-1]), (a.north, a.east)) < .005
    config = json.loads(a.config[-1].data)
    assert config['route_stop_clearance_reserve'] == .05
    assert config['route_stop_policy'] == 'measured_momentum_clearance_guard'


@pytest.mark.parametrize('vn,ve', [(-.10, 0.), (0., .10), (.02, 0.)])
def test_momentum_guard_allows_escape_parallel_and_settled_motion(adapter, vn, ve):
    a = adapter
    a.north = .48  # .32 m clearance: inside .33 reserve, outside hard margin.
    a.vn, a.ve = vn, ve
    start = len(a.sp)
    a.run(.3)
    assert a.sp[start:]
    assert all(not braking(m) for m in a.sp[start:])
    assert not landed(a)


def test_momentum_guard_cannot_waive_obstructed_stop_for_valid_follower(adapter):
    a = adapter
    a.north = .35
    a.vn = .342
    a.armed = True
    assert a.valid
    a.wait(lambda: landed(a), timeout=.6)
    assert 'velocity stop measured corridor rejected:' in a.log_path.read_text()
