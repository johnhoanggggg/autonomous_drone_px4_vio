"""Production 3D adapter against a mock PX4, with all FMU topics remapped.
No camera, hardware DDS domain, or real arm command is used.
"""
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
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from octomap_msgs.msg import Octomap
from px4_msgs.msg import VehicleLocalPosition, VehicleControlMode, VehicleOdometry, VehicleCommand, VehicleStatus, TrajectorySetpoint
from rtabmap_msgs.msg import OdomInfo
from std_msgs.msg import String
from px4_vio_bridge.process_singleton import ProcessSingleton


def occupied_tree(points, resolution=.05, free_points=()):
    """OctoMap binary format: two bits per child, depth-first inner nodes."""
    root = {}
    for point, leaf_state in [*((point, 1) for point in free_points),
                              *((point, 2) for point in points)]:
        key = [int(math.floor(x/resolution))+32768 for x in point]
        node = root
        for bit in range(15, -1, -1):
            index = sum(((key[axis] >> bit) & 1) << axis for axis in range(3))
            if bit == 0:
                node[index] = leaf_state
            else:
                node = node.setdefault(index, {})

    def encode(node):
        bits = sum((child if isinstance(child, int) else 3) << (2*i)
                   for i, child in node.items())
        out = [bits & 255, bits >> 8]
        for i in sorted(node):
            if not isinstance(node[i], int):
                out.extend(encode(node[i]))
        return out
    return [b if b < 128 else b-256 for b in encode(root)]


@pytest.mark.parametrize('auto_arm,terminal_fault', [(False,'lost_info'), (True,'px4_reset'), (True,'ev_reset')])
def test_unknown_flight_adapter_mock_px4(tmp_path, auto_arm, terminal_fault):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    domain = 160 + os.getpid() % 30
    context = Context(); rclpy.init(context=context, domain_id=domain)
    node = Node('flight3d_mock', context=context)
    executor = SingleThreadedExecutor(context=context); executor.add_node(node)
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    prefix = '/flight3d_test'
    pub = {}
    for kind, topic, qos in [
        (VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', durable),
        (VehicleControlMode, '/fmu/out/vehicle_control_mode', durable),
        (VehicleStatus, '/fmu/out/vehicle_status_v1', durable),
        (VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile_sensor_data),
        (PoseStamped, '/rtabmap/vio_pose', 10), (OdomInfo, '/rtabmap/vio_odom_info_lite', qos_profile_sensor_data),
        (Octomap, '/rtabmap/octomap', durable), (String, '/rtabmap/octomap_metadata', durable),
        (String, '/planner3d/pose_snapshot', 10), (String, '/planner3d/path_snapshot', 10),
    ]:
        pub[topic] = node.create_publisher(kind, prefix+topic if topic.startswith('/fmu/') else topic, qos)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(tmp_path/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    record_types = {'/rtabmap/octomap':'octomap_msgs/msg/Octomap',
                    '/rtabmap/octomap_metadata':'std_msgs/msg/String',
                    '/planner3d/flight/status':'std_msgs/msg/String',
                    '/planner3d/flight/config':'std_msgs/msg/String',
                    '/fmu/in/trajectory_setpoint':'px4_msgs/msg/TrajectorySetpoint'}
    for i,(topic,kind) in enumerate(record_types.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(id=i,name=topic,type=kind,serialization_format='cdr'))

    def record(topic,msg):
        if writer is not None:writer.write(topic,serialize_message(msg),node.get_clock().now().nanoseconds)

    received = {'status': [], 'commands': [], 'setpoints': []}
    def receive(key,msg):
        received[key].append(msg)
        if key=='status':record('/planner3d/flight/status',msg)
        elif key=='setpoints':record('/fmu/in/trajectory_setpoint',msg)
    setpoint_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    subs = [node.create_subscription(kind, topic, lambda m, key=key: receive(key,m), qos)
            for kind, topic, key, qos in [
                (String, '/planner3d/flight/status', 'status', 10),
                (VehicleCommand, prefix+'/fmu/in/vehicle_command', 'commands', qos_profile_sensor_data),
                (TrajectorySetpoint, prefix+'/fmu/in/trajectory_setpoint', 'setpoints', setpoint_qos)]]
    subs.append(node.create_subscription(String,'/planner3d/flight/config',lambda m:record('/planner3d/flight/config',m),durable))
    binary = Path(get_package_prefix('px4_vio_bridge'))/'lib/px4_vio_bridge/cpp_flight_adapter_3d'
    args = [str(binary), '--ros-args']
    params = {'auto_arm': auto_arm, 'allow_unknown': False,
              'path_snapshot_topic': '/planner3d/path_snapshot', 'hover_height': .3,
              'min_height': .1, 'max_height': 1.5, 'command_speed': .3,
              'command_vertical_speed': .3, 'command_acceleration': .6,
              'command_vertical_acceleration': .6, 'min_vio_features': 20,
              'climb_rate': .25, 'climb_leash': .12, 'climb_release': .05,
              'climb_feedforward': True,
              'rate_hz': 10., 'path_timeout': 3.5, 'px4_timeout': .5,
              'vehicle_control_mode_timeout': .75, 'follower_rejoin_grace': 1.5,
              'require_px4_preflight_status': True, 'px4_preflight_timeout': 1.,
              'stream_time': .2, 'climb_timeout': 8., 'planner_fault_land_time': 1.5,
              'max_flight_time': 90., 'goal_hold_time': 20., 'keyboard_kill': False,
              'keyboard_land': False, 'yaw_follows_heading': auto_arm, 'yaw_rate_deg': 180.,
              'brake_damping': True}
    for k,v in params.items(): args += ['-p', f'{k}:={str(v).lower()}']
    for topic in ['/fmu/out/vehicle_local_position_v1', '/fmu/out/vehicle_control_mode',
                  '/fmu/out/vehicle_status_v1',
                  '/fmu/in/vehicle_visual_odometry', '/fmu/in/vehicle_command',
                  '/fmu/in/trajectory_setpoint', '/fmu/in/offboard_control_mode', '/fmu/out/sensor_combined']:
        args += ['-r', f'{topic}:={prefix}{topic}']
    log = (tmp_path/'adapter.log').open('w')
    process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
        env={**os.environ, 'ROS_DOMAIN_ID': str(domain)}, start_new_session=True)
    position = [4., -2., 1.]; ground = position[:]
    armed = False; offboard = False; generation = 0; map_stamp = 0; goal_generation = 1
    goal = [.7,.1,.65]; next_map = 0.; next_path = 0.; last_pump = 0.; command_index = 0
    pre_hover_path_pending = True; path_generation_lag = 0; map_history = {}
    preflight_ready = False
    fault = None; obstacle = None; external_offset = [0., 0., 0.]; drift_velocity = [0., 0.]
    pose_lag = False; map_published_at = 0.
    # Use native voxel centres. Coordinates on cell boundaries can round into
    # an adjacent OctoMap key and leave a checkerboard of accidental unknowns.
    free_points = [((x+.5)*.05, (y+.5)*.05, (z+.5)*.05)
                   for x in range(-6, 23) for y in range(-8, 13) for z in range(-6, 23)]
    clear_tree_data = occupied_tree([[2., 2., 2.]], free_points=free_points)

    def status(): return json.loads(received['status'][-1].data) if received['status'] else {}

    def pump():
        nonlocal armed, offboard, generation, map_stamp, next_map, next_path, last_pump, command_index, pre_hover_path_pending, map_published_at
        now = time.monotonic()
        if now-last_pump < .02: return
        last_pump = now
        ns = node.get_clock().now().nanoseconds
        for m in received['commands'][command_index:]:
            if m.command == 176: offboard = True
            if m.command == 400 and m.param1 == 1: armed = True
        command_index = len(received['commands'])
        if received['setpoints']:
            target = received['setpoints'][-1].position
            position[:] = [float(x)+external_offset[i] for i,x in enumerate(target)]
            if auto_arm and status().get('state')=='YAW_ALIGN':position[0]+=.003
        local = VehicleLocalPosition(); local.timestamp = ns//1000
        local.xy_valid = local.z_valid = True
        local.x,local.y,local.z = position
        local.v_xy_valid = True
        local.vx,local.vy = drift_velocity
        heading=float(received['setpoints'][-1].yaw) if received['setpoints'] else (0. if auto_arm else math.pi/2)
        local.heading = heading
        local.xy_reset_counter = 1 if fault=='px4_reset' else 0
        pub['/fmu/out/vehicle_local_position_v1'].publish(local)
        mode = VehicleControlMode();mode.timestamp = ns//1000;mode.flag_armed = armed
        mode.flag_control_offboard_enabled = offboard
        if fault != 'mode': pub['/fmu/out/vehicle_control_mode'].publish(mode)
        vehicle_status = VehicleStatus();vehicle_status.timestamp = ns//1000
        vehicle_status.pre_flight_checks_pass = preflight_ready
        pub['/fmu/out/vehicle_status_v1'].publish(vehicle_status)
        body = [.1+position[1]-ground[1], .1+position[0]-ground[0], .2+ground[2]-position[2]]
        pose = PoseStamped();pose.header.stamp.sec,pose.header.stamp.nanosec = divmod(ns,10**9)
        pose.header.frame_id = 'rtabmap_odom';pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = body
        pose.pose.orientation.z = math.sin((math.pi/2-heading)/2)
        pose.pose.orientation.w = math.cos((math.pi/2-heading)/2)
        pub['/rtabmap/vio_pose'].publish(pose)
        ev = VehicleOdometry();ev.timestamp=ns//1000;ev.timestamp_sample=ns//1000
        ev.pose_frame=VehicleOdometry.POSE_FRAME_NED;ev.position=position;ev.q=[math.cos(heading/2),0.,0.,math.sin(heading/2)]
        ev.reset_counter=1 if fault=='ev_reset' else 0
        pub['/fmu/in/vehicle_visual_odometry'].publish(ev)
        info=OdomInfo();info.header.stamp=pose.header.stamp;info.inliers=100;info.lost=False
        if fault != 'lost_info': pub['/rtabmap/vio_odom_info_lite'].publish(info)
        if now>=next_map:
            generation += 1;map_stamp=ns;next_map=now+1.;map_published_at=now
            tree=Octomap();tree.header.frame_id='world';tree.header.stamp=pose.header.stamp
            tree.id='OcTree';tree.binary=True;tree.resolution=.05
            tree.data=(occupied_tree([obstacle], free_points=free_points)
                       if obstacle else clear_tree_data)
            metadata={'generation':generation,'stamp_ns':map_stamp,'frame_id':'world','resolution':.05,
                      'map_from_odom':[0,0,0,0,0,0,1]}
            map_history[generation] = (map_stamp, metadata['map_from_odom'])
            pub['/rtabmap/octomap'].publish(tree);record('/rtabmap/octomap',tree)
            metadata_msg=String(data=json.dumps(metadata))
            pub['/rtabmap/octomap_metadata'].publish(metadata_msg);record('/rtabmap/octomap_metadata',metadata_msg)
        correction=[.1 if fault=='correction' else 0,0,0,0,0,0,1]
        mapped=[body[0]+correction[0],body[1],body[2]]
        # The real pose relay pairs with a new map ~0.1 s after it is published.
        pose_generation, pose_map_stamp = generation, map_stamp
        if pose_lag and now-map_published_at < .15 and generation-1 in map_history:
            pose_generation = generation-1; pose_map_stamp = map_history[pose_generation][0]
        snapshot={'stamp_ns':ns,'frame_id':'world','odom_frame_id':'rtabmap_odom',
                  'map_generation':pose_generation,'map_stamp_ns':pose_map_stamp,'raw_pose':body+[0,0,pose.pose.orientation.z,pose.pose.orientation.w],
                  'pose':mapped+[0,0,pose.pose.orientation.z,pose.pose.orientation.w],'correction':correction}
        if fault != 'pose': pub['/planner3d/pose_snapshot'].publish(String(data=json.dumps(snapshot)))
        if goal is not None and now>=next_path:
            # Publish the requested goal once before takeoff, then withhold its
            # next route snapshot until the test explicitly releases it after
            # hover. This proves the 2D-style policy accepts the same goal only
            # after a fresh post-hover replan, without requiring another click.
            if pre_hover_path_pending:
                # The path topic is intentionally volatile, so repeat until the
                # adapter confirms receipt; its subscription may still be joining
                # DDS when the first mock publication occurs.
                next_path=now+.1
                if status().get('goal_generation') == goal_generation and not status().get('hover_ready'):
                    pre_hover_path_pending=False
                    next_path=float('inf')
            else:
                next_path=now+.5
            path_generation=max(1,generation-path_generation_lag)
            path_map_stamp,_=map_history.get(path_generation,(map_stamp,None))
            reference={'stamp_ns':ns,'frame_id':'wrong' if fault=='path_frame' else 'world',
                       'map_generation':path_generation,'map_stamp_ns':path_map_stamp,'goal_generation':goal_generation,
                       'lethal_radius':.35,'executable':True, 'goal_exact':True,
                       'goal_terminal':True, 'points':[body,goal]}
            pub['/planner3d/path_snapshot'].publish(String(data=json.dumps(reference)))

    def wait(predicate, seconds=10.):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            pump();executor.spin_once(timeout_sec=.01)
            if predicate(): return
            assert process.poll() is None, (tmp_path/'adapter.log').read_text()
        raise AssertionError(f'{status()}\n{(tmp_path/"adapter.log").read_text()[-4000:]}')

    try:
        # With no RC/GCS, PX4 may not make its aggregate preflight result true
        # until Offboard has been requested. The adapter must therefore enter
        # ENGAGE and request the mode, while still withholding ARM.
        wait(lambda: offboard and status().get('fault') == 'PX4_PREFLIGHT_CHECKS' and
             status().get('flight_state') == 'ENGAGE')
        assert not any(m.command == 400 and m.param1 == 1 for m in received['commands'])
        preflight_ready = True
        wait(lambda:status().get('state')=='WAITING_FOR_GOAL')
        hover_status = status()
        assert hover_status['flight_state'] == 'WAITING_FOR_ROUTE'
        assert hover_status['hover_ready'] is True
        assert hover_status['minimum_goal_generation'] == goal_generation
        assert hover_status['post_hover_path'] is False
        assert abs(position[2]-(ground[2]-.3)) < .08
        # Takeoff must use the same vertical velocity feedforward as the proven
        # 2D adapter, while the route retains its independent 3D limits.
        climb_vz=[float(m.velocity[2]) for m in received['setpoints']
                  if math.isfinite(float(m.velocity[2])) and float(m.velocity[2]) < 0.]
        assert climb_vz and min(climb_vz) <= -.24
        wait(lambda:math.isfinite(float(received['setpoints'][-1].velocity[2])) and
             abs(float(received['setpoints'][-1].velocity[2])) < 1e-3,seconds=2.)
        assert abs(float(received['setpoints'][-1].velocity[2])) < 1e-3
        # Hover readiness is a one-way takeoff latch. An altitude overshoot
        # while waiting for the post-hover route must not re-enter CLIMB
        # and later fire the old climb timeout.
        external_offset[2] = -.12
        until=time.monotonic()+.15
        wait(lambda:time.monotonic()>=until,seconds=.4)
        assert status().get('state')=='WAITING_FOR_GOAL'
        external_offset[2] = 0.
        # While hovering for the route: a pose relay that lags each new map by 0.15 s must not fault: the
        # adapter adopts a map only together with a matching pose snapshot.
        pose_lag=True;first_status=len(received['status'])
        until=time.monotonic()+2.3
        wait(lambda:time.monotonic()>=until,seconds=2.8)
        lagged=[json.loads(m.data) for m in received['status'][first_status:]]
        assert lagged and not any(s.get('fault')=='POSE_MAP_MISMATCH' for s in lagged)
        assert max(s['map_generation'] for s in lagged)>min(s['map_generation'] for s in lagged)
        pose_lag=False
        # A transient input fault must freeze the last safe command, not adopt
        # the disturbed vehicle position. Alternating fault/recovery did the
        # latter in flight and ratcheted the takeoff hold by 0.51 m.
        before_fault = [float(x) for x in received['setpoints'][-1].position]
        external_offset[0] = .12
        fault = 'pose'
        wait(lambda:status().get('state')=='HOLD' and status().get('fault')=='POSE_STALE',seconds=1.3)
        held = [float(x) for x in received['setpoints'][-1].position]
        assert math.hypot(held[0]-before_fault[0],held[1]-before_fault[1]) < .01
        until=time.monotonic()+.15
        wait(lambda:time.monotonic()>=until,seconds=.4)
        assert math.dist([float(x) for x in received['setpoints'][-1].position],held) < 1e-6
        external_offset[0] = 0.
        fault = None;next_map=0.
        wait(lambda:status().get('state')=='WAITING_FOR_GOAL')
        with pytest.raises(RuntimeError, match='duplicate flight_authority'):
            ProcessSingleton('flight_authority',domain_id=domain)
        duplicate=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,
            env={**os.environ,'ROS_DOMAIN_ID':str(domain)})
        wait(lambda:duplicate.poll() is not None,seconds=10.)
        _, duplicate_error=duplicate.communicate()
        assert duplicate.returncode != 0 and 'another flight adapter' in duplicate_error
        # Release a new route snapshot for the already-requested preflight goal.
        # No second goal generation/click is needed under the default policy.
        # Reproduce the flown load: every route trails the newest OctoMap by one
        # generation. It may run only after odom-frame reprojection and complete
        # swept-sphere validation against the current tree.
        path_generation_lag=1;next_map=0.;next_path=0.
        wait(lambda:status().get('state')=='FOLLOWING')
        assert status()['post_hover_path'] is True
        assert status()['path_map_generation'] < status()['map_generation']
        assert status()['path_validated_map_generation'] == status()['map_generation']
        wait(lambda:position[1]-ground[1]>.08 and ground[2]-position[2]>.32)
        assert not any(m.command==400 and m.param1==1 for m in received['commands']) if not auto_arm else armed
        if auto_arm:
            # Heading follows the route, not the body: a lateral offset must
            # not turn the nose toward the lookahead (~18 deg for 8 cm).
            wait(lambda:status().get('state')=='FOLLOWING')
            next_path=float('inf')
            yaw_before=float(received['setpoints'][-1].yaw)
            external_offset[0]=.08
            until=time.monotonic()+.4
            wait(lambda:time.monotonic()>=until,seconds=.7)
            assert status().get('state')=='FOLLOWING'
            assert abs(math.remainder(float(received['setpoints'][-1].yaw)-yaw_before,2*math.pi))<math.radians(2.)
            external_offset[0]=0.;next_path=0.
            wait(lambda:status().get('state')=='FOLLOWING')
        # Following feeds the limiter velocity forward, bounded by command_speed.
        wait(lambda:status().get('state')=='FOLLOWING' and
             math.isfinite(float(received['setpoints'][-1].velocity[0])),seconds=2.)
        feedforward=[float(x) for x in received['setpoints'][-1].velocity]
        assert all(math.isfinite(x) for x in feedforward)
        assert math.hypot(feedforward[0],feedforward[1])<=.3*1.01
        before=position[:]
        for injected,expected in [('pose','POSE_STALE'),('correction','POSE_MAP_MISMATCH'),('path_frame','PATH_STALE')]:
            fault=injected;next_path=0.
            wait(lambda:status().get('state')=='HOLD' and status().get('fault')==expected,seconds=1.3)
            assert status()['follower_latched'] is True
            assert status()['follower_latch_reason'] == expected
            held=[float(x) for x in received['setpoints'][-1].position]
            until=time.monotonic()+.15
            wait(lambda:time.monotonic()>=until,seconds=.4)
            assert math.dist([float(x) for x in received['setpoints'][-1].position],held)<1e-6
            # Holds are pure position control: no velocity feedforward.
            assert math.isnan(float(received['setpoints'][-1].velocity[0]))
            fault=None;next_path=0.
            wait(lambda:status().get('state')=='FOLLOWING')
            assert status()['follower_latched'] is False
        assert status()['equivalent_replans_retained'] > 0
        # The observed 0.66 s control-mode gap must not trip the stricter
        # 0.50 s local-position watchdog. The independently configured 0.75 s
        # mode watchdog still freezes the command if the gap persists.
        fault='mode'
        until=time.monotonic()+.60
        wait(lambda:time.monotonic()>=until,seconds=.9)
        assert not (status().get('state')=='HOLD' and
                    status().get('fault')=='PX4_STATE_STALE_OR_INVALID')
        wait(lambda:status().get('state')=='HOLD' and
             status().get('fault')=='PX4_STATE_STALE_OR_INVALID',seconds=.5)
        assert status()['follower_latched'] is True
        fault=None
        wait(lambda:status().get('state')=='FOLLOWING')
        obstacle=[.1+position[1]-ground[1]+.15,.1+position[0]-ground[0],.2+ground[2]-position[2]]
        # Sliding east (map +x) at 0.2 m/s, towards the obstacle.
        drift_velocity[:]=[0.,.2]
        next_map=0.;next_path=0.
        wait(lambda:status().get('state')=='HOLD' and ('OBSTACLE' in status().get('fault','') or 'BLOCKED' in status().get('fault','')))
        # Held 0.125 m from the voxel (inside required_clearance): escape
        # steps the command away from it (map -x = NED -y) instead of pinning.
        # Escape only moves an armed vehicle; the disarmed mock just holds.
        hold_start=list(status()['command_ned'])
        if armed:
            wait(lambda:status().get('escape',{}).get('escaping') is True and
                 status()['command_ned'][1]<hold_start[1]-.03,seconds=2.)
            escape=status()['escape']
            assert escape['clearance']<.35 and escape['travel']<=.5+1e-6
            # A slide towards the obstacle gets a braking lead (0.2 m/s x 0.5 s)
            # and damping stays active against the escape's own velocity.
            assert abs(escape['lead']-.10)<.02
            assert status()['damping_reason']=='braking_or_hold'
        drift_velocity[:]=[0.,0.]
        if armed:
            assert abs(status()['command_ned'][0]-hold_start[0])<.03
        else:
            until=time.monotonic()+.3
            wait(lambda:time.monotonic()>=until,seconds=.6)
            assert status()['escape']['escaping'] is False
            assert math.dist(status()['command_ned'],hold_start)<1e-6
        obstacle=None;next_map=0.;next_path=0.
        wait(lambda:status().get('state')=='FOLLOWING')
        if auto_arm:
            # A fault hold freezes heading as well as position. Reverse the
            # route so YAW_ALIGN starts a long ramp, then fault mid-turn: the
            # nose must stop, not finish turning toward the failed route.
            route_goal=goal[:]
            goal[:]=[.1,.1,.65];next_path=0.
            wait(lambda:status().get('state')=='YAW_ALIGN')
            fault='correction'
            wait(lambda:status().get('state')=='HOLD' and status().get('fault')=='POSE_MAP_MISMATCH',seconds=1.3)
            held_yaw=float(received['setpoints'][-1].yaw)
            until=time.monotonic()+.4
            wait(lambda:time.monotonic()>=until,seconds=.7)
            assert status().get('state')=='HOLD'
            assert abs(float(received['setpoints'][-1].yaw)-held_yaw)<1e-6
            # Opt-in hold damping opposes measured drift, bounded by its 0.10
            # m/s2 limit, without moving the frozen position command.
            held=[float(x) for x in received['setpoints'][-1].position]
            drift_velocity[:]=[.12,-.06]
            until=time.monotonic()+.5
            wait(lambda:time.monotonic()>=until,seconds=.8)
            braking=[float(x) for x in received['setpoints'][-1].acceleration]
            assert status().get('state')=='HOLD' and status()['damping_reason']=='braking_or_hold'
            assert braking[0]<-.05 and braking[1]>.02 and math.hypot(braking[0],braking[1])<=.10+1e-6
            assert math.dist([float(x) for x in received['setpoints'][-1].position],held)<1e-6
            drift_velocity[:]=[0.,0.]
            fault=None;goal[:]=route_goal;next_path=0.
            wait(lambda:status().get('state')=='FOLLOWING')
            # Route following stays pure position control.
            wait(lambda:status().get('state')=='FOLLOWING' and
                 math.isnan(float(received['setpoints'][-1].acceleration[0])))
        # A non-fatal fault may inhibit arming indefinitely, but none of that
        # pre-arm time may consume the armed LAND grace period.
        fault=terminal_fault
        if not auto_arm:
            wait(lambda:status().get('state')=='HOLD' and status().get('fault')=='VIO_HEALTH',seconds=1.3)
            until=time.monotonic()+1.7
            wait(lambda:time.monotonic()>=until,seconds=2.)
            land_commands=sum(m.command==21 for m in received['commands'])
            armed=True
            until=time.monotonic()+.3
            wait(lambda:time.monotonic()>=until,seconds=.6)
            assert sum(m.command==21 for m in received['commands']) == land_commands
        else:
            armed=True
        wait(lambda:any(m.command==21 for m in received['commands']),seconds=4.)
        if auto_arm:
            states=[json.loads(m.data) for m in received['status']]
            assert any(s['state']=='YAW_ALIGN' for s in states)
            for a,b in zip(states,states[1:]):
                if a['state']==b['state']=='YAW_ALIGN':
                    assert math.dist(a['command_ned'],b['command_ned'])<1e-6
        assert not any(name.startswith('/fmu/in/') for name,_ in node.get_topic_names_and_types())
        (tmp_path/'report.json').write_text(json.dumps({'auto_arm':auto_arm,'terminal_fault':terminal_fault,'passed':True,
            'setpoints':len(received['setpoints']),'commands':[int(m.command) for m in received['commands']],
            'position_before_faults':before,'final_position':position,'status':status()},indent=2))
        writer=None
        audit_script=Path(__file__).resolve().parents[4]/'scripts/audit_flight_3d_bag.py'
        result=subprocess.run(['python3',str(audit_script),str(tmp_path/'bag'),'--out',str(tmp_path/'audit.json')],
                              capture_output=True,text=True,timeout=30)
        assert result.returncode==0,result.stdout+result.stderr
        assert json.loads((tmp_path/'audit.json').read_text())['counts']['unverified_segments']==0
        armed=False
        # The adapter may reach DONE immediately after the mock disarms. Do not
        # require the process to remain alive merely to give callbacks time to
        # drain during test cleanup.
        until=time.monotonic()+.15
        while time.monotonic()<until and process.poll() is None:
            pump();executor.spin_once(timeout_sec=.01)
    finally:
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
        log.close();executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)
