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
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from octomap_msgs.msg import Octomap
from px4_msgs.msg import VehicleLocalPosition, VehicleControlMode, VehicleOdometry, VehicleCommand, TrajectorySetpoint
from rtabmap_msgs.msg import OdomInfo
from std_msgs.msg import String
from px4_vio_bridge.process_singleton import ProcessSingleton


def occupied_tree(points, resolution=.05):
    """OctoMap binary format: two bits per child, depth-first inner nodes."""
    root = {}
    for point in points:
        key = [int(math.floor(x/resolution))+32768 for x in point]
        node = root
        for bit in range(15, -1, -1):
            index = sum(((key[axis] >> bit) & 1) << axis for axis in range(3))
            if bit == 0:
                node[index] = None
            else:
                node = node.setdefault(index, {})

    def encode(node):
        bits = sum((2 if child is None else 3) << (2*i) for i, child in node.items())
        out = [bits & 255, bits >> 8]
        for i in sorted(node):
            if node[i] is not None:
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
        (VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile_sensor_data),
        (PoseStamped, '/rtabmap/vio_pose', 10), (OdomInfo, '/rtabmap/vio_odom_info', qos_profile_sensor_data),
        (Octomap, '/rtabmap/octomap', durable), (String, '/rtabmap/octomap_metadata', durable),
        (String, '/planner3d/pose_snapshot', 10), (String, '/planner3d/reference_snapshot', 10),
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
    subs = [node.create_subscription(kind, topic, lambda m, key=key: receive(key,m), qos)
            for kind, topic, key, qos in [
                (String, '/planner3d/flight/status', 'status', 10),
                (VehicleCommand, prefix+'/fmu/in/vehicle_command', 'commands', qos_profile_sensor_data),
                (TrajectorySetpoint, prefix+'/fmu/in/trajectory_setpoint', 'setpoints', qos_profile_sensor_data)]]
    subs.append(node.create_subscription(String,'/planner3d/flight/config',lambda m:record('/planner3d/flight/config',m),durable))
    binary = Path(get_package_prefix('px4_vio_bridge'))/'lib/px4_vio_bridge/cpp_flight_adapter_3d'
    args = [str(binary), '--ros-args']
    params = {'auto_arm': auto_arm, 'allow_unknown': True, 'hover_height': .3,
              'min_height': .1, 'max_height': 1.5, 'command_speed': .3,
              'command_vertical_speed': .3, 'command_acceleration': .6,
              'command_vertical_acceleration': .6, 'min_vio_features': 20,
              'stream_time': .2, 'climb_timeout': 8., 'planner_fault_land_time': 1.5,
              'max_flight_time': 90., 'goal_hold_time': 20., 'keyboard_kill': False,
              'keyboard_land': False, 'yaw_follows_heading': auto_arm, 'yaw_rate_deg': 180.}
    for k,v in params.items(): args += ['-p', f'{k}:={str(v).lower()}']
    for topic in ['/fmu/out/vehicle_local_position_v1', '/fmu/out/vehicle_control_mode',
                  '/fmu/in/vehicle_visual_odometry', '/fmu/in/vehicle_command',
                  '/fmu/in/trajectory_setpoint', '/fmu/in/offboard_control_mode', '/fmu/out/sensor_combined']:
        args += ['-r', f'{topic}:={prefix}{topic}']
    log = (tmp_path/'adapter.log').open('w')
    process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
        env={**os.environ, 'ROS_DOMAIN_ID': str(domain)}, start_new_session=True)
    position = [4., -2., 1.]; ground = position[:]
    armed = False; offboard = False; generation = 0; map_stamp = 0; goal_generation = 0
    goal = None; next_map = 0.; next_path = 0.; last_pump = 0.; command_index = 0
    fault = None; obstacle = None

    def status(): return json.loads(received['status'][-1].data) if received['status'] else {}

    def pump():
        nonlocal armed, offboard, generation, map_stamp, next_map, next_path, last_pump, command_index
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
            position[:] = [float(x) for x in target]
            if auto_arm and status().get('state')=='YAW_ALIGN':position[0]+=.003
        local = VehicleLocalPosition(); local.timestamp = ns//1000
        local.xy_valid = local.z_valid = True
        local.x,local.y,local.z = position
        heading=float(received['setpoints'][-1].yaw) if received['setpoints'] else (0. if auto_arm else math.pi/2)
        local.heading = heading
        local.xy_reset_counter = 1 if fault=='px4_reset' else 0
        pub['/fmu/out/vehicle_local_position_v1'].publish(local)
        mode = VehicleControlMode();mode.timestamp = ns//1000;mode.flag_armed = armed
        mode.flag_control_offboard_enabled = offboard
        pub['/fmu/out/vehicle_control_mode'].publish(mode)
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
        if fault != 'lost_info': pub['/rtabmap/vio_odom_info'].publish(info)
        if now>=next_map:
            generation += 1;map_stamp=ns;next_map=now+1.
            tree=Octomap();tree.header.frame_id='world';tree.header.stamp=pose.header.stamp
            tree.id='OcTree';tree.binary=True;tree.resolution=.05
            tree.data=occupied_tree([obstacle or [2.,2.,2.]])
            metadata={'generation':generation,'stamp_ns':map_stamp,'frame_id':'world','resolution':.05,
                      'map_from_odom':[0,0,0,0,0,0,1]}
            pub['/rtabmap/octomap'].publish(tree);record('/rtabmap/octomap',tree)
            metadata_msg=String(data=json.dumps(metadata))
            pub['/rtabmap/octomap_metadata'].publish(metadata_msg);record('/rtabmap/octomap_metadata',metadata_msg)
        correction=[.1 if fault=='correction' else 0,0,0,0,0,0,1]
        mapped=[body[0]+correction[0],body[1],body[2]]
        snapshot={'stamp_ns':ns,'frame_id':'world','odom_frame_id':'rtabmap_odom',
                  'map_generation':generation,'map_stamp_ns':map_stamp,'raw_pose':body+[0,0,pose.pose.orientation.z,pose.pose.orientation.w],
                  'pose':mapped+[0,0,pose.pose.orientation.z,pose.pose.orientation.w],'correction':correction}
        if fault != 'pose': pub['/planner3d/pose_snapshot'].publish(String(data=json.dumps(snapshot)))
        if goal is not None and now>=next_path:
            next_path=now+.5
            reference={'stamp_ns':ns,'frame_id':'wrong' if fault=='path_frame' else 'world',
                       'map_generation':generation,'map_stamp_ns':map_stamp,'goal_generation':goal_generation,
                       'lethal_radius':.35,'unknown_policy':'penalized','executable':False,
                       'points':[body,goal], 'verification':{'goal_exact':True,'state':'PATH_VALID'}}
            pub['/planner3d/reference_snapshot'].publish(String(data=json.dumps(reference)))

    def wait(predicate, seconds=10.):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            pump();executor.spin_once(timeout_sec=.01)
            if predicate(): return
            assert process.poll() is None, (tmp_path/'adapter.log').read_text()
        raise AssertionError(f'{status()}\n{(tmp_path/"adapter.log").read_text()[-4000:]}')

    try:
        wait(lambda:status().get('state')=='WAITING_FOR_GOAL')
        assert abs(position[2]-(ground[2]-.3)) < .08
        with pytest.raises(RuntimeError, match='duplicate flight_authority'):
            ProcessSingleton('flight_authority',domain_id=domain)
        duplicate=subprocess.Popen(args,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,
            env={**os.environ,'ROS_DOMAIN_ID':str(domain)})
        wait(lambda:duplicate.poll() is not None,seconds=10.)
        _, duplicate_error=duplicate.communicate()
        assert duplicate.returncode != 0 and 'another flight adapter' in duplicate_error
        goal_generation=1
        goal=[.7,.1,.65]
        wait(lambda:status().get('state')=='FOLLOWING')
        wait(lambda:position[1]-ground[1]>.08 and ground[2]-position[2]>.32)
        assert not any(m.command==400 and m.param1==1 for m in received['commands']) if not auto_arm else armed
        before=position[:]
        for injected,expected in [('pose','POSE_STALE'),('correction','POSE_MAP_MISMATCH'),('path_frame','PATH_STALE')]:
            fault=injected;next_path=0.
            wait(lambda:status().get('state')=='HOLD' and status().get('fault')==expected,seconds=1.3)
            held=position[:];until=time.monotonic()+.15
            wait(lambda:time.monotonic()>=until,seconds=.4)
            assert math.dist(position,held)<.002
            fault=None;next_path=0.
            wait(lambda:status().get('state')=='FOLLOWING')
        obstacle=[.1+position[1]-ground[1]+.15,.1+position[0]-ground[0],.2+ground[2]-position[2]]
        next_map=0.;next_path=0.
        wait(lambda:status().get('state')=='HOLD' and ('OBSTACLE' in status().get('fault','') or 'BLOCKED' in status().get('fault','')))
        obstacle=None;next_map=0.;next_path=0.
        wait(lambda:status().get('state')=='FOLLOWING')
        # Mock an armed vehicle for the inherited health->LAND watchdog.
        armed=True;fault=terminal_fault
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
        assert json.loads((tmp_path/'audit.json').read_text())['counts']['unverified_segments']>0
        armed=False
        until=time.monotonic()+.15;wait(lambda:time.monotonic()>=until,seconds=.4)
    finally:
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
        log.close();executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)
