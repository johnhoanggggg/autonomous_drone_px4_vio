#!/usr/bin/env python3
"""Audit exact recorded 3D PX4 commands under their explicit unknown policy."""
import argparse
import json
import math
from pathlib import Path
import subprocess
from collections import Counter


def audit(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from ament_index_python.packages import get_package_prefix
    selected = {'/rtabmap/octomap', '/rtabmap/octomap_metadata',
                '/planner3d/flight/status', '/planner3d/flight/config', '/fmu/in/trajectory_setpoint'}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    kinds = {t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in selected}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(selected)))
    maps, metadata, setpoints, statuses, configs = {}, {}, {}, [], []
    while reader.has_next():
        topic,data,ns = reader.read_next();msg = deserialize_message(data,kinds[topic])
        if topic == '/rtabmap/octomap':
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec;maps[stamp]=msg
        elif topic == '/fmu/in/trajectory_setpoint':setpoints[msg.timestamp]=[float(v) for v in msg.position]
        else:
            value=json.loads(msg.data)
            if topic.endswith('octomap_metadata'):metadata[value['stamp_ns']]=value
            elif topic.endswith('/config'):configs.append((ns,value))
            else:statuses.append((ns,value))
    binary=Path(get_package_prefix('px4_vio_bridge'))/'lib/px4_vio_bridge/octomap_path_audit'
    proc=subprocess.Popen([str(binary)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    errors=[];counts=Counter();last_map=None;artificial=False

    def send(record):
        proc.stdin.write(json.dumps(record)+'\n');proc.stdin.flush()
        outcome=json.loads(proc.stdout.readline())
        if not outcome['valid']:errors.append({'record':record.get('kind'),'generation':record['generation'],**outcome})
        counts['unverified_segments']+=outcome.get('unverified_flight_segments',0)

    try:
        for ns,value in statuses:
            counts['statuses']+=1
            if not value.get('command_clearance_checked',False):continue
            cfg=next((c for t,c in reversed(configs) if t<=ns),None)
            if cfg is None or cfg['allow_unknown'] != value['allow_unknown']:
                errors.append({'error':'missing/mismatched flight policy'});continue
            stamp=value['map_stamp_ns'];meta=metadata.get(stamp);tree=maps.get(stamp)
            if not meta or tree is None or meta['generation']!=value['map_generation']:
                errors.append({'error':'missing exact command map'});continue
            if value['pose_map_generation']!=meta['generation']:
                errors.append({'error':'command pose/map mismatch'});continue
            if value['state']=='FOLLOWING':
                validated_generation=value.get('path_validated_map_generation',value['path_map_generation'])
                validated_stamp=value.get('path_validated_map_stamp_ns',value['map_stamp_ns'])
                if validated_generation!=meta['generation'] or validated_stamp!=stamp:
                    errors.append({'error':'command path validation/map mismatch'});continue
                if (value['path_map_generation']!=meta['generation'] and
                    cfg.get('route_map_policy')!='odom_retained_revalidated_each_map'):
                    errors.append({'error':'lagged path lacks explicit revalidation policy'});continue
            if abs(value['required_clearance']-cfg['required_clearance'])>1e-9:
                errors.append({'error':'command clearance/config mismatch'});continue
            actual=setpoints.get(value['setpoint_timestamp_us'])
            if actual is None or actual != value['command_ned']:
                errors.append({'error':'status does not match emitted PX4 setpoint','timestamp_us':value['setpoint_timestamp_us'],'actual':actual,'status':value['command_ned'],'state':value['state']});continue
            origin=value['ground_ned'];height=origin[2]-actual[2]
            # Since 2026-09-28 the adapter judges height in the map frame above
            # the ground latched at arm (PX4's VIO frame can be tilted).
            if 'ground_map_z' in value:height=value['command_map'][2]-value['ground_map_z']
            low=cfg['min_height'] if value['flight_state']=='ROUTE' else -.05
            if math.hypot(actual[0]-origin[0],actual[1]-origin[1])>cfg['geofence_radius']-cfg['required_clearance']+1e-6 or not low-1e-6<=height<=cfg['max_height']+1e-6:
                errors.append({'error':'command exceeds configured flight bounds'});continue
            # Independently invert NED + configured yaw offset, then apply the
            # full quaternion. Verify the point checked is the point emitted.
            delta=[a-b for a,b in zip(actual,value['command_origin_ned'])]
            theta=math.radians(cfg['vio_yaw_offset_deg']);c,s=math.cos(theta),math.sin(theta)
            x,y,z=c*delta[1]+s*delta[0],-s*delta[1]+c*delta[0],-delta[2]
            qx,qy,qz,qw=value['map_from_odom'][3:]
            tx,ty,tz=2*(qy*z-qz*y),2*(qz*x-qx*z),2*(qx*y-qy*x)
            rotated=[x+qw*tx+qy*tz-qz*ty,y+qw*ty+qz*tx-qx*tz,z+qw*tz+qx*ty-qy*tx]
            mapped=[a+b for a,b in zip(value['body_map'],rotated)]
            if math.dist(mapped,value['command_map'])>1e-6:
                errors.append({'error':'PX4 command/frame conversion mismatch'});continue
            artificial |= meta.get('test_start_clear',{}).get('enabled',False)
            if stamp!=last_map:
                send({'kind':'map','generation':meta['generation'],'stamp_ns':stamp,'id':tree.id,
                      'binary':tree.binary,'resolution':tree.resolution,'data':list(tree.data)})
                last_map=stamp;counts['maps']+=1
            for start in [value['body_map'],value['previous_command_map']]:
                send({'kind':'flight_commands','generation':meta['generation'],'stamp_ns':stamp,
                      'allow_unknown':value['allow_unknown'],'radius':value['required_clearance'],
                      'points':[start,mapped]})
                counts['chords']+=1
            counts['commands']+=1
    finally:
        proc.stdin.close();proc.wait(timeout=10);proc.stdout.close()
    return {'valid':bool(counts['commands']) and not errors,'counts':dict(counts),'errors':errors,
            'uses_artificial_start_clear':artificial,
            'policy':'unknown allowed only when recorded flight config explicitly permits it',
            'physical_clearance_verified':not artificial and not counts['unverified_segments'] and not errors and bool(counts['commands'])}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag',type=Path);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    if args.out.exists():p.error('output already exists')
    result=audit(args.bag);args.out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['valid'] else 1)


if __name__=='__main__':main()
