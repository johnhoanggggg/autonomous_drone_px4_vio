import sys, json, pathlib
sys.path.insert(0, str(pathlib.Path.cwd()/'scripts'))
from analyze_yaw_tracking import read
from rosidl_runtime_py.convert import message_to_ordereddict
import yaml
root=pathlib.Path('ros2_ws/flight_logs')
meta=yaml.safe_load((root/'offboard_global_20260920T125348Z/metadata.yaml').read_text())
topics=[x['topic_metadata']['name'] for x in meta['rosbag2_bagfile_information']['topics_with_message_count'] if x['topic_metadata']['name'].startswith(('/planner/','/fmu/','/rtabmap/','/rosout','/perf/','/waypoint/','/vio/'))]
for suffix in ['125348','125424','125509']:
    data,t0=read(str(root/f'offboard_global_20260920T{suffix}Z'),topics)
    out={k:[[(t-t0)*1e-9,message_to_ordereddict(m)] for t,m in v] for k,v in data.items() if not k.startswith('_')}
    pathlib.Path(f'reports/tunnel_flights_20260920_1253/{suffix}.json').write_text(json.dumps(out))
    with open(f'reports/tunnel_flights_20260920_1253/{suffix}_events.txt','w') as f:
        for k in ['/planner/flight/config','/planner/follower/config','/planner/config','/planner/flight/status','/rosout','/fmu/in/vehicle_command','/fmu/out/vehicle_command_ack']:
            f.write('\n'+k+'\n')
            for t,m in out.get(k,[]):
                if k=='/rosout' and not (m['level']>=30 or 'offboard_global' in m['name']): continue
                f.write(f'{t:.3f} '+str(m.get('data',m.get('msg',m)))+'\n')
    pathlib.Path(f'reports/tunnel_flights_20260920_1253/{suffix}_decode_errors.json').write_text(json.dumps(data['_decode_errors'],indent=2))
    print(suffix, 'duration',max(v[-1][0] for v in out.values() if v),'decode errors',data['_decode_errors'])
