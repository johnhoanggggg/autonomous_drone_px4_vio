from pathlib import Path
import bisect,json,math,subprocess
import numpy as np
R=Path('/home/john/autonomous_drone_px4_vio');bin=R/'ros2_ws/build/px4_vio_bridge/planner_flight_replay'
config=dict(max_speed=.2,max_acceleration=.3,max_projection_error=.05,corner_tolerance=.05,max_entry_error=.3,max_connector_error=.2,suffix_tolerance=.03,corner_blending=True,junction_deviation=.05)
out=[]
for stamp in ['125631','125730']:
 b=json.loads((R/f'ros2_ws/flight_logs/offboard_global_20260913T{stamp}Z/render/bag_data.json').read_text());steps=[];times=[];accepted=None
 def at(k,t):
  rows=b[k];i=bisect.bisect_right([r['t'] for r in rows],t)-1;return rows[max(0,i)]
 for sp in b['/fmu/in/trajectory_setpoint']:
  t=sp['t'];st=at('/planner/flight/status',t)['text'].split()[0]
  if not 18<=t<=29 or st not in ['ROUTE','DEGRADED','CORNER_HOLD','YAW_ALIGN']:continue
  p=at('/rtabmap/body_pose',t);lp=at('/fmu/out/vehicle_local_position_v1',t);co=at('/rtabmap/odom_correction',t);route=at('/planner/path',t)
  if accepted is None or st not in ['DEGRADED','YAW_ALIGN']:accepted=route
  yaw=math.atan2(2*(co['qw']*co['qz']+co['qx']*co['qy']),1-2*(co['qy']**2+co['qz']**2));c,s=math.cos(yaw),math.sin(yaw);vx,vy=sp['position'][1]-lp['y'],sp['position'][0]-lp['x']
  steps.append(dict(op='heading_sample',command=[p['x']+c*vx-s*vy,p['y']+s*vx+c*vy],points=accepted['xy'],measured_heading=lp['heading'],correction_yaw=yaw,command_speed=.2,dt=.05));times.append(t)
 for look in [.35,.5,.7]:
  scenario=dict(limiter=config,initial_yaw=math.radians(160),heading_lookahead=look,heading_rate=math.radians(20),steps=steps)
  ans=json.loads(subprocess.run([str(bin)],input=json.dumps(scenario),capture_output=True,text=True,check=True).stdout)
  v=np.unwrap([a['target'] for a in ans]);out.append(dict(flight=stamp,lookahead=look,target_travel_deg=float(np.degrees(abs(np.diff(v)).sum())),range_deg=np.degrees([v.min(),v.max()]).tolist(),hold_samples=sum(a['yaw_holding'] for a in ans)))
print(json.dumps(out,indent=2));(R/'reports/new_onboard_20260913/heading_shadow.json').write_text(json.dumps(out,indent=2))
