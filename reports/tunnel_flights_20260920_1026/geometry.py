import json,math,pathlib,bisect,subprocess
import numpy as np
R=pathlib.Path(__file__).parent
p=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def latest(seq,t):return seq[max(0,bisect.bisect_right([a for a,b in seq],t)-1)][1]
def stamp(m):return m['header']['stamp']['sec']*1000000000+m['header']['stamp']['nanosec']
def yaw(m):
 q=m['pose']['orientation'];return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
def sd(v):return v*.25+.5*.3*.25**2+(v+.3*.25)**2/(2*.1)
results={}
for s,times in {'102622':[15.47,18.756,18.86,18.96,19.06,19.16,19.26,19.36,19.8,35.86]}.items():
 d=json.loads((R/f'{s}.json').read_text());out=[]
 for t in times:
  corrections={stamp(m):m for a,m in d['/rtabmap/odom_correction'] if a<=t}
  pairs=[m for a,m in d['/rtabmap/body_pose'] if a<=t and stamp(m) in corrections];body=pairs[-1];cor=corrections[stamp(body)]
  pos=latest(d['/fmu/out/vehicle_local_position_v1'],t);disp=latest(d['/planner/follower/vio_displacement'],t)['vector'];g=latest(d['/rtabmap/grid'],t)
  start=np.array([body['pose']['position'][k] for k in ['x','y']]);ang=yaw(cor);rot=np.array([[math.cos(ang),-math.sin(ang)],[math.sin(ang),math.cos(ang)]])
  measured=rot@np.array([pos['vy'],pos['vx']]);desired=rot@np.array([disp['x'],disp['y']]);v=np.linalg.norm(measured)
  queries=[[*start,*start]];speeds=np.linspace(0,.2,201)
  for direction in [measured,desired]:
   direction=direction/np.linalg.norm(direction)
   for c in speeds:queries.append([*start,*(start+direction*sd(c))])
  queries.append([*start,*(start+measured/max(v,1e-12)*sd(max(0,v-.01)))])
  queries.append([*start,*(start+measured/max(v,1e-12)*(v*v/.2))])
  residual=max(0,v-.01)
  queries.append([*start,*(start+measured/max(v,1e-12)*(residual*.25+residual*residual/.2))])
  p.stdin.write(json.dumps({'grid':g,'queries':queries})+'\n');p.stdin.flush();vals=json.loads(p.stdout.readline());threshold=min(.38,vals[0])
  caps=[]
  for k in [0,1]:
   good=[c for c,x in zip(speeds,vals[1+k*201:1+(k+1)*201]) if x is not None and x+1e-6>=threshold]
   caps.append(float(max(good)) if good else 0)
  out.append({'t':t,'speed':v,'body_clearance':vals[0],'measured_direction_cap':caps[0],'desired_direction_cap':caps[1],'actual_measured_governor_clearance':vals[-3],'actual_active_stop_clearance_no_response':vals[-2],'settled_recovery_clearance_with_response':vals[-1],'threshold':threshold})
 results[s]=out
p.stdin.close();assert p.wait()==0
(R/'geometry.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
