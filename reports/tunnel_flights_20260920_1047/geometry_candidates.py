import json,math,pathlib,bisect,subprocess
import numpy as np
R=pathlib.Path(__file__).parent
p=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def latest(seq,t):return seq[max(0,bisect.bisect_right([a for a,b in seq],t)-1)][1]
def stamp(m):return m['header']['stamp']['sec']*1000000000+m['header']['stamp']['nanosec']
def xy(m):return np.array([m['pose']['position'][k] for k in ['x','y']])
def yaw(m):
 q=m['pose']['orientation'];return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
def sd(v,acc=.3):return v*.25+.5*acc*.25**2+(v+acc*.25)**2/(2*.1)
def probe(g,queries):
 p.stdin.write(json.dumps({'grid':g,'queries':queries})+'\n');p.stdin.flush();return json.loads(p.stdout.readline())
results={}
for f,times in {'104759':[11.713,13.417,18.814,20.973,21.163,21.5,22.1], '104844':[10.263,12.967,16.868,19.114,21.96,22.013,22.5,23.5,25.4]}.items():
 d=json.loads((R/f'{f}.json').read_text());out=[]
 for t in times:
  corrections={stamp(m):m for a,m in d['/rtabmap/odom_correction'] if a<=t}
  pairs=[m for a,m in d['/rtabmap/body_pose'] if a<=t and stamp(m) in corrections];body=pairs[-1];cor=corrections[stamp(body)]
  pos=latest(d['/fmu/out/vehicle_local_position_v1'],t);disp=latest(d['/planner/follower/vio_displacement'],t)['vector'];g=latest(d['/rtabmap/grid'],t)
  start=xy(body);ang=yaw(cor);rot=np.array([[math.cos(ang),-math.sin(ang)],[math.sin(ang),math.cos(ang)]])
  measured=rot@np.array([pos['vy'],pos['vx']]);desired=rot@np.array([disp['x'],disp['y']]);v=np.linalg.norm(measured);direction=desired/np.linalg.norm(desired)
  sp=latest(d['/fmu/in/trajectory_setpoint'],t);ff=rot@np.array([sp['velocity'][1],sp['velocity'][0]])
  command=rot@np.array([sp['position'][1]-pos['y'],sp['position'][0]-pos['x']])+start
  qs=[[*start,*start], [*start,*(start+measured/max(v,1e-12)*sd(max(0,v-.01)))], [*start,*(start+measured/max(v,1e-12)*sd(max(0,v-.01),0))], [*start,*(start+measured*(.25+v/.2))], [*start,*(start+direction*sd(.06))], [*start,*(start+direction*.10)], [*(start+direction*.10),*(start+direction*.10)]]
  candidates=[]
  for a in np.linspace(-math.pi,math.pi,72,endpoint=False):
   unit=np.array([math.cos(a),math.sin(a)]);end=start+.10*unit
   candidates.append((unit,end));qs.extend([[*start,*end],[*end,*end],[*start,*(start+unit*sd(.06))]])
  vals=probe(g,qs);base=vals[0];best=None
  for k,(unit,end) in enumerate(candidates):
   chord,ep,st=vals[7+k*3:10+k*3];prog=float(unit@direction)
   if chord is None or ep is None or st is None:continue
   if chord+1e-6<base or ep<base+.01 or st<max(.28,min(.38,base)) or prog<=0:continue
   score=ep-base+.05*prog
   if best is None or score>best['score']:best={'end':end.tolist(),'unit':unit.tolist(),'endpoint_clearance':ep,'chord_clearance':chord,'command_stop_corridor':st,'progress_cosine':prog,'score':score}
  out.append({'t':t,'body_map':start.tolist(),'measured_map_velocity':measured.tolist(),'measured_speed':v,'follower_direction':direction.tolist(),'feedforward_map_velocity':ff.tolist(),'position_target_map':command.tolist(),'yaw_map_deg':math.degrees(pos['heading'])*-1+90+math.degrees(ang),'follower_valid':latest(d['/planner/follower/valid'],t)['data'],'body_clearance':base,'full_measured_governor_clearance':vals[1],'settled_measured_governor_clearance':vals[2],'full_actual_stop_with_response_clearance':vals[3],'follower_command_stop_corridor_at_006':vals[4],'follower_10cm_chord':vals[5],'follower_10cm_endpoint_clearance':vals[6],'best_10cm_progress_candidate':best})
 results[f]=out
p.stdin.close();assert p.wait()==0
(R/'geometry_candidates.json').write_text(json.dumps(results,indent=2))
for f,rows in results.items():
 print(f)
 for x in rows:print('t',round(x['t'],3),'v',round(x['measured_speed'],3),'body',np.round(x['body_map'],3),'motion',np.round(x['measured_map_velocity'],3),'ff',np.round(x['feedforward_map_velocity'],3),'clear',round(x['body_clearance'],3),'route10cmend',x['follower_10cm_endpoint_clearance'],'candidate',x['best_10cm_progress_candidate'])
