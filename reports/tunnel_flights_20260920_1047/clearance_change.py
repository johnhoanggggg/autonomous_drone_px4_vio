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

results=[]
for f,t0,t1 in [('104759',20.973,21.163),('104759',21.163,22.10),('104844',22.013,25.40)]:
 d=json.loads((R/f'{f}.json').read_text());states=[]
 for t in [t0,t1]:
  cs={stamp(m):m for a,m in d['/rtabmap/odom_correction'] if a<=t}
  b=[m for a,m in d['/rtabmap/body_pose'] if a<=t and stamp(m) in cs][-1];c=cs[stamp(b)]
  states.append((b,c,latest(d['/rtabmap/grid'],t),latest(d['/fmu/out/vehicle_local_position_v1'],t)))
 b0,c0,g0,l0=states[0];b1,c1,g1,l1=states[1];p0=xy(b0);p1=xy(b1)
 def rot(m):
  a=yaw(m);return np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
 p1old=rot(c0)@rot(c1).T@(p1-xy(c1))+xy(c0)
 v=probe(g0,[[*p0,*p0],[*p1old,*p1old],[*p1,*p1]])+probe(g1,[[*p1,*p1]])
 results.append({'flight':f,'t0':t0,'t1':t1,'old_clearance':v[0],'after_raw_pose_motion_old_grid_and_correction':v[1],'after_correction_change_old_grid':v[2],'new_clearance':v[3],'raw_pose_displacement':(p1old-p0).tolist(),'correction_translation_delta':(xy(c1)-xy(c0)).tolist(),'px4_xy_displacement':np.array([l1['x']-l0['x'],l1['y']-l0['y']]).tolist()})
p.stdin.close();assert p.wait()==0
(R/'clearance_change.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
