import json, math, bisect, subprocess
from pathlib import Path
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle
R=Path(__file__).parent
LP='/fmu/out/vehicle_local_position_v1'; SP='/fmu/in/trajectory_setpoint'
COLORS={'ROUTE':'#246ce0','ACTIVE_RECOVERY':'#009981','RESERVE_BRAKE':'#dc851c','CORRECTION_BRAKE':'#aa56b7','YAW_ALIGN':'#909097','HOLD':'#795548'}
def xy(m):return np.array([m['x'],m['y']],float)
def pose(m):return np.array([m['pose']['position'][k] for k in ['x','y']],float)
def stamp(m):return m['header']['stamp']['sec']*1000000000+m['header']['stamp']['nanosec']
def yaw(m):
 q=m['pose']['orientation'];return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
def rot(a):return np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
def speed(m):return math.hypot(m['vx'],m['vy'])
def sd(v,a=.3):return v*.25+.5*a*.25**2+(v+a*.25)**2/.2
class Flight:
 def __init__(self,f):
  self.f=f;self.d=json.loads((R/f'{f}.json').read_text());self.ts={k:[x[0] for x in v] for k,v in self.d.items()}
  self.pairs=[];body={};cor={}
  for t,k,m in sorted([(t,k,m) for k in ['/rtabmap/body_pose','/rtabmap/odom_correction'] for t,m in self.d[k]]):
   s=stamp(m);(body if k.endswith('body_pose') else cor)[s]=m
   if s in body and s in cor:self.pairs.append((t,(body[s],cor[s])))
  self.pair_ts=[t for t,_ in self.pairs]
 def at(self,k,t):
  i=bisect.bisect_right(self.ts[k],t)-1
  return None if i<0 else self.d[k][i][1]
 def pair(self,t):return self.pairs[bisect.bisect_right(self.pair_ts,t)-1][1]
 def frame(self,t):
  b,c=self.pair(t);return pose(b),rot(yaw(c))
 def phases(self):
  events=[];land=None;start=None;kill=None
  for t,m in self.d['/rosout']:
   if m['name']!='cpp_flight_adapter':continue
   s=m['msg'];state=None
   if s=='-> ROUTE':start=t;state='ROUTE'
   elif s.startswith('reserve recovery: starting'):state='ACTIVE_RECOVERY'
   elif 'braking until measured capture' in s:state='CORRECTION_BRAKE' if s.startswith('material correction') else 'RESERVE_BRAKE'
   elif s.startswith('active centring stopped:'):state='CORRECTION_BRAKE' if events[-1][1]=='CORRECTION_BRAKE' else 'RESERVE_BRAKE'
   elif 'rejoining route slowly' in s:state='ROUTE'
   elif s.startswith('YAW_ALIGN '):state='YAW_ALIGN'
   elif s.startswith('HOLD planner fault:'):state='HOLD'
   elif s.startswith('ROUTE valid'):state='ROUTE'
   elif '-> AUTO.LAND' in s:land=t
   elif s.startswith('KILL KEY'):kill=t
   if state is not None and start is not None and (not events or events[-1][1]!=state):events.append([t,state])
  end=land if land is not None else min(self.ts[LP][-1],self.ts[SP][-1])
  return start,end,land,kill,[[a,events[i+1][0] if i+1<len(events) else end,k] for i,(a,k) in enumerate(events) if a<end]
probeproc=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def probe(g,qs):
 probeproc.stdin.write(json.dumps({'grid':g,'queries':qs})+'\n');probeproc.stdin.flush();return json.loads(probeproc.stdout.readline())
metrics={};fig,axs=plt.subplots(4,2,figsize=(17,13),layout='constrained',squeeze=False);maps,mapaxes=plt.subplots(1,2,figsize=(17,9),layout='constrained',squeeze=False)
for col,f in enumerate(['020152','020232']):
 F=Flight(f);d=F.d;start,end,land,kill,phases=F.phases()
 def kind(t):return next((k for a,b,k in phases if a<=t<b),'OUTSIDE')
 out={'route_entry':start,'analysis_end':end,'land':land,'kill':kill,'recording_end':max(x[-1][0] for x in d.values() if x),'intervals':phases,'phase_seconds':{},'goal_reached':any(m['data'] for t,m in d['/planner/follower/goal_reached'] if start<=t<=end),'segments':[]}
 for a,b,k in phases:
  out['phase_seconds'][k]=out['phase_seconds'].get(k,0)+b-a
  samples=[(t,m) for t,m in d[LP] if a<=t<b];vv=[speed(m) for t,m in samples];pp=np.array([xy(m) for t,m in samples]);v0=F.at(LP,a);v1=F.at(LP,b)
  sps=[(t,m) for t,m in d[SP] if a<=t<b];errs=[float(np.linalg.norm(np.array(m['position'][:2])-xy(F.at(LP,t)))) for t,m in sps if math.isfinite(m['position'][0])]
  if not samples:continue
  p0,c0=F.frame(a);p1,c1=F.frame(b)
  row={'kind':k,'start':a,'end':b,'duration':b-a,'speed_entry':speed(v0),'speed_end':speed(v1),'speed_peak':max(vv),'speed_mean':float(np.mean(vv)),'speed_min':min(vv),'ned_delta':(xy(v1)-xy(v0)).tolist(),'net_displacement':float(np.linalg.norm(xy(v1)-xy(v0))),'measured_path_length':float(np.linalg.norm(np.diff(pp,axis=0),axis=1).sum()),'map_delta':(p1-p0).tolist(),'setpoints':len(sps),'velocity_only_setpoints':sum(not math.isfinite(m['position'][0]) for t,m in sps),'nonzero_velocity_setpoints':sum(math.hypot(*m['velocity'][:2])>1e-6 for t,m in sps),'position_lead_max':max(errs,default=None),'position_lead_p95':float(np.percentile(errs,95)) if errs else None}
  out['segments'].append(row)
 moving=[(t,m) for t,m in d[LP] if start<=t<end and kind(t) in ['ROUTE','ACTIVE_RECOVERY']]
 out['moving_speed_peak']=max(speed(m) for t,m in moving);out['net_displacement']=float(np.linalg.norm(xy(F.at(LP,end))-xy(F.at(LP,start))))
 routeerr=[float(np.linalg.norm(np.array(m['position'][:2])-xy(F.at(LP,t)))) for t,m in d[SP] if kind(t)=='ROUTE' and math.isfinite(m['position'][0])]
 out['route_position_error_max']=max(routeerr);out['route_position_error_p95']=float(np.percentile(routeerr,95))
 out['route_cross_track_max']=max(abs(m['data']) for t,m in d['/planner/follower/cross_track'] if kind(t)=='ROUTE')
 out['follower_remaining_at_entry']=F.at('/planner/follower/remaining',start)['data'];out['follower_remaining_at_end']=F.at('/planner/follower/remaining',end)['data']
 out['remaining_note']='Follower path remaining is affected by replanning/goal changes; not fixed-goal progress.'
 clearance=[]
 for t,(body,cor) in F.pairs:
  if not start<=t<=end:continue
  p=pose(body);lp=F.at(LP,t);v=rot(yaw(cor))@np.array([lp['vy'],lp['vx']]);vabs=np.linalg.norm(v);stop=p+v*(.25+vabs/.2);g=F.at('/rtabmap/grid',t)
  vals=probe(g,[[*p,*p],[*p,*stop]])
  clearance.append([t,*vals])
 out['clearance_trace']=clearance
 for row in out['segments']:
  cs=[c for t,c,z in clearance if row['start']<=t<row['end'] and c is not None]
  row['body_clearance_start']=cs[0] if cs else None;row['body_clearance_end']=cs[-1] if cs else None;row['body_clearance_min']=min(cs) if cs else None
 out['setpoint_timing']={}
 for k in ['ROUTE','ACTIVE_RECOVERY','RESERVE_BRAKE','CORRECTION_BRAKE']:
  gaps=[b-a for (a,m),(b,n) in zip(d[SP],d[SP][1:]) if kind(a)==k and kind(b)==k]
  out['setpoint_timing'][k]={'max_gap':max(gaps,default=None),'p95_gap':float(np.percentile(gaps,95)) if gaps else None}
 out['goal_changes']=[];last=None
 for t,m in d.get('/waypoint/clicked',[]):
  p=m.get('point',m)
  if p!=last:out['goal_changes'].append([t,p]);last=p
 metrics[f]=out
 ts=[t for t,m in d[LP]];axs[0,col].plot(ts,[speed(m) for t,m in d[LP]],label='Measured XY speed')
 axs[0,col].plot([t for t,m in d[SP]],[math.hypot(*m['velocity'][:2]) for t,m in d[SP]],label='Commanded XY velocity');axs[0,col].set(ylabel='m/s',title=f'Flight {f[:2]}:{f[2:4]}:{f[4:]} UTC')
 for j,n in enumerate(['N','E']):
  axs[1,col].plot(ts,[m['x' if j==0 else 'y'] for t,m in d[LP]],label=f'Measured {n}')
  axs[1,col].plot([t for t,m in d[SP]],[m['position'][j] for t,m in d[SP]],':',label=f'Target {n}')
 axs[1,col].set(ylabel='PX4 local position (m)')
 axs[2,col].plot([t for t,c,s in clearance],[c for t,c,s in clearance],label='Body clearance');axs[2,col].plot([t for t,c,s in clearance],[s for t,c,s in clearance],label='Coast + stop (.25 s response)')
 axs[2,col].axhline(.28,c='red',ls='--',label='Hard .28 m');axs[2,col].axhline(.38,c='orange',ls='--',label='Reserve .38 m');axs[2,col].set(ylabel='Clearance (m)')
 axs[3,col].plot(ts,[math.degrees(m['heading']) for t,m in d[LP]],label='Measured heading');axs[3,col].plot([t for t,m in d[SP]],[math.degrees(m['yaw']) for t,m in d[SP]],label='Commanded heading');axs[3,col].set(ylabel='NED heading (degrees)',xlabel='Seconds from first selected bag message')
 for ax in axs[:,col]:
  for a,b,k in phases:
   if k!='ROUTE':ax.axvspan(a,b,color=COLORS[k],alpha=.12)
  ax.axvline(end,c='red' if land else '#444',ls='--');ax.set_xlim(start-1,end+1);ax.grid(alpha=.2);ax.legend(fontsize=8)
 # Display final map; use live map per sample only for quantitative clearances.
 ax=mapaxes[0,col];g=F.at('/rtabmap/grid',end);inf=g['info'];res=inf['resolution'];ox=inf['origin']['position']['x'];oy=inf['origin']['position']['y'];grid=np.array(g['data']).reshape(inf['height'],inf['width'])
 ax.imshow(np.where(grid<0,0,np.where(grid>=65,2,1)),origin='lower',extent=[ox,ox+res*inf['width'],oy,oy+res*inf['height']],cmap=ListedColormap(['#d8dce3','#fafafa','#263340']),vmin=0,vmax=2,interpolation='nearest')
 path=np.array([pose(p) for p in F.at('/planner/path',end)['poses']]);ax.plot(*path.T,'--',c='#268c3c',lw=1.4,label='Last planner path')
 seen=set();tracks=[]
 for a,b,k in phases:
  pts=np.array([pose(m) for t,m in d['/rtabmap/body_pose'] if a<=t<b]);tracks.extend(pts)
  if len(pts):ax.plot(*pts.T,c=COLORS[k],lw=2.6,label=k if k not in seen else None);seen.add(k)
  if k in ['ACTIVE_RECOVERY','CORRECTION_BRAKE','RESERVE_BRAKE']:
   p,_=F.frame(a);ax.scatter(*p,c=COLORS[k],s=30,zorder=6);ax.annotate(f'{a:.1f}s',p,xytext=(3,6),textcoords='offset points',fontsize=7)
 targets=[]
 for t,m in d[SP]:
  if not start<=t<=end or not math.isfinite(m['position'][0]):continue
  p,rotation=F.frame(t);lp=F.at(LP,t);targets.append(p+rotation@np.array([m['position'][1]-lp['y'],m['position'][0]-lp['x']]))
 ax.scatter(*np.array(targets).T,c='#bc3497',s=3,label='Position targets (reconstructed)',alpha=.5)
 p,_=F.frame(start);ax.scatter(*p,c='#246ce0',s=70,marker='o');ax.annotate('ROUTE entry',p,xytext=(-5,-15),textcoords='offset points',fontsize=8)
 p,_=F.frame(end);ax.scatter(*p,c='red',s=70,marker='X',zorder=8);ax.add_patch(Circle(p,.28,fill=False,color='red',lw=1.3));ax.annotate('LAND' if land else 'Recording ends\nin correction brake',p,xytext=(8,8),textcoords='offset points',fontsize=8)
 goal=F.at('/planner/effective_goal',end)['point'];ax.scatter(goal['x'],goal['y'],marker='*',s=120,c='#268c3c',label='Last effective goal')
 allpoints=np.array([*tracks,*path,np.array([goal['x'],goal['y']])]);lo=allpoints.min(axis=0)-.45;hi=allpoints.max(axis=0)+.45
 ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='Map x (m)',ylabel='Map y (m)',title=f'{f[:2]}:{f[2:4]}:{f[4:]} — '+('adapter LAND' if land else 'truncated recording'))
 ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(fontsize=7,loc='lower left')
probeproc.stdin.close();assert probeproc.wait()==0
(R/'metrics.json').write_text(json.dumps(metrics,indent=2))
fig.suptitle('Latest flights: active steering (green), reserve stop (orange), correction stop (purple), yaw alignment (gray)')
fig.savefig(R/'flight_performance.png',dpi=160)
maps.suptitle('September 21 — corrected body trajectory on last map\nUnknown gray / free white / obstacles dark; map/localization changes can shift historical tracks.',fontsize=12);maps.savefig(R/'flight_maps.png',dpi=180)
for f,m in metrics.items():
 print(f,json.dumps({k:v for k,v in m.items() if k not in ['clearance_trace','segments']},indent=2))
 for s in m['segments']:print(s)
