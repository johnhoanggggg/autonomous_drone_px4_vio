import json, math, pathlib, bisect
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=pathlib.Path(__file__).parent
TOP='/fmu/out/vehicle_local_position_v1'
def at(seq,t):
 i=bisect.bisect_right([a for a,b in seq],t)-1
 return seq[max(0,i)][1]
def xy(m):return np.array([m['x'],m['y']])
def pose(m):return np.array([m['pose']['position'][k] for k in ['x','y']])
def speed(m):return math.hypot(m['vx'],m['vy'])
results={}
fig, axs=plt.subplots(3,2,figsize=(14,10),layout='constrained')
for col,s in enumerate(['104759','104844']):
 d=json.loads((ROOT/f'{s}.json').read_text()); logs=d['/rosout']; status=d['/planner/flight/status']
 start=next(t for t,m in logs if m['msg']=='-> ROUTE'); land=next(t for t,m in logs if '-> AUTO.LAND' in m['msg'])
 starts=[(t,'correction' if m['msg'].startswith('material correction') else 'reserve') for t,m in logs if 'braking until measured capture' in m['msg']]
 rejoins=[t for t,m in logs if 'rejoining route' in m['msg']]
 intervals=[]; cur=start
 for st,kind in starts:
  intervals.append((cur,st,'route'))
  en=next((t for t in rejoins if t>st),land)
  intervals.append((st,en,kind));cur=en
 if cur<land: intervals.append((cur,land,'route'))
 def route(t):return any(a<=t<b and k=='route' for a,b,k in intervals)
 flight=[(t,m) for t,m in d[TOP] if start<=t<land]
 routepos=[(t,m) for t,m in flight if route(t)]
 ct={}
 for name in ['cross_track','cross_track_filtered']:
  v=[abs(m['data']) for t,m in d['/planner/follower/'+name] if route(t)]
  ct[name]={'mean':float(np.mean(v)),'max':max(v),'p95':float(np.percentile(v,95))}
 out={'route_entry':start,'land':land,'intervals':intervals,'route_seconds':sum(b-a for a,b,k in intervals if k=='route'),'preland_seconds':land-start,'cross_track':ct,'route_speed_mean':np.mean([speed(m) for t,m in routepos]),'route_speed_peak':max(speed(m) for t,m in routepos),'net_displacement':float(np.linalg.norm(xy(flight[-1][1])-xy(flight[0][1]))),'goal_reached':any(m['data'] for t,m in d['/planner/follower/goal_reached'] if start<=t<land),'stops':[]}
 for a,b,k in intervals:
  if k=='route':continue
  samples=[(t,m) for t,m in d[TOP] if a<=t<b]; p0=xy(at(d[TOP],a)); v0=speed(at(d[TOP],a))
  sp=[(t,m) for t,m in d['/fmu/in/trajectory_setpoint'] if a<=t<b]
  tt=np.array([t for t,m in samples]);vv=np.array([speed(m) for t,m in samples]);pp=np.array([xy(m) for t,m in samples])
  corr=np.array([pose(m) for t,m in d['/rtabmap/odom_correction'] if a<=t<b])
  st={'kind':k,'start':a,'end':b,'duration':b-a,'entry_speed':v0,'peak_speed':max(vv),'minimum_speed':min(vv),'end_speed':vv[-1],'first_below_005':float(tt[np.where(vv<=.05)[0][0]]-a) if any(vv<=.05) else None,'max_displacement_from_entry':float(max(np.linalg.norm(pp-p0,axis=1))),'path_length':float(sum(np.linalg.norm(np.diff(pp,axis=0),axis=1))),'correction_translation_range':float(max(np.linalg.norm(corr-corr[0],axis=1))) if len(corr) else 0,'setpoints':len(sp),'nonzero_velocity_setpoints':sum(math.hypot(*m['velocity'][:2])>1e-6 for t,m in sp),'velocity_only_setpoints':sum(not math.isfinite(m['position'][0]) for t,m in sp)}
  modes=[];last=None
  for t,m in sp:
   mode='position_capture' if math.isfinite(m['position'][0]) else 'velocity_only'
   if mode!=last:modes.append([t,mode]);last=mode
  st['setpoint_mode_changes']=modes
  if k=='reserve':
   caps=[]
   import re
   for t,m in status:
    if not a<=t<b:continue
    cap=re.search(r'release_cap=([-.0-9]+)',m['data'])
    if cap and float(cap[1])>=0:caps.append([t,float(cap[1]),speed(at(d[TOP],t))])
   st['release_caps_with_actual_speed']=caps
  out['stops'].append(st)
 # position error and combined velocity demand estimate using historical MPC_XY_P=.95, not onboard demand
 err=[];combined=[]
 for t,m in d['/fmu/in/trajectory_setpoint']:
  if route(t) and math.isfinite(m['position'][0]):
   p=at(d[TOP],t);e=np.array(m['position'][:2])-xy(p);err.append(float(np.linalg.norm(e)));combined.append(float(np.linalg.norm(np.array(m['velocity'][:2])+.95*e)))
 out['route_position_error_p95']=float(np.percentile(err,95));out['route_position_error_max']=max(err);out['estimated_combined_velocity_peak_using_historical_gain']=max(combined)
 results[s]=out
 ts=np.array([t for t,m in d[TOP]]);vs=[speed(m) for t,m in d[TOP]]
 ax=axs[0,col];ax.plot(ts,vs,label='Measured XY speed');ax.plot([t for t,m in d['/fmu/in/trajectory_setpoint']],[math.hypot(*m['velocity'][:2]) for t,m in d['/fmu/in/trajectory_setpoint']],label='Velocity feedforward');ax.set(ylabel='m/s',title=f'Flight {s} UTC')
 ax.legend(fontsize=8)
 ax=axs[1,col]
 for name in ['cross_track','cross_track_filtered']:ax.plot([t for t,m in d['/planner/follower/'+name]],[m['data'] for t,m in d['/planner/follower/'+name]],label=name)
 ax.set(ylabel='Cross-track (m)');ax.legend(fontsize=8)
 ax=axs[2,col];ax.plot(ts,[m['x'] for t,m in d[TOP]],label='Measured N');ax.plot(ts,[m['y'] for t,m in d[TOP]],label='Measured E')
 for j,n in enumerate(['Target N','Target E']):ax.plot([t for t,m in d['/fmu/in/trajectory_setpoint']],[m['position'][j] for t,m in d['/fmu/in/trajectory_setpoint']],':',label=n)
 ax.set(ylabel='PX4 local position (m)',xlabel='Seconds since first selected bag topic');ax.legend(fontsize=8)
 for ax in axs[:,col]:
  for a,b,k in intervals:
   if k!='route':ax.axvspan(a,b,color='orange' if k=='reserve' else 'purple',alpha=.14)
  ax.axvline(land,color='red',linestyle='--');ax.set_xlim(start-1,land+1);ax.grid(alpha=.2)
(ROOT/'metrics.json').write_text(json.dumps(results,indent=2))
fig.savefig(ROOT/'flight_performance.png',dpi=170)
print(json.dumps({s:{k:v for k,v in r.items() if k not in ['stops']} for s,r in results.items()},indent=2))
for s,r in results.items():
 print(s)
 for st in r['stops']:print({k:v for k,v in st.items() if k!='release_caps_with_actual_speed'})
