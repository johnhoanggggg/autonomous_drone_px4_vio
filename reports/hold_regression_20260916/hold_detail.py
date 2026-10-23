from pathlib import Path
import json, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]; OUT=Path(__file__).resolve().parent
specs=[('081620',33.55),('081709',30.70),('101553',40.296),('101647',23.067)]
fig,axes=plt.subplots(4,2,figsize=(13,12)); results=[]
for row,(s,land) in enumerate(specs):
 b=json.loads((ROOT/f'ros2_ws/flight_logs/offboard_global_20260916T{s}Z/render/bag_data.json').read_text())
 def series(k,fields):
  r=b[k];return np.array([x['t'] for x in r]),np.array([[x[f] for f in fields] for x in r])
 st,sp=series('/fmu/in/trajectory_setpoint',['position','velocity']);pos=sp[:,0,:2]; ff=sp[:,1,:2]
 lt,lp=series('/fmu/out/vehicle_local_position_v1',['x','y','vx','vy','z']); rt,raw=series('/rtabmap/body_vio_pose',['x','y']);pt,pose=series('/rtabmap/body_pose',['x','y'])
 def near(t,q):return np.argmin(abs(t-q))
 status=b['/planner/flight/status'];phases=[]
 for r in status:
  state=r['text'].split()[0]
  if not phases or phases[-1][1]!=state:phases.append([r['t'],state])
 holds=[]
 for i,(t,state) in enumerate(phases):
  if state!='HOLD':continue
  end=min(phases[i+1][0] if i+1<len(phases) else land,land)
  j=near(st,t); k=near(lt,st[j]); sl=(lt>=st[j])&(lt<end); rl=(rt>=st[j])&(rt<end); ss=(st>=st[j])&(st<end)
  h=dict(start=float(st[j]),end=end,duration=end-st[j],entry_step=float(np.linalg.norm(pos[j]-pos[j-1])),entry_xy_error=float(np.linalg.norm(pos[j]-lp[k,:2])),entry_east_error=float(pos[j,1]-lp[k,1]),entry_east_velocity=float(lp[k,3]),reference_xy_span=np.ptp(pos[ss],axis=0).tolist(),local_east_span=float(np.ptp(lp[sl,1])),raw_x_span=float(np.ptp(raw[rl,0])),peak_east_speed=float(np.max(abs(lp[sl,3]))),peak_xy_speed=float(np.max(np.linalg.norm(lp[sl,2:4],axis=1))))
  h['local_east_extrema']=[dict(t=float(lt[sl][idx]),east=float(lp[sl,1][idx]),relative_to_held=float(lp[sl,1][idx]-pos[j,1])) for idx in [np.argmin(lp[sl,1]),np.argmax(lp[sl,1])]]
  h['entry_position_ned']=pos[j].tolist();holds.append(h)
 result=dict(flight=s,phases=phases,hold=holds)
 if holds:
  h=holds[0];t=h['start'];end=h['end'];j=near(st,t);selection=(st>=t-2)&(st<=min(t+7,land));rawsel=(rt>=t)&(rt<end)
  ax=axes[row,0];ax.plot(lt-t,lp[:,1]-pos[j,1],label='PX4 east position minus held target');ax.plot(st[selection]-t,pos[selection,1]-pos[j,1],label='east target');ax.axvspan(0,end-t,color='orange',alpha=.13,label='HOLD');ax.axhline(0,color='grey',lw=.6);ax.set_xlim(-2,min(7,land-t));ax.set_ylim(-.35,.35);ax.set_ylabel('m');ax.set_title(s+' first HOLD');ax.legend(fontsize=7);ax.grid(alpha=.2)
  ax=axes[row,1];ax.plot(lt-t,lp[:,3],label='actual east velocity');ax.plot(st-t,ff[:,1],label='east velocity feedforward');ax.axvspan(0,end-t,color='orange',alpha=.13);ax.set_xlim(-2,min(7,land-t));ax.set_ylim(-.4,.4);ax.set_ylabel('m/s');ax.legend(fontsize=7);ax.grid(alpha=.2)
 results.append(result)
for ax in axes[-1]:ax.set_xlabel('seconds relative to first HOLD')
fig.tight_layout();fig.savefig(OUT/'hold_comparison.png',dpi=140)
(OUT/'hold_detail.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))
