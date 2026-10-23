from pathlib import Path
import json,sys,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_yaw_tracking import read
from render_flight_map import Grid

def idx(t,q):return np.clip(np.searchsorted(t,q,side='right')-1,0,len(t)-1)
def series(b,k,fields):
 r=b[k];return np.array([x['t'] for x in r]),np.array([[x[f] for f in fields] for x in r])
def yaw(r):return np.arctan2(2*(r['qw']*r['qz']+r['qx']*r['qy']),1-2*(r['qy']**2+r['qz']**2))
results=[]
fig,axes=plt.subplots(1,2,figsize=(12,9))
for ax,stamp,start,end in zip(axes,['081620','081709'],[9.,8.6],[34.,30.7]):
 folder=ROOT/f'ros2_ws/flight_logs/offboard_global_20260916T{stamp}Z';b=json.loads((folder/'render/bag_data.json').read_text());t0=b['t0_receipt_ns']
 st,sp=series(b,'/fmu/in/trajectory_setpoint',['position','velocity']);pos=sp[:,0,:2];vel=sp[:,1,:2];dt=np.diff(st)
 pt,p=series(b,'/rtabmap/body_pose',['x','y']);rt,raw=series(b,'/rtabmap/body_vio_pose',['x','y']);ct,c=series(b,'/rtabmap/odom_correction',['x','y']);cy=np.array([yaw(r) for r in b['/rtabmap/odom_correction']]);lt,lp=series(b,'/fmu/out/vehicle_local_position_v1',['x','y','vx','vy'])
 status=b['/planner/flight/status'];ft=np.array([r['t'] for r in status]);states=np.array([r['text'].split()[0] for r in status]);state=states[idx(ft,st)]
 active=(st>=start)&(st<=end);finite=np.isfinite(vel).all(axis=1);normal=active[1:]&finite[1:]&finite[:-1]&(dt>.035)&(dt<.07)
 dp=np.linalg.norm(np.diff(pos,axis=0),axis=1);dv=np.linalg.norm(np.diff(vel,axis=0),axis=1);res=np.linalg.norm(np.diff(pos,axis=0)-vel[1:]*dt[:,None],axis=1)
 transitions=[]
 for i in np.flatnonzero(active[1:] & (dp>.035)):
  transitions.append(dict(t=float(st[i+1]),dp=float(dp[i]),finite_both=bool(finite[i] and finite[i+1]),state=str(state[i+1]),old=pos[i].tolist(),new=pos[i+1].tolist()))
 # Plot geometry in each sample's current map frame. Command reconstructed
 # from instantaneous map-body plus rotated PX4 command-minus-position.
 ci=idx(ct,st);ppi=idx(pt,st);lpi=idx(lt,st);offset=pos-lp[lpi,:2];north,east=offset[:,0],offset[:,1]
 cmd=p[ppi]+np.column_stack((east*np.cos(cy[ci])-north*np.sin(cy[ci]),east*np.sin(cy[ci])+north*np.cos(cy[ci])))
 corr=[]
 for i in range(1,len(ct)):
  rr=raw[idx(rt,ct[i])]
  def tr(j):
   co,si=np.cos(cy[j]),np.sin(cy[j]);return np.array([co*rr[0]-si*rr[1],si*rr[0]+co*rr[1]])+c[j]
  d=tr(i)-tr(i-1)
  if start<=ct[i]<=end and np.linalg.norm(d)>.025:corr.append(dict(t=float(ct[i]),dx=float(d[0]),dy=float(d[1]),norm=float(np.linalg.norm(d)),yaw_deg=float(np.degrees(cy[i]-cy[i-1]))))
 data,_=read(str(folder),['/rtabmap/grid']);grids=[((s-t0)/1e9,Grid(m)) for s,m in data['/rtabmap/grid']];gt=np.array([t for t,g in grids]);clear=[]
 for t,point in zip(pt,p):
  if not start<=t<=end:continue
  gi=np.searchsorted(gt,t,side='right')-1
  if gi<0:continue
  cl=grids[gi][1].point_clearance(*point)
  if cl is not None:clear.append(dict(t=float(t),clearance=float(cl),x=float(point[0]),y=float(point[1])))
 g=b['final_grid'];arr=np.array(g['data']).reshape(g['height'],g['width']);im=np.ones(arr.shape)*.95;im[arr<0]=.65;im[arr>=65]=.15
 ax.imshow(im,origin='lower',cmap='gray',vmin=0,vmax=1,extent=[g['origin_x'],g['origin_x']+g['width']*g['resolution'],g['origin_y'],g['origin_y']+g['height']*g['resolution']]);ax.plot(p[:,0],p[:,1],color='tab:red',label='map body track');ax.plot(cmd[active,0],cmd[active,1],color='tab:blue',alpha=.75,label='reconstructed position command')
 for t in [12,16,18,21,25,29]:
  pa=[r for r in b['/planner/path'] if r['t']<=t and r['xy']]
  if pa:
   v=np.array(pa[-1]['xy']);ax.plot(v[:,0],v[:,1],'--',alpha=.4,lw=1)
  j=idx(pt,t);ax.annotate(str(t),(p[j,0],p[j,1]),fontsize=9)
 ax.set_xlim(-.45,1.25);ax.set_ylim(-2.6,.55);ax.set_aspect('equal');ax.set_title(stamp+' — final grid for context');ax.set_xlabel('map x / m');ax.set_ylabel('map y / m');ax.legend(fontsize=8)
 route=(lt>=start)&(lt<=end);err=np.linalg.norm(pos-lp[lpi,:2],axis=1)
 result=dict(flight=stamp,t0=t0,window=[start,end],samples=int(normal.sum()),finite_velocity_step_max=float(dv[normal].max()),finite_position_step_max=float(dp[normal].max()),finite_secant_residual_max=float(res[normal].max()),position_jumps=transitions,correction_events=corr,clearance_min=min(clear,key=lambda r:r['clearance']),clearance_below_28cm=sum(r['clearance']<.28 for r in clear),clearance_samples=len(clear),peak_speed=float(np.linalg.norm(lp[route,2:],axis=1).max()),peak_command_error=float(err[active].max()),p90_command_error=float(np.quantile(err[active],.9)),largest_finite_steps=[dict(t=float(st[i+1]),dp=float(dp[i]),res=float(res[i]),dv=float(dv[i]),state=str(state[i+1])) for i in sorted(np.flatnonzero(normal),key=lambda i:dp[i],reverse=True)[:12]])
 results.append(result)
 # bag-only motion plot
 f,aa=plt.subplots(4,1,figsize=(13,10),sharex=True)
 aa[0].plot(pt,p[:,0],label='map body x');aa[0].plot(st,cmd[:,0],label='reconstructed command x');j=idx(ct,start);co,si=np.cos(cy[j]),np.sin(cy[j]);frozen=co*raw[:,0]-si*raw[:,1]+c[j,0];aa[0].plot(rt,frozen,label='raw VIO x, transform frozen at route start',alpha=.6)
 aa[1].plot(st[1:],dp*100,label='position step / cm');aa[1].plot(st[1:],res*100,label='position secant residual / cm');aa[1].set_ylim(0,30)
 aa[2].plot(lt,lp[:,3]*np.cos(cy[idx(ct,lt)])-lp[:,2]*np.sin(cy[idx(ct,lt)]),label='actual lateral velocity');aa[2].plot(st,vel[:,1]*np.cos(cy[ci])-vel[:,0]*np.sin(cy[ci]),label='velocity feedforward')
 aa[3].plot([r['t'] for r in clear],[r['clearance'] for r in clear],label='body-centre occupied-cell clearance');aa[3].axhline(.28,color='red',ls='--',label='configured .28m');aa[3].set_ylim(.15,.6)
 for a in aa:
  a.grid(alpha=.2);a.legend(fontsize=8);a.set_xlim(start,end)
  for e in corr:
   if e['norm']>.06:a.axvline(e['t'],alpha=.3,color='purple')
 aa[-1].set_xlabel('Seconds from bag export origin');f.suptitle(stamp);f.tight_layout();f.savefig(OUT/f'{stamp}_bag_motion.png',dpi=130);plt.close(f)
fig.tight_layout();fig.savefig(OUT/'routes.png',dpi=140)
(OUT/'motion.json').write_text(json.dumps(results,indent=2))
for r in results:print(json.dumps(r,indent=2))
