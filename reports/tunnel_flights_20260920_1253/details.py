from flight_data import *
from collections import defaultdict
p=subprocess.Popen([str(R/'snapshot_probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
gp=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def rpc(proc,obj):proc.stdin.write(json.dumps(obj)+'\n');proc.stdin.flush();return json.loads(proc.stdout.readline())
results={}
for f,times in [('125348',[15.5,16.5,17.0,17.04,17.075,17.085]),('125509',[24.25,25.90,25.93,30.22,30.87,31.52,31.97,32.17,34.97,35.6,36.5,37.5,38.5,39.0,39.56])]:
 F=Flight(f);d=F.d;start,end,land,kill,phases=F.phases();rows=[]
 for t in times:
  lp=F.at(LP,t);body,rotm=F.frame(t);grid=F.at('/rtabmap/grid',t);v=rotm@np.array([lp['vy'],lp['vx']]);di=F.at('/planner/follower/vio_displacement',t)['vector'];forward=rotm@np.array([di['x'],di['y']]);sp=F.at(SP,t);reference=rotm@np.array(sp['velocity'][:2][::-1]);lead=rotm@(np.array(sp['position'][:2])-xy(lp))[::-1]
  req={'grid':grid,'body':body.tolist(),'velocity':v.tolist(),'forward':forward.tolist(),'directions':[forward.tolist(),reference.tolist(),[0,-1]]}
  answer=rpc(p,req)
  vs=np.linalg.norm(v);evalspeed=max(0,vs-.01);direction=v/max(vs,1e-9)
  queries=[[*body,*(body+direction*sd(evalspeed))],[*body,*(body+direction*sd(evalspeed,0))]]
  cs=rpc(gp,{'grid':grid,'queries':queries})
  row={'t':t,'body_map':body.tolist(),'measured_velocity_map':v.tolist(),'command_velocity_map':reference.tolist(),'lead_map':lead.tolist(),'desired_direction_map':(forward/np.linalg.norm(forward)).tolist(),'snapshot':answer,'full_governor_clearance':cs[0],'coast_governor_clearance':cs[1]}
  if not answer['accepted']:
   row['without_gradient_veto']=rpc(p,{**req,'disable_gradient':True,'directions':[]})
   row['slower_selector_trials']={}
   for speed in [.05,.04,.03,.02,.01]:row['slower_selector_trials'][speed]=rpc(p,{**req,'speed':speed,'directions':[]})
  rows.append(row)
 statuses=d['/planner/flight/status'];dur=defaultdict(float)
 for (t,m),(tn,_) in zip(statuses,statuses[1:]):
  dt=max(0,min(tn,end)-max(t,start));s=m['data']
  if s.startswith('ROUTE valid'):dur['route_status']+=dt;dur['route_slow_rejoin' if 'slow_rejoin=' in s else 'route_no_rejoin']+=dt
  elif s.startswith('ACTIVE_RECOVERY'):dur['active_status']+=dt
 Fcor=[(t,m) for t,m in d['/rtabmap/odom_correction'] if start<=t<=end]
 changes=[(t,float(np.linalg.norm(pose(m)-pose(pm))),abs(math.degrees((yaw(m)-yaw(pm)+math.pi)%(2*math.pi)-math.pi))) for (_,pm),(t,m) in zip(Fcor,Fcor[1:])]
 frozen=[]
 intervals=[(17.08,21.90)] if f=='125348' else [(24.23,25.90),(35.55,39.55),(39.56,44.05)]
 for a,b in intervals:
  b0,c0=F.pair(a);b1,c1=F.pair(b);p0=pose(b0);p1=pose(b1);p1old=rot(yaw(c0))@rot(yaw(c1)).T@(p1-pose(c1))+pose(c0)
  cs0=rpc(gp,{'grid':F.at('/rtabmap/grid',a),'queries':[[*p0,*p0],[*p1old,*p1old]]});cs1=rpc(gp,{'grid':F.at('/rtabmap/grid',b),'queries':[[*p1,*p1]]})
  frozen.append({'start':a,'end':b,'body_start':p0.tolist(),'body_end':p1.tolist(),'old_clearance':cs0[0],'motion_on_frozen_grid':cs0[1],'final_live_clearance':cs1[0],'correction_delta':float(np.linalg.norm(pose(c1)-pose(c0)))})
 results[f]={'snapshots':rows,'status_seconds':dict(dur),'correction_change_count':sum(v>1e-6 or a>1e-6 for t,v,a in changes),'maximum_correction_translation_edge':max((v for t,v,a in changes),default=0),'maximum_correction_yaw_edge':max((a for t,v,a in changes),default=0),'frozen_map_intervals':frozen}
for proc in [p,gp]:proc.stdin.close();assert proc.wait()==0
(R/'details.json').write_text(json.dumps(results,indent=2))
for f,r in results.items():
 print('\nFLIGHT',f,{k:v for k,v in r.items() if k!='snapshots'})
 for s in r['snapshots']:
  a=s['snapshot'];print(s['t'],'body',round(a['body_clearance'],3),'caps desired/reference/tunnel',a['caps'],'selector',a['accepted'],'gov full/coast',s['full_governor_clearance'],s['coast_governor_clearance'],'slower',[(k,v['accepted']) for k,v in s.get('slower_selector_trials',{}).items()],'forward',s['desired_direction_map'],'command',s['command_velocity_map'],'lead',s['lead_map'])
