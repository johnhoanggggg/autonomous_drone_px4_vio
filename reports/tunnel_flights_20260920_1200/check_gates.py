from flight_data import *
from collections import defaultdict
geom=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
selector=subprocess.Popen([str(R.parent/'active_recovery_20260920/selector_probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def rpc(p,j):p.stdin.write(json.dumps(j)+'\n');p.stdin.flush();return json.loads(p.stdout.readline())
def clamp(v,n):return v*min(1,n/max(np.linalg.norm(v),1e-20))
F=Flight('120016');phases=F.phases()[-1];act=[(a,b) for a,b,k in phases if k=='ACTIVE_RECOVERY'];out=[]
for a,b in act:
 origin=xy(F.at(LP,a));samples=[a,a+.5,a+1,a+2,a+3,b-.10,b-.05,b-.001,b]
 for t in samples:
  lp=F.at(LP,t);body=xy(lp);mapbody,rotation=F.frame(t);measured=rotation@np.array([lp['vy'],lp['vx']]);di=F.at('/planner/follower/vio_displacement',t)['vector'];forward=rotation@np.array([di['x'],di['y']]);grid=F.at('/rtabmap/grid',t)
  vabs=np.linalg.norm(measured);qs=[[*mapbody,*mapbody],[*mapbody,*(mapbody+measured*(.25+vabs/.2))],[*mapbody,*(mapbody+measured/max(vabs,1e-9)*sd(max(0,vabs-.01)))]]
  baseline=rpc(geom,{'grid':grid,'queries':qs});candidate=rpc(selector,{'grid':grid,'body':mapbody.tolist(),'velocity':measured.tolist(),'forward':forward.tolist()})
  sps=[(st,m) for st,m in F.d[SP] if a<=st<t and math.isfinite(m['position'][0])];st,sp=sps[-1] if sps else (a,{'position':body.tolist(),'velocity':[0,0]})
  previous=np.array(sp['position'][:2]);pv=np.array(sp['velocity'][:2]);r={'t':t,'active_start':a,'elapsed':t-a,'speed':vabs,'body_clearance':baseline[0],'actual_stop_clearance':baseline[1],'full_governor_clearance':baseline[2],'candidate':candidate,'body_ned':body.tolist(),'origin_ned':origin.tolist(),'mapbody':mapbody.tolist(),'map_velocity':measured.tolist(),'forward_map':forward.tolist(),'previous_setpoint_t':st}
  if candidate['accepted']:
   desired=(rotation.T@np.array(candidate['velocity']))[::-1];velocity=pv+clamp(desired-pv,.15*.05);target=body+clamp(previous+velocity*.05-body,.05);stop=body+np.array([lp['vx'],lp['vy']])*(.25+vabs/.2)
   speedcmd=np.linalg.norm(velocity);delta=velocity/max(speedcmd,1e-9)*sd(speedcmd);cmdend=body+delta;combined=stop+(target-body)+delta
   transform=lambda point:mapbody+rotation@(point-body)[::-1]
   names=['actual_stop','target','command_stop','combined_stop','reference_sweep'];segments=[(body,stop),(body,target),(body,cmdend),(stop,combined),(previous,target)]
   vals=rpc(geom,{'grid':grid,'queries':[[*transform(x),*transform(y)] for x,y in segments]})
   floor=min(baseline[0],.38);minimums=[.28,floor,floor,.28,.28]
   r.update(command_map_velocity=(rotation@velocity[::-1]).tolist(),target_ned=target.tolist(),candidate_speed=speedcmd,clearance_checks={n:{'value':v,'required':minimum,'passes':v is not None and v+1e-6>=minimum} for n,v,minimum in zip(names,vals,minimums)},radial_checks={n:float(np.linalg.norm(v-origin)) for n,v in [('body',body),('target',target),('actual_stop',stop),('command_stop',cmdend),('combined_stop',combined)]})
   r['inferred_failed_checks']=[n for n,c in r['clearance_checks'].items() if not c['passes']]+[n+'_travel_radius' for n,v in r['radial_checks'].items() if n!='body' and v>.25]
  out.append(r)
for p in [geom,selector]:p.stdin.close();assert p.wait()==0
(R/'gate_reconstruction.json').write_text(json.dumps(out,indent=2))
for r in out:print('t',round(r['t'],4),'v',round(r['speed'],3),'clear',r['body_clearance'],'fullgov',r['full_governor_clearance'],'candidate',r['candidate']['accepted'],'failure',r.get('inferred_failed_checks'),'radius',r.get('radial_checks'),'corridors',r.get('clearance_checks'))
