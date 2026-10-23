"""Reconstruct checkpoint prerequisites on the flown trajectory; not a counterfactual flight."""
from flight_data import *
p=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def rpc(g,qs):p.stdin.write(json.dumps(dict(grid=g,queries=qs))+'\n');p.stdin.flush();return json.loads(p.stdout.readline())
def inv(point,c):return rot(-yaw(c))@(point-pose(c))
def to_map(point,c):return rot(yaw(c))@point+pose(c)
def floor(v):return .33+.05*min(abs(v)/.15,1.)**2
rows=[]
for f,episodes in [('033547',[(18.26,21.204),(27.424,30.484),(34.08,37.31)]),('033744',[(19.194,21.953)])]:
 F=Flight(f);corr={stamp(m):m for _,m in F.d['/rtabmap/odom_correction']}
 for start,end in episodes:
  b,rt=F.frame(start);c0=F.pair(start)[1];orig=xy(F.at(LP,start));path0=F.at('/planner/path',start);goal_vio=inv(pose(path0['poses'][-1]),corr[stamp(path0)]);goal_map=to_map(goal_vio,c0);goal_ned=orig+(rt.T@(goal_map-b))[::-1];gd=np.linalg.norm(inv(b,c0)-goal_vio)
  eligible=[]
  for t in np.arange(start+.1,end,.05):
   b,rt=F.frame(t);c=F.pair(t)[1];lp=F.at(LP,t);body=xy(lp);path=F.at('/planner/path',t);pc=corr[stamp(path)];gv=inv(pose(path['poses'][-1]),pc)
   progress=np.linalg.norm(orig-goal_ned)-np.linalg.norm(body-goal_ned);vp=gd-np.linalg.norm(inv(b,c)-goal_vio);goal_delta=np.linalg.norm(gv-goal_vio)
   samples=np.array([xy(m) for tm,m in F.d[LP] if start<=tm<=t]);travel=np.linalg.norm(np.diff(samples,axis=0),axis=1).sum()
   if min(progress,vp)<.08 or progress<.35*travel:continue
   points=np.array([to_map(inv(pose(m),pc),c) for m in path['poses']]);lens=np.linalg.norm(np.diff(points,axis=0),axis=1);cum=np.r_[0,np.cumsum(lens)];projs=[]
   for i,(a,z) in enumerate(zip(points,points[1:])):
    d=z-a;u=np.clip((b-a)@d/(d@d),0,1) if d@d else 0;projs.append((np.linalg.norm(a+u*d-b),cum[i]+u*lens[i]))
   along=min(projs)[1]+.2;i=min(np.searchsorted(cum,along,side='right')-1,len(lens)-1);ahead=points[i]+(points[i+1]-points[i])*np.clip((along-cum[i])/max(lens[i],1e-9),0,1)
   vel=np.array([lp['vx'],lp['vy']]);vabs=np.linalg.norm(vel);sp=F.at(SP,t);target=np.array(sp['position'][:2]);cv=np.array(sp['velocity'][:2]);cspeed=np.linalg.norm(cv);stop=body+vel*(.25+vabs/.2);cd=cv*sd(cspeed)/cspeed if cspeed else cv;ce=body+cd;combined=stop+target-body+cd
   def mp(x):return b+rt@(x-body)[::-1]
   qs=[[*b,*ahead],[*b,*mp(stop)],[*b,*b],[*b,*mp(target)],[*b,*mp(ce)],[*mp(stop),*mp(combined)]];cl=rpc(F.at('/rtabmap/grid',t),qs)
   radius=max(np.linalg.norm(x-orig) for x in [target,stop,ce,combined]);ready=goal_delta<=.02 and vabs<=.1 and cl[0]>=floor(.06) and cl[1]>=floor(max(vabs,.06)) and radius<=.25 and cl[3]>=min(cl[2],floor(cspeed))-1e-6 and cl[4]>=min(cl[2],floor(cspeed))-1e-6 and cl[5]>=.28 and travel<.25
   row=dict(flight=f,entry=start,t=float(t),goal_progress=float(progress),vio_progress=float(vp),goal_change=float(goal_delta),travel=float(travel),speed=float(vabs),radius=float(radius),clearances=cl,ready=bool(ready));rows.append(row)
   if ready:eligible.append(t)
  print(f,start,'first checkpoint',eligible[0] if eligible else None,'eligible ticks',len(eligible))
p.stdin.close();p.wait();(R/'checkpoint_replay.json').write_text(json.dumps(rows,indent=2))
