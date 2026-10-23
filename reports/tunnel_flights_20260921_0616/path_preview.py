from flight_data import *
for f,times in [('061515',[20.,23.,24.4,25.,25.8]),('061647',[24.,28.,31.])]:
 F=Flight(f);print(f)
 for t in times:
  b,rt=F.frame(t);path=F.at('/planner/path',t);source=next(c for _,(bp,c) in reversed(F.pairs) if stamp(c)==stamp(path));current=F.pair(t)[1]
  pts=np.array([pose(current)+rt@rot(yaw(source)).T@(pose(x)-pose(source)) for x in path['poses']]);segs=np.diff(pts,axis=0);lens=np.linalg.norm(segs,axis=1);s=np.r_[0,np.cumsum(lens)]
  alphas=np.clip(np.sum((b-pts[:-1])*segs,axis=1)/np.maximum(lens**2,1e-9),0,1);projs=pts[:-1]+alphas[:,None]*segs;k=np.argmin(np.linalg.norm(projs-b,axis=1));along=s[k]+alphas[k]*lens[k];q=projs[k]
  def at(a):return np.array([np.interp(a,s,pts[:,i]) for i in range(2)])
  near=at(along+.08)-q;far=at(along+.4)-q
  print(t,'body',np.round(b,3),'projection',np.round(q,3),'near',np.round(near,3),'far',np.round(far,3),'carrot .35',np.round(at(along+.35)-b,3),'head',np.round(pts[:6],3).tolist())
