from flight_data import *
p=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def rpc(g,q):p.stdin.write(json.dumps(dict(grid=g,queries=q))+'\n');p.stdin.flush();return json.loads(p.stdout.readline())
rows=[]
for f,entries in [('033547',[(18.26,[19.4,19.9,20.4,20.6,20.9,21.1]),(27.424,[28.4,28.8,29.2,29.5,29.8,30.1]),(34.08,[34.7,35.2,35.8,36.2,36.5,36.8,37.1])]),('033744',[(19.194,[20.,20.5,21.,21.5,21.9,22.5,23.,23.3])])]:
 F=Flight(f)
 for origin_t,times in entries:
  orig=xy(F.at(LP,origin_t));print('\n',f,'entry',origin_t)
  for t in times:
   b,rt=F.frame(t);lp=F.at(LP,t);v=rt@np.array([lp['vy'],lp['vx']]);path=F.at('/planner/path',t);goal=pose(path['poses'][-1]); fw=goal-b;fw/=np.linalg.norm(fw);dis=xy(lp)-orig;dis=rt@dis[::-1]; initial=goal-(b-dis);progress=np.linalg.norm(initial)-np.linalg.norm(goal-b)
   vec=F.at('/planner/follower/vio_displacement',t)['vector'];fwd=rt@np.array([vec['x'],vec['y']]);fwd/=np.linalg.norm(fwd);lat=np.array([-fwd[1],fwd[0]])
   qs=[[*b,*b],[*b,*(b+v*(.25+np.linalg.norm(v)/.2))],[*b,*(b+fwd*.2)]]
   for x in [-.10,-.05,0,.05,.1]:z=b+lat*x;qs+=[[*z,*z],[*z,*(z+fwd*.2)]]
   cs=rpc(F.at('/rtabmap/grid',t),qs)
   row=dict(f=f,t=t,progress=progress,speed=np.linalg.norm(v),clearances=cs);rows.append(row);print(t,'progress',round(progress,3),'speed',round(np.linalg.norm(v),3),'body/stop/fwd',np.round(cs[:3],3),'lateral',np.round(cs[3:],3))
p.stdin.close();p.wait();(R/'corridor_probe.json').write_text(json.dumps(rows,indent=2))
