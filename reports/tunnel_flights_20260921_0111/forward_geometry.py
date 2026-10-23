from flight_data import *
F=Flight('011120'); p=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def probe(g,qs):p.stdin.write(json.dumps(dict(grid=g,queries=qs))+'\n');p.stdin.flush();return json.loads(p.stdout.readline())
rows=[]
for t in [16.05,17.4,18.74,19.97,20.65]:
 b,rt=F.frame(t);g=F.at('/rtabmap/grid',t);path=np.array([pose(m) for m in F.at('/planner/path',t)['poses']]);cs=probe(g,[[*a,*z] for a,z in zip(path,path[1:])]);print(t,'path',path.tolist(),'segmentclear',cs,'min',min(x for x in cs if x is not None))
 r=next(x for x in json.loads((R/'diagnosis.json').read_text()) if x['t']==t)
 if t==19.97:
  lp=F.at(LP,t);sp=F.at(SP,t);prev=b+np.array(r['lead']);pv=np.array(r['command']);bodyclear=r['body_clearance'];meas=np.array(r['measured']);stop=b+meas*(.25+np.linalg.norm(meas)/.2)
  for name in ['slower','hard_floor']:
   x=r['variants'][name];desired=np.array(x['velocity']);dv=desired-pv;dv*=min(1,.0075/max(np.linalg.norm(dv),1e-9));v=pv+dv;target=prev+v*.05;delta=target-b;target=b+delta*min(1,.046999/max(np.linalg.norm(delta),1e-9));speed=np.linalg.norm(v);distance=speed*.25+.5*.3*.25**2+(speed+.3*.25)**2/.2;cmd=distance*v/max(speed,1e-9);combined=stop+(target-b)+cmd
   cs=probe(g,[[*b,*stop],[*b,*target],[*b,*(b+cmd)],[*stop,*combined],[*prev,*target]])
   print('RAMP',name,'v',v,'clearances',cs,'required',[.28,min(bodyclear,.38),min(bodyclear,.38),.28,.28])
 # Frozen-frame displacement during final zero-velocity stop.
 if t==19.97:
  a=F.at(LP,19.974);z=F.at(LP,21.04);delta=rt@np.array([z['y']-a['y'],z['x']-a['x']]);print('zero-stop frozen displacement',delta,'inward',delta@np.array(r['gradient']))
p.stdin.close();p.wait()
