from flight_data import *
p=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def rpc(g,q):p.stdin.write(json.dumps({'grid':g,'queries':q})+'\n');p.stdin.flush();return json.loads(p.stdout.readline())
out={}
for f,times in [('133740',[18.75,19.3,19.8,20.3,20.8,21.3,21.6,22.0,22.4,22.9,23.3,23.7,23.9]),('133831',[21.7,22.2,22.9,23.3,23.8,24.3,24.8,25.1,25.35,25.6,25.8,26.0])]:
 F=Flight(f);rows=[]
 for t in times:
  b,c=F.pair(t);m=pose(b);rotm=rot(yaw(c));lp=F.at(LP,t);sp=F.at(SP,t);v=rotm@np.array([lp['vy'],lp['vx']]);cv=rotm@np.array(sp['velocity'][:2][::-1]);lead=rotm@(np.array(sp['position'][:2])-xy(lp))[::-1];g=F.at('/rtabmap/grid',t);pts=[m,m+[.03,0],m+[-.03,0],m+[0,.03],m+[0,-.03]];vals=rpc(g,[[*z,*z] for z in pts]);grad=np.array([vals[1]-vals[2],vals[3]-vals[4]])/.06;grad/=max(np.linalg.norm(grad),1e-9)
  row=dict(t=t,body=m.tolist(),correction=c['pose'],clearance=vals[0],gradient=grad.tolist(),measured=v.tolist(),command=cv.tolist(),lead=lead.tolist(),inward_measured=float(grad@v),inward_command=float(grad@cv),inward_lead=float(grad@lead));rows.append(row)
 out[f]=rows
 for r in rows:print(f,r['t'],'clear',round(r['clearance'],3),'grad',np.round(r['gradient'],2),'vm',np.round(r['measured'],3),'vc',np.round(r['command'],3),'lead',np.round(r['lead'],3),'inward m/c/lead',*[round(r[k],3) for k in ['inward_measured','inward_command','inward_lead']])
p.stdin.close();p.wait();(R/'drift.json').write_text(json.dumps(out,indent=2))
