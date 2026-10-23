from flight_data import *
F=Flight('011120')
p=subprocess.Popen([str(R/'selector_probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
c=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def rpc(proc,q):proc.stdin.write(json.dumps(q)+'\n');proc.stdin.flush();return json.loads(proc.stdout.readline())
rows=[]
for t in [13.35,14.42,16.05,16.15,17.4,18.0,18.5,18.74,18.82,19.1,19.3,19.5,19.7,19.85,19.92,19.97,20.0,20.2,20.65,21.04]:
 b,rt=F.frame(t);lp=F.at(LP,t);sp=F.at(SP,t);d=F.at('/planner/follower/vio_displacement',t)['vector'];fw=rt@np.array([d['x'],d['y']]);fw/=np.linalg.norm(fw);v=rt@np.array([lp['vy'],lp['vx']]);cv=rt@np.array(sp['velocity'][:2][::-1]);lead=rt@(np.array(sp['position'][:2])-xy(lp))[::-1];g=F.at('/rtabmap/grid',t)
 stop=b+v*(.25+np.linalg.norm(v)/.2)
 points=[b,b+[.03,0],b+[-.03,0],b+[0,.03],b+[0,-.03]]
 query=[[*z,*z] for z in points]+[[*b,*stop]]+[[*b,*(b+fw*l)] for l in [.1,.3,.6]]
 cs=rpc(c,dict(grid=g,queries=query));gr=np.array([cs[1]-cs[2],cs[3]-cs[4]]);gr/=max(np.linalg.norm(gr),1e-9)
 q=dict(grid=g,body=b.tolist(),velocity=v.tolist(),forward=fw.tolist())
 variants={name:rpc(p,{**q,**args}) for name,args in [('current',{}),('slower',dict(speed=.03)),('hard_floor',dict(reserve=.28)),('short_lookahead',dict(lookahead=.03))]}
 r=dict(t=t,body=b.tolist(),body_clearance=cs[0],actual_stop_clearance=cs[5],forward_clearances=cs[6:],forward=fw.tolist(),gradient=gr.tolist(),measured=v.tolist(),command=cv.tolist(),lead=lead.tolist(),forward_measured=float(v@fw),forward_command=float(cv@fw),inward_measured=float(v@gr),inward_command=float(cv@gr),variants=variants);rows.append(r)
 print(t,'body/stop',np.round([cs[0],cs[5]],3),'fwd10/30/60cm',np.round(cs[6:],3),'v/fwdcmd',np.round([v@fw,cv@fw],3),'inward m/c',np.round([v@gr,cv@gr],3),'selector',[(n,x['accepted'],x['rejections']) for n,x in variants.items()])
(R/'diagnosis.json').write_text(json.dumps(rows,indent=2))
for proc in [p,c]:proc.stdin.close();proc.wait()
