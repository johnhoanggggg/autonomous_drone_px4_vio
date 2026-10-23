from flight_data import *
p=subprocess.Popen([str(R.parent/'continuous_tunnel_20260921/replay_probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
c=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def rpc(proc,q):proc.stdin.write(json.dumps(q)+'\n');proc.stdin.flush();return json.loads(proc.stdout.readline())
rows=[]
for f,ts in [('020152',[18.08,18.27,23.05,23.38]),('020232',[16.78,17.10,19.36])]:
 F=Flight(f)
 for t in ts:
  b,rt=F.frame(t);lp=F.at(LP,t);sp=F.at(SP,t);v=rt@np.array([lp['vy'],lp['vx']]);cv=rt@np.array(sp['velocity'][:2][::-1]);old_target=b+rt@(np.array(sp['position'][:2])-xy(lp))[::-1]
  lead=old_target-b;target=b+lead*min(1,.046999/max(np.linalg.norm(lead),1e-9));stop=b+v*(.25+np.linalg.norm(v)/.2)
  # At zero seed velocity the response-acceleration envelope has no direction.
  # Check the complete retained target transition including the reference sweep.
  combined=stop+target-b
  cs=rpc(c,dict(grid=F.at('/rtabmap/grid',t),queries=[[*b,*b],[*b,*target],[*b,*stop],[*stop,*combined],[*old_target,*target]]))
  valid=all(x is not None for x in cs) and cs[0]>=.28 and cs[1]+1e-6>=cs[0] and min(cs[2:])>=.28 and max(np.linalg.norm(z-b) for z in [target,stop,combined])<=.25
  d=F.at('/planner/follower/vio_displacement',t)['vector']
  q=dict(grid=F.at('/rtabmap/grid',t),body=b.tolist(),velocity=v.tolist(),forward=(rt@np.array([d['x'],d['y']])).tolist(),previous_velocity=[0,0],origin=b.tolist(),launch=b.tolist(),adaptive=True)
  variants={}
  for name,seed in [('reset',b),('retain_position',target if valid else b)]:
   variants[name]=rpc(p,{**q,'previous_position':seed.tolist()})
  row=dict(f=f,t=t,retention_permitted=bool(valid),body_clearance=cs[0],retained_clearances=cs[1:],previous_lead=float(np.linalg.norm(lead)),previous_speed=float(np.linalg.norm(cv)),retained_lead=float(np.linalg.norm(target-b)),variants=variants);rows.append(row)
  print(f,t,'retain',valid,'body',round(cs[0],3),'lead',round(float(np.linalg.norm(target-b)),3),'accepted',[(k,v['accepted']) for k,v in variants.items()])
(R/'entry_replay.json').write_text(json.dumps(rows,indent=2))
for proc in [p,c]:proc.stdin.close();proc.wait()
