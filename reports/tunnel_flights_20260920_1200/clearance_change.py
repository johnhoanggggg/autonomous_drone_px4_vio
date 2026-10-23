from flight_data import *
p=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def probe(g,qs):p.stdin.write(json.dumps({'grid':g,'queries':qs})+'\n');p.stdin.flush();return json.loads(p.stdout.readline())
results=[]
for f,t0,t1 in [('120016',19.491,22.491),('120016',22.491,23.046),('120016',23.046,27.025),('120055',17.20,17.30)]:
 F=Flight(f);b0,c0=F.pair(t0);b1,c1=F.pair(t1);p0=pose(b0);p1=pose(b1);g0=F.at('/rtabmap/grid',t0);g1=F.at('/rtabmap/grid',t1)
 p1old=rot(yaw(c0))@rot(yaw(c1)).T@(p1-pose(c1))+pose(c0)
 vals=probe(g0,[[*p0,*p0],[*p1old,*p1old],[*p1,*p1]])+probe(g1,[[*p1,*p1]])
 results.append({'flight':f,'t0':t0,'t1':t1,'old_clearance':vals[0],'after_raw_motion_old_correction_and_grid':vals[1],'after_correction_change_old_grid':vals[2],'new_clearance':vals[3],'raw_pose_displacement':(p1old-p0).tolist(),'px4_ned_displacement':(xy(F.at(LP,t1))-xy(F.at(LP,t0))).tolist(),'correction_translation_delta':(pose(c1)-pose(c0)).tolist()})
(R/'clearance_change.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
# Independent snapshot of route handover caps, and a one-step smaller command at final active exit.
F=Flight('120016');gates=json.loads((R/'gate_reconstruction.json').read_text());out=[]
for r in gates:
 t=r['t'];body=np.array(r['mapbody']);g=F.at('/rtabmap/grid',t);thresh=min(.38,r['body_clearance']);dirs=[np.array(r['forward_map']),np.array(r['command_map_velocity'])]
 def cap(v):
  norm=np.linalg.norm(v)
  if norm<1e-6:return .15
  v=v/norm
  def ok(travel):
   c=probe(g,[[*body,*(body+v*travel)]])[0];return c is not None and c+1e-6>=thresh
  if ok(sd(.15)):return .15
  a,b=0,sd(.15)
  for _ in range(12):
   mid=(a+b)/2
   if ok(mid):a=mid
   else:b=mid
  lo,hi=0.,.15
  for _ in range(40):
   mid=(lo+hi)/2
   if sd(mid)<=a:lo=mid
   else:hi=mid
  return lo
 caps=[cap(v) for v in dirs]
 out.append({'t':t,'follower_command_cap':caps[0],'active_reference_command_cap':caps[1],'readiness_cap':min(caps),'full_governor_clearance':r['full_governor_clearance'],'threshold':thresh})
(R/'route_headroom.json').write_text(json.dumps(out,indent=2));print('HEADROOM',json.dumps(out,indent=2))
p.stdin.close();assert p.wait()==0
