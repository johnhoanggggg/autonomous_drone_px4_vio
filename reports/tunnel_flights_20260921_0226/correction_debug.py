from flight_data import *
F=Flight('022647');d=F.d
p=subprocess.Popen([str(R/'replay_probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
c=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def rpc(proc,q):proc.stdin.write(json.dumps(q)+'\n');proc.stdin.flush();return json.loads(proc.stdout.readline())
rows=[]
for t in [24.72,25.20,26.67,27.20,27.80,28.20,29.67,30.15,30.50,31.00,32.10,33.00,34.45]:
 b,rt=F.frame(t);lp=F.at(LP,t);v=rt@np.array([lp['vy'],lp['vx']]);vec=F.at('/planner/follower/vio_displacement',t)['vector'];fw=rt@np.array([vec['x'],vec['y']]);fw/=np.linalg.norm(fw);g=F.at('/rtabmap/grid',t)
 stop=b+v*(.25+np.linalg.norm(v)/.2)
 cs=rpc(c,dict(grid=g,queries=[[*b,*b],[*b,*stop],[*b,*(b+fw*.3)],[*b,*(b+fw*.6)]]))
 q=dict(grid=g,body=b.tolist(),velocity=v.tolist(),forward=fw.tolist(),previous_position=b.tolist(),previous_velocity=[0,0],origin=b.tolist(),launch=b.tolist(),adaptive=True)
 result=rpc(p,q)
 row=dict(t=t,body_clearance=cs[0],measured_stop_clearance=cs[1],forward_30cm=cs[2],forward_60cm=cs[3],measured_speed=float(np.linalg.norm(v)),entry_speed_ok=bool(np.linalg.norm(v)<=.1),candidate=result,flight_status=F.at('/planner/flight/status',t)['data'],follower_valid=F.at('/planner/follower/valid',t)['data']);rows.append(row)
 print(t,'body/stop/fwd30/60',np.round(cs,3),'speed',round(float(np.linalg.norm(v)),3),'candidate',result['accepted'],'valid',row['follower_valid'])
(R/'correction_debug.json').write_text(json.dumps(rows,indent=2))
print('\nCorrection edges:')
cor=d['/rtabmap/odom_correction'];ts=[t for t,m in cor]
for t in [10.642,24.698,26.631,29.642]:
 i=bisect.bisect_right(ts,t)-1
 for j in [i-1,i,i+1]:
  tm,m=cor[j];last=cor[j-1][1];dt=pose(m)-pose(last);da=(yaw(m)-yaw(last)+math.pi)%(2*math.pi)-math.pi
  dz=m['pose']['position']['z']-last['pose']['position']['z']
  print('trigger',t,'receipt',round(tm,4),'raw XY',round(float(np.linalg.norm(dt)),5),'Z',round(dz,5),'XYZ',round(float(np.sqrt(dt@dt+dz*dz)),5),'yawdeg',round(math.degrees(da),3))
for proc in [p,c]:proc.stdin.close();assert proc.wait()==0
