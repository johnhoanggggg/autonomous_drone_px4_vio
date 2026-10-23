from flight_data import *
c=subprocess.Popen([str(R.parent/'tunnel_review_20260920/probe')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def rpc(q):c.stdin.write(json.dumps(q)+'\n');c.stdin.flush();return json.loads(c.stdout.readline())
rows=[]
for f,windows in [('020152',[(16.23,18.29),(22.05,23.4)]),('020232',[(15.08,17.12),(18.33,19.38)])]:
 F=Flight(f);print('\nFLIGHT',f)
 for a,z in windows:
  first=None
  for t in np.arange(a,z,.05):
   b,rt=F.frame(t);lp=F.at(LP,t);sp=F.at(SP,t);d=F.at('/planner/follower/vio_displacement',t)['vector'];fw=rt@np.array([d['x'],d['y']]);fw/=np.linalg.norm(fw);v=rt@np.array([lp['vy'],lp['vx']]);lat=v-fw*(v@fw);s=np.linalg.norm(lat)
   lead=rt@(np.array(sp['position'][:2])-xy(lp))[::-1];target=b+lead;old=b+lat*(.25+s/.2);new=b+lat*(1.+s/.2)
   qs=[[*b,*b],[*b,*old],[*b,*new],[*b,*target]]
   cs=rpc(dict(grid=F.at('/rtabmap/grid',t),queries=qs))
   eligible=np.linalg.norm(v)<=.1 and s>=.03
   old_trigger=eligible and cs[0]<=.43 and cs[1]+.005<min(cs[0],.38)
   new_trigger=eligible and cs[2]+.005<min(cs[0],.38)
   row=dict(f=f,t=float(t),body=cs[0],old_stop=cs[1],anticipated_stop=cs[2],target_clearance=cs[3],speed=float(np.linalg.norm(v)),lateral_speed=float(s),lead=lead.tolist(),old=bool(old_trigger),new=bool(new_trigger));rows.append(row)
   if new_trigger and first is None:first=row;print('first predictive trigger',row)
  print('last',rows[-1])
(R/'drift_diagnosis.json').write_text(json.dumps(rows,indent=2));c.stdin.close();c.wait()
