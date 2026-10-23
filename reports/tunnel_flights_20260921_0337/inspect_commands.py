from flight_data import *
for f,times in [('033547',[19.4,19.9,20.6,21.0,28.4,29.5,34.7,36.5,37.1]),('033744',[21.9,22.2,22.5,22.8,23.1,23.35,23.5,23.7,23.85])]:
 F=Flight(f); print('\n',f)
 for t in times:
  b,rt=F.frame(t);lp=F.at(LP,t);sp=F.at(SP,t);v=rt@np.array([lp['vy'],lp['vx']]);vs=rt@np.array([sp['velocity'][1],sp['velocity'][0]]);dv=F.at('/planner/follower/vio_displacement',t)['vector'];fw=rt@np.array([dv['x'],dv['y']]);fw/=np.linalg.norm(fw);err=rt@np.array([sp['position'][1]-lp['y'],sp['position'][0]-lp['x']]);acc=rt@np.array([sp['acceleration'][1],sp['acceleration'][0]])
  print(t,'body',np.round(b,3),'v',np.round(v,3),'cmd',np.round(vs,3),'poserr',np.round(err,3),'acc',np.round(acc,3),'fwd',np.round(fw,2),'speed',round(np.linalg.norm(v),3),'lateral',round(np.linalg.norm(v-fw*(v@fw)),3))
