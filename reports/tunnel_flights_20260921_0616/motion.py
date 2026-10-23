from flight_data import *
for f,intervals in [('061515',[(23,24.4),(24.4,25.8),(25.8,26.9)]),('061647',[(28,31)])]:
 F=Flight(f);print(f)
 for a,b in intervals:
  samples=[(t,m) for t,m in F.d[LP] if a<=t<=b];p0=F.at(LP,a);p1=F.at(LP,b);r=F.frame(b)[1];lpdelta=r@np.array([p1['y']-p0['y'],p1['x']-p0['x']]);vd=r@np.trapezoid(np.array([[m['vy'],m['vx']] for t,m in samples]),[t for t,m in samples],axis=0)
  v0=F.at('/rtabmap/vio_pose',a);v1=F.at('/rtabmap/vio_pose',b);rawdelta=r@(pose(v1)-pose(v0))
  print(a,b,'LP delta',np.round(lpdelta,3),'velocity integral',np.round(vd,3),'VIO delta',np.round(rawdelta,3),'offset change',np.round(lpdelta-rawdelta,3))
