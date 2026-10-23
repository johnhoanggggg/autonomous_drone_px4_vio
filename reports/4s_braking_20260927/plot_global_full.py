import numpy as np, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from pyulog import ULog
S='reports/4s_braking_20260927'
B=np.load(f'{S}/global_20260928T072523Z.npz')
u=ULog('ros2_ws/flight_logs/px4_20260928_07_07_47.ulg'); t0=int(u.start_timestamp)
D={x.name:x.data for x in u.data_list if x.multi_id==0}
T=lambda a:(a['timestamp'].astype(np.int64)-t0)/1e6
lp=D['vehicle_local_position']; sp=D['trajectory_setpoint']; st=D['vehicle_status']; acc=D['vehicle_acceleration']
tl=T(lp)
# --- transform NED(x,y) -> map(x,y) by rigid fit: bag body_pose vs bag local position (same PX4 clock via px4 timestamp)
blp=B['lp']; pose=B['pose']
bx=np.interp(pose[:,0],blp[:,0],blp[:,2]); by=np.interp(pose[:,0],blp[:,0],blp[:,3])
P=np.c_[by,bx]; Q=pose[:,1:3]
Pc,Qc=P.mean(0),Q.mean(0); H=(P-Pc).T@(Q-Qc); U,_,Vt=np.linalg.svd(H); Rm=Vt.T@U.T
if np.linalg.det(Rm)<0: Vt[1]*=-1; Rm=Vt.T@U.T
tr=Qc-Rm@Pc; res=np.linalg.norm((Rm@P.T).T+tr-Q,axis=1)
print('fit rot %.1f deg, residual median %.3f max %.3f m'%(np.degrees(np.arctan2(Rm[1,0],Rm[0,0])),np.median(res),res.max()))
tomap=lambda x,y:(Rm@np.c_[y,x].T).T+tr
# ULog time of bag end: px4 timestamp in bag lp
bag_end_ulog=(blp[-1,1]-t0)/1e6; bag_arm_off=(blp[0,1]-t0)/1e6-blp[0,0]
print('bag ends at ULog t=%.2f'%bag_end_ulog)
ts=T(st); armed=st['arming_state']==2; ta=ts[armed][0] if ts[armed][0]>-1 else 0; tb=ts[armed][-1]
tland=ts[np.argmax(st['nav_state']==18)]
m=(tl>=-0.1)&(tl<=tb+0.6)
xy=tomap(lp['x'][m],lp['y'][m]); tt=tl[m]
tsp=T(sp); ms=(tsp>=0)&(tsp<=tb)&np.isfinite(sp['position[0]'])
sxy=tomap(sp['position[0]'][ms],sp['position[1]'][ms])
# --- figure
fig=plt.figure(figsize=(13,11.5),dpi=110)
gs=fig.add_gridspec(4,1,height_ratios=[3.3,1,1,1],hspace=0.32)
ax=fig.add_subplot(gs[0])
ox,oy=B['origin']; res_=float(B['res']); w,h=B['wh']
ax.imshow(B['img'],extent=[ox,ox+w*res_,oy,oy+h*res_],interpolation='nearest')
pad=0.6; lo=np.minimum(xy.min(0),B['path_first'].min(0))-pad; hi=np.maximum(xy.max(0),B['path_first'].max(0))+pad
ax.set_xlim(lo[0],hi[0]); ax.set_ylim(lo[1],hi[1])
ax.plot(*B['path_first'].T,color='#1f6fd1',lw=1.4,ls='--',label='planned route (first)')
ax.plot(sxy[:,0],sxy[:,1],color='#7a7a7a',lw=1,alpha=0.9,label='command sent to PX4')
inb=tt<=bag_end_ulog
ax.plot(xy[inb,0],xy[inb,1],color='#1b9e4b',lw=2.4,label='flown (bag + ULog)')
ax.plot(xy[~inb,0],xy[~inb,1],color='#b3279a',lw=2.4,label='flown (ULog only: bag lost tail)')
g=B['goal']; ax.plot(*g,'o',ms=11,mfc='white',mec='black',label='goal')
ax.plot(*xy[0],'^',ms=10,color='#1b9e4b',mec='black',label='takeoff')
il=np.searchsorted(tt,tland); ax.plot(*xy[il],'v',ms=10,color='#d9480f',mec='black',label='AUTO.LAND')
# time ticks every 5 s
for k in range(5,int(tb)+1,5):
    i=np.searchsorted(tt,k); ax.plot(*xy[i],'.',color='black',ms=5); ax.annotate(f'{k}s',xy[i],xytext=(4,4),textcoords='offset points',fontsize=8)
ax.set_aspect('equal'); ax.set_xlabel('map x (m)'); ax.set_ylabel('map y (m)')
ax.legend(loc='lower right',fontsize=8,framealpha=0.9)
ax.set_title('Global flight 07:25:23Z (ULog 07_07_47) — goal (2.22, −0.64), 5.6 m route, C++ adapter, 0.20 m/s\n'
             'tune: ROLL/PITCH_P 6.5, VEL_P 2.2, VEL_I 0.9, Z_VEL_I 3.0, HOVER 0.38, X_OFF −4.66 · shaded = 0.25 m clearance band',fontsize=10)
# time series
def shade(a):
    a.axvline(bag_end_ulog,color='#b3279a',ls=':',lw=1); a.axvline(tland,color='#d9480f',ls=':',lw=1)
    a.set_xlim(0,tb+1); a.grid(alpha=0.3)
a1=fig.add_subplot(gs[1]); a1.plot(tl,-lp['z'],color='#1b9e4b',label='altitude'); a1.plot(tsp[ms],-sp['position[2]'][ms],color='#7a7a7a',lw=1,label='setpoint')
a1.set_ylabel('alt (m)'); a1.set_ylim(-0.05,0.4); a1.legend(fontsize=8,loc='lower center',ncol=2); shade(a1)
a1.text(bag_end_ulog,0.36,' bag ends',color='#b3279a',fontsize=8); a1.text(tland,0.36,' LAND',color='#d9480f',fontsize=8)
a2=fig.add_subplot(gs[2]); a2.plot(tl,np.hypot(lp['vx'],lp['vy']),color='#1f6fd1',label='horizontal speed'); a2.axhline(0.2,color='#7a7a7a',ls='--',lw=1,label='route speed 0.20')
a2.set_ylabel('m/s'); a2.legend(fontsize=8,loc='upper left'); shade(a2)
px=np.interp(tsp,tl,lp['x']); py=np.interp(tsp,tl,lp['y']); e=np.hypot(sp['position[0]']-px,sp['position[1]']-py)
a3=fig.add_subplot(gs[3]); a3.plot(tsp[ms],e[ms]*100,color='#d9480f',label='command − position (cm)')
ta_=T(acc); a3b=a3.twinx(); a3b.plot(ta_,np.hypot(acc['xyz[0]'],acc['xyz[1]']),color='black',lw=0.6,alpha=0.5); a3b.set_ylabel('|a_xy| m/s²',fontsize=8); a3b.set_ylim(0,14)
a3.set_ylabel('cm'); a3.set_xlabel('time since arm (s)'); a3.legend(fontsize=8,loc='upper left'); shade(a3)
fig.savefig(f'{S}/global_20260928T072523Z_full.png',bbox_inches='tight')
mm=ms&(tsp>4)&(tsp<tland)
print('track err median %.1f p95 %.1f max %.1f cm'%(np.median(e[mm])*100,np.percentile(e[mm],95)*100,e[mm].max()*100))
gm=tomap(lp['x'][tl<tland][-1:],lp['y'][tl<tland][-1:])[0]; print('pos at LAND map',gm,'goal',g,'dist %.3f'%np.hypot(*(gm-g)))
