from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
fig,axes=plt.subplots(1,2,figsize=(14,7))
for ax,stamp,lims,ts in zip(axes,['071623','071845'],[(.5,1.95,-2.3,.25),(1.5,4.6,-.55,.55)],[[20.75,21.28,22.26,27.75],[14.23,15.24,17.23,21.24]]):
 b=json.loads((ROOT/f'ros2_ws/flight_logs/offboard_global_20260916T{stamp}Z/render/bag_data.json').read_text());g=b['final_grid'];arr=np.array(g['data']).reshape(g['height'],g['width']);im=np.ones(arr.shape)*.95;im[arr<0]=.65;im[arr>=65]=.15
 ax.imshow(im,origin='lower',cmap='gray',vmin=0,vmax=1,extent=[g['origin_x'],g['origin_x']+g['width']*g['resolution'],g['origin_y'],g['origin_y']+g['height']*g['resolution']]);p=b['/rtabmap/body_pose'];ax.plot([r['x'] for r in p],[r['y'] for r in p],color='tab:red',label='corrected body track')
 for t in ts:
  paths=[r for r in b['/planner/path'] if r['t']<=t]
  if paths:
   path=paths[-1];v=np.array(path['xy']);ax.plot(v[:,0],v[:,1],'.--',label=f'published route t={path["t"]:.2f}')
 for t in ([20.8,21.2,22.2,23.2,28.5] if stamp=='071623' else [15,16.4,18.5,20,23,29.5]):
  r=min(p,key=lambda r:abs(r['t']-t));ax.annotate(f'{t:.1f}',(r['x'],r['y']),fontsize=8);ax.scatter(r['x'],r['y'],s=10,color='tab:red')
 ax.set_xlim(*lims[:2]);ax.set_ylim(*lims[2:]);ax.set_aspect('equal');ax.set_title(stamp+' (final occupancy for context only)');ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)');ax.legend(fontsize=8)
fig.tight_layout();fig.savefig(ROOT/'reports/lateral_excursions_20260916/routes.png',dpi=150)
