#!/usr/bin/env python3
"""Offline ROS-bag extraction for the three heading-validation flights on 2026-09-08.

Run from repository root after sourcing ROS and the workspace. Uses causal
receipt-time pairing; map normals point toward the nearest occupied cell.
No ROS publishers or vehicle connections. Onboard control pairing is handled
separately by analyze_control_response.py.
"""
import sys,json,math
from pathlib import Path
from bisect import bisect_right
import numpy as np
sys.path.insert(0,'scripts')
from analyze_yaw_tracking import read,TOPICS,yaw_of
from render_flight_map import Grid
extra=['/planner/flight/config','/planner/flight/heading_debug','/planner/follower/status','/planner/flight/command_speed','/planner/follower/remaining','/fmu/out/battery_status_v1','/planner/follower/config','/planner/path','/rtabmap/grid','/rtabmap/odom_correction','/fmu/in/vehicle_command','/planner/follower/valid']
allout={}
for stamp in ['032407','032454','032636']:
 d,_=read(f'ros2_ws/flight_logs/offboard_global_20260908T{stamp}Z',list(TOPICS)+extra)
 t0=min(d[n][0][0] for n in TOPICS if d[n]); streams={n:[t for t,m in v] for n,v in d.items() if n!='_decode_errors'}
 def at(n,t,age=1):
  i=bisect_right(streams[n],t)-1
  return (d[n][i][1],(t-streams[n][i])/1e9) if i>=0 and t-streams[n][i]<=age*1e9 else (None,None)
 offsets=[(t-t0)/1e9-m.timestamp/1e6 for t,m in d['/fmu/out/vehicle_local_position_v1']]
 rows=[];grid_cache={}
 for t,lp in d['/fmu/out/vehicle_local_position_v1']:
  row=dict(time=(t-t0)/1e9,boot=lp.timestamp/1e6,x=lp.x,y=lp.y,z=lp.z,vx=lp.vx,vy=lp.vy,heading=lp.heading,ax=lp.ax,ay=lp.ay)
  for n,key in [('/planner/flight/status','flight'),('/planner/follower/status','follower')]:
   m,age=at(n,t);row[key]=m.data if m else None
  for n,key in [('/planner/flight/command_speed','command_speed'),('/rtabmap/vio_feature_count','features'),('/planner/follower/remaining','remaining')]:
   m,age=at(n,t);row[key]=m.data if m else None
  m,_=at('/fmu/in/trajectory_setpoint',t,.2)
  if m:
   row.update(spx=float(m.position[0]),spy=float(m.position[1]),spvx=float(m.velocity[0]),spvy=float(m.velocity[1]),spax=float(m.acceleration[0]),spay=float(m.acceleration[1]),spyaw=float(m.yaw))
  p,age=at('/rtabmap/body_pose',t,.25);g,ga=at('/rtabmap/grid',t,3);c,ca=at('/rtabmap/odom_correction',t,1)
  if p and g and c:
   key=id(g)
   if key not in grid_cache:grid_cache[key]=Grid(g)
   grid=grid_cache[key];px,py=p.pose.position.x,p.pose.position.y;cyaw=yaw_of(c)
   row.update(mapx=px,mapy=py,pose_age=age,grid_age=ga,correction_yaw=cyaw,clearance=grid.point_clearance(px,py))
   ccx,ccy=grid.world_to_cell(px,py);span=34
   x0,x1=max(0,ccx-span),min(grid.width,ccx+span+1);y0,y1=max(0,ccy-span),min(grid.height,ccy+span+1)
   ys,xs=np.nonzero(grid.data[y0:y1,x0:x1]>=65)
   if len(xs):
    left=grid.origin_x+(xs+x0)*grid.resolution;bottom=grid.origin_y+(ys+y0)*grid.resolution
    wx=np.clip(px,left,left+grid.resolution);wy=np.clip(py,bottom,bottom+grid.resolution);dist=np.hypot(wx-px,wy-py);i=np.argmin(dist)
    if dist[i]>1e-5:
     nx,ny=float((wx[i]-px)/dist[i]),float((wy[i]-py)/dist[i]);cos,sin=math.cos(cyaw),math.sin(cyaw)
     # NED->map: R(correction)*[vy,vx]. Dot with map normal.
     nednx,nedny=-sin*nx+cos*ny,cos*nx+sin*ny
     row.update(wallnx=nednx,wallny=nedny,wall_v=nednx*lp.vx+nedny*lp.vy,wallx=float(wx[i]),wally=float(wy[i]))
  rows.append(row)
 headings=[dict(time=(t-t0)/1e9,**json.loads(m.data)) for t,m in d['/planner/flight/heading_debug']]
 commands=[dict(time=(t-t0)/1e9,command=m.command,param1=m.param1,param2=m.param2) for t,m in d['/fmu/in/vehicle_command']]
 events={n:[dict(time=(t-t0)/1e9,text=m.data) for t,m in d[n]] for n in ['/planner/flight/status','/planner/follower/status']}
 paths=[dict(time=(t-t0)/1e9,points=[[p.pose.position.x,p.pose.position.y] for p in m.poses]) for t,m in d['/planner/path']]
 batteries=[dict(time=(t-t0)/1e9,voltage=m.voltage_v,current=m.current_a,remaining=m.remaining) for t,m in d['/fmu/out/battery_status_v1']]
 lastg=d['/rtabmap/grid'][-1][1]
 grid=dict(data=list(lastg.data),width=lastg.info.width,height=lastg.info.height,res=lastg.info.resolution,x=lastg.info.origin.position.x,y=lastg.info.origin.position.y)
 allout[stamp]=dict(t0=t0,offset=float(np.median(offsets)),rows=rows,headings=headings,commands=commands,events=events,paths=paths,battery=batteries,grid=grid)
 print(stamp,'offset',np.median(offsets),'headingholdticks',sum(x['holding'] for x in headings),'landcommands',[x for x in commands if x['command'] in [21,400]])
 print('follow transitions:')
 prev=None
 for e in events['/planner/follower/status']:
  st=e['text'].split()[0]
  if st!=prev:print(round(e['time'],2),e['text']);prev=st
Path('reports/heading_flights_20260908_bags.json').write_text(json.dumps(allout))
