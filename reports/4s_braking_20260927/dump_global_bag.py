import sys, numpy as np
sys.path.insert(0,'scripts')
import render_flight_map as R
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, out = sys.argv[1], sys.argv[2]
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag,storage_id="mcap"),rosbag2_py.ConverterOptions("",""))
types={t.name:t.type for t in r.get_all_topics_and_types()}
t0=None; grid=None; pose=[]; lp=[]; paths=[]; goal=None; fol=[]; cmd=[]
while r.has_next():
    tp,d,st=r.read_next()
    if t0 is None: t0=st
    t=(st-t0)/1e9
    if tp not in ("/rtabmap/grid","/rtabmap/body_pose","/fmu/out/vehicle_local_position_v1","/planner/path","/waypoint/clicked","/planner/follower/status","/fmu/in/trajectory_setpoint"): continue
    m=deserialize_message(d,get_message(types[tp]))
    if tp=="/rtabmap/grid": grid=m
    elif tp=="/rtabmap/body_pose": pose.append((t,m.pose.position.x,m.pose.position.y))
    elif tp=="/fmu/out/vehicle_local_position_v1": lp.append((t,m.timestamp,m.x,m.y,m.z))
    elif tp=="/planner/path": paths.append((t,[(p.pose.position.x,p.pose.position.y) for p in m.poses]))
    elif tp=="/waypoint/clicked": goal=(m.point.x,m.point.y)
    elif tp=="/planner/follower/status": fol.append((t,R.state_of(m.data)))
    elif tp=="/fmu/in/trajectory_setpoint": cmd.append((t,m.position[0],m.position[1]))
g=R.Grid(grid); c=R.Canvas(g,1,0.25,3,65)
img=np.array(c.image)
# first path installed after takeoff and the last path
np.savez(out,img=img,origin=[g.origin_x,g.origin_y],res=g.resolution,wh=[g.width,g.height],
         pose=np.array(pose),lp=np.array(lp,dtype=float),goal=np.array(goal),
         path_last=np.array(paths[-1][1]),path_first=np.array(paths[0][1]),
         fol_t=np.array([f[0] for f in fol]),fol_s=np.array([f[1] for f in fol]))
print(len(pose),len(lp),len(paths),goal, 'bag end %.2f'%t)
