import sys, json, os, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag,storage_id="mcap"),rosbag2_py.ConverterOptions("",""))
types={t.name:t.type for t in r.get_all_topics_and_types()}
t0=None; grids=[]; paths=[]; poses=[]; route_t=None; goal_t=None; fcfg=None; goal=None
while r.has_next():
    tp,d,st=r.read_next()
    if t0 is None: t0=st
    t=(st-t0)/1e9
    if tp not in ("/rtabmap/grid","/planner/path","/rtabmap/body_pose","/planner/flight/status","/planner/follower/config","/waypoint/clicked"): continue
    m=deserialize_message(d,get_message(types[tp]))
    if tp=="/rtabmap/grid":
        i=len(grids); fn=f"grid_{i}.bin"
        np.array(m.data,dtype=np.int8).tofile(os.path.join(out,fn))
        grids.append(dict(t=t,file=fn,width=m.info.width,height=m.info.height,resolution=m.info.resolution,origin_x=m.info.origin.position.x,origin_y=m.info.origin.position.y))
    elif tp=="/planner/path":
        pts=[[p.pose.position.x,p.pose.position.y] for p in m.poses]
        if len(pts)>=2: paths.append(dict(t=t,points=pts))
    elif tp=="/rtabmap/body_pose":
        poses.append([t,m.pose.position.x,m.pose.position.y])
    elif tp=="/planner/flight/status":
        s=m.data
        if route_t is None and s.startswith("ROUTE"): route_t=t
        if goal_t is None and s.startswith("GOAL_REACHED"): goal_t=t
    elif tp=="/planner/follower/config": fcfg=json.loads(m.data)
    elif tp=="/waypoint/clicked": goal=[m.point.x,m.point.y]
json.dump(dict(grids=grids,paths=paths,poses=poses,route_t=route_t,goal_t=goal_t,goal=goal,bag_end=t,
  required_clearance=fcfg["robot_radius"]+fcfg["safety_margin"],occupied_threshold=fcfg["occupied_threshold"],lookahead=fcfg["lookahead"]),open(os.path.join(out,"meta.json"),"w"))
print(bag.split('/')[-1], len(grids),"grids",len(paths),"paths route_t",route_t,"goal_t",goal_t)
