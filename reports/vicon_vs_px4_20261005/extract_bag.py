import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag=sys.argv[1]; out=sys.argv[2]
want={"/fmu/out/vehicle_local_position_v1","/fmu/in/vehicle_visual_odometry","/fmu/out/vehicle_odometry",
      "/fmu/out/vehicle_status_v1","/rtabmap/vio_pose","/waypoint/target","/fmu/out/vehicle_attitude","/fmu/in/trajectory_setpoint"}
r=rosbag2_py.SequentialReader()
import glob; mc=glob.glob(bag+"/*.mcap")[0]
r.open(rosbag2_py.StorageOptions(uri=mc,storage_id="mcap"),rosbag2_py.ConverterOptions("",""))
types={t.name:t.type for t in r.get_all_topics_and_types()}
D={k:[] for k in want}
while r.has_next():
    tp,data,ts=r.read_next()
    if tp not in want: continue
    m=deserialize_message(data,get_message(types[tp])); tb=ts*1e-9
    if tp.endswith("vehicle_local_position_v1"):
        D[tp].append([tb,m.timestamp*1e-6,m.x,m.y,m.z,m.heading,m.xy_valid,m.z_valid,m.vx,m.vy,m.vz])
    elif tp.endswith("odometry"):
        D[tp].append([tb,m.timestamp*1e-6,m.timestamp_sample*1e-6,*m.position,*m.q])
    elif tp.endswith("vehicle_status_v1"):
        D[tp].append([tb,m.timestamp*1e-6,m.arming_state,m.nav_state])
    elif tp.endswith("vehicle_attitude"):
        D[tp].append([tb,m.timestamp*1e-6,*m.q])
    elif tp.endswith("trajectory_setpoint"):
        D[tp].append([tb,m.timestamp*1e-6,*m.position,m.yaw])
    else:
        p=m.pose.position;q=m.pose.orientation
        D[tp].append([tb,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
np.savez(out,**{k.strip("/").replace("/","__"):np.array(v) for k,v in D.items()})
print(out,{k:len(v) for k,v in D.items()})
