"""Camera MP4 for ROS-backend 3D flights (offboard_planner_3d_unknown bags): /oak/left/image_rect/compressed
at its recorded rate, H.264, with a flight-time + /planner3d/flight/status state/fault overlay.
For DepthAI-backend bags (/rtabmap/image/compressed) use render_camera.py instead.
Usage (system cv2 needs system numpy): PYTHONNOUSERSITE=1 python3 render_camera_3d.py <bag_dir> <out.mp4>
"""
import sys, json, bisect, numpy as np, cv2, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
bag, out = sys.argv[1], sys.argv[2]
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/oak/left/image_rect/compressed','/planner3d/flight/status']))
frames=[]; st_t=[]; st=[]
while r.has_next():
    tp,d,t=r.read_next()
    if tp.endswith('compressed'): m=deserialize_message(d,CompressedImage); frames.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9, bytes(m.data)))
    else:
        s=json.loads(deserialize_message(d,String).data); st_t.append(t/1e9); st.append((s.get('state',''),s.get('fault') or ''))
frames.sort(); t0=frames[0][0]; fps=(len(frames)-1)/(frames[-1][0]-t0)
img=cv2.imdecode(np.frombuffer(frames[0][1],np.uint8),cv2.IMREAD_COLOR); h,w=img.shape[:2]
for cc in ('avc1','mp4v'):
    vw=cv2.VideoWriter(out,cv2.VideoWriter_fourcc(*cc),fps,(w,h))
    if vw.isOpened(): break
for ts,data in frames:
    img=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
    i=bisect.bisect_right(st_t,ts)-1; label='%5.1f s  %s %s'%(ts-t0,*(st[i] if i>=0 else ('','')))
    cv2.rectangle(img,(0,0),(w,30),(0,0,0),-1); cv2.putText(img,label,(8,21),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),1,cv2.LINE_AA)
    vw.write(img)
vw.release(); print('codec',cc,'frames',len(frames),'%.2f fps'%fps,'size %dx%d'%(w,h),'duration %.1f s'%(frames[-1][0]-t0))
