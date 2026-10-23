"""MP4 of the recorded camera feed from a flight bag (record_camera:=true).

Plays at real speed: each JPEG on /rtabmap/image/compressed is held until the
next one arrives (the feed is ~14 Hz, tied to the VIO rate). A strip at the
bottom shows bag time (same t=0 as render_global.py) and the flight adapter
state from /planner/flight/status.
Usage (ROS sourced): python3 render_camera.py <bag_dir> <out.mp4> [threads]
"""
import io
import os
import subprocess
import sys
import tempfile

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from PIL import Image, ImageDraw, ImageFont
sys.path.append('/home/john/autonomous_drone_px4_vio/.venv-mavlink/lib/python3.12/site-packages')
import imageio_ffmpeg  # noqa: E402

bag, out = sys.argv[1], sys.argv[2]
threads = sys.argv[3] if len(sys.argv) > 3 else '2'
IMAGE, STATUS = '/rtabmap/image/compressed', '/planner/flight/status'

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
if IMAGE not in types:
    sys.exit(f'{bag} has no {IMAGE}; fly with record_camera:=true')
t0 = r.get_metadata().starting_time.nanoseconds
r.set_filter(rosbag2_py.StorageFilter(topics=[IMAGE, STATUS]))
frames, state = [], ''
while r.has_next():
    topic, data, ns = r.read_next()
    msg = deserialize_message(data, get_message(types[topic]))
    if topic == STATUS:
        state = msg.data.split(' ')[0] if msg.data else ''
    else:
        frames.append(((ns - t0) / 1e9, bytes(msg.data), state))
if len(frames) < 2:
    sys.exit('fewer than two camera frames')

try:
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 16)
except OSError:
    font = ImageFont.load_default()

with tempfile.TemporaryDirectory() as tmp:
    lines = []
    for i, (t, jpeg, st) in enumerate(frames):
        img = Image.open(io.BytesIO(jpeg)).convert('RGB')
        w, h = img.size
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, h - 24, w, h], fill=(0, 0, 0))
        draw.text((8, h - 21), f't={t:6.1f} s   {st}', fill=(255, 255, 255), font=font)
        path = os.path.join(tmp, f'{i:05d}.jpg')
        img.save(path, quality=92)
        duration = (frames[i + 1][0] - t) if i + 1 < len(frames) else 0.07
        lines += [f"file '{path}'", f'duration {max(duration, 0.001):.4f}']
    lines.append(f"file '{path}'")  # concat demuxer ignores the last duration otherwise
    listing = os.path.join(tmp, 'frames.txt')
    with open(listing, 'w') as handle:
        handle.write('\n'.join(lines) + '\n')
    subprocess.run([
        imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error',
        '-f', 'concat', '-safe', '0', '-i', listing,
        '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2', '-fps_mode', 'vfr',
        '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '23',
        '-threads', threads, '-movflags', '+faststart', out,
    ], check=True)
span = frames[-1][0] - frames[0][0]
print(f'{out}: {len(frames)} frames over {span:.1f} s ({len(frames) / span:.1f} Hz), '
      f'{os.path.getsize(out) / 1e6:.1f} MB')
