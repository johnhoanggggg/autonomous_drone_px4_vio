#!/usr/bin/env python3
"""Minimal holistic recorder: the flight mono pair (640x480 -> 640x400 @ fps) + IMU 200 Hz.
Prints a beep line every 5 s. Ctrl-C or --seconds ends it and writes the tar."""
import argparse, time, signal
import depthai as dai
ap = argparse.ArgumentParser()
ap.add_argument('--seconds', type=float, default=90)
ap.add_argument('--fps', type=float, default=20)
ap.add_argument('--out', required=True)
ap.add_argument('--encode', choices=['raw', 'h264', 'mjpeg'], default='raw')
a = ap.parse_args()
p = dai.Pipeline()
left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorResolution=(640, 480), sensorFps=a.fps)
right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorResolution=(640, 480), sensorFps=a.fps)
imu = p.create(dai.node.IMU)
imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
imu.setBatchReportThreshold(1); imu.setMaxBatchReports(10)
ql = left.requestOutput((640, 400)).createOutputQueue(maxSize=4, blocking=False)
qr = right.requestOutput((640, 400)).createOutputQueue(maxSize=4, blocking=False)
qi = imu.out.createOutputQueue(maxSize=50, blocking=False)
rc = dai.RecordConfig(); rc.outputDir = a.out
if a.encode == 'raw':
    rc.videoEncoding.enabled = False
else:
    rc.videoEncoding.enabled = True
    rc.videoEncoding.profile = (dai.VideoEncoderProperties.Profile.H264_HIGH if a.encode == 'h264'
                                else dai.VideoEncoderProperties.Profile.MJPEG)
    rc.videoEncoding.quality = 100
    rc.videoEncoding.lossless = a.encode == 'mjpeg'
p.enableHolisticRecord(rc)
stop = False
def h(*_):
    global stop; stop = True
signal.signal(signal.SIGINT, h); signal.signal(signal.SIGTERM, h)
p.start()
t0 = time.monotonic(); n = 0; last = 0
while p.isRunning() and not stop and time.monotonic() - t0 < a.seconds:
    n += len(ql.tryGetAll()); qr.tryGetAll(); qi.tryGetAll()
    el = time.monotonic() - t0
    if el - last >= 5:
        last = el; print(f'REC t={el:4.0f}s frames={n}', flush=True)
    time.sleep(0.005)
print(f'stopping after {time.monotonic() - t0:.1f}s, {n} left frames', flush=True)
p.stop()
print('done', flush=True)
