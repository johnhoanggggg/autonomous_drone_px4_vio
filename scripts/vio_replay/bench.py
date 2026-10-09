#!/usr/bin/env python3
"""Standalone stationary/carry VIO bench (no ROS): BasaltVIO or RTABMapVIO on the
flight pipeline (640x480 mono -> 640x400, StereoDepth, IMU 200 Hz). Logs every pose.

Live:    python3 bench.py --backend basalt --cfg none --seconds 60 --out runs/x
Replay:  python3 record.py --seconds 90 --out ~/vio_recordings        (cameras + IMU, raw)
         python3 bench.py --backend basalt --direct --replay REC.tar --seconds <LEN-1> --out runs/x
         python3 bench.py --backend rtabmap --replay REC.tar --seconds <LEN-1> --out runs/y
         python3 analyze.py runs/x runs/y
--cfg is a JSON dict of Basalt VioConfig overrides, loaded through setConfigPath (reaches
optical_flow_matching_guess_type, which setConfig() from Python cannot). --accel-nd/--gyro-nd
are CONTINUOUS noise densities (sigma*sqrt(Hz)); Basalt multiplies by sqrt(200) itself.
--seconds must be shorter than the recording: replay loops and Basalt aborts on the
non-monotonic timestamp. --direct (Basalt only) skips the device round trip: ~19.5 Hz, repeatable.
Pose TransformData carries no timestamp; latency comes from the passthrough image (*.ages.npy)."""
import argparse, json, os, time, threading
import numpy as np
import depthai as dai

HERE = os.path.dirname(os.path.abspath(__file__))
# DepthAI setDefaultVIOConfig() values (BasaltVIO.cpp), all 66 serialized keys.
DEFAULTS = dict(
    optical_flow_type="frame_to_frame", optical_flow_detection_grid_size=50,
    optical_flow_detection_num_points_cell=1, optical_flow_detection_min_threshold=5,
    optical_flow_detection_max_threshold=40, optical_flow_detection_nonoverlap=True,
    optical_flow_max_recovered_dist2=0.04, optical_flow_pattern=51, optical_flow_max_iterations=5,
    optical_flow_epipolar_error=0.005, optical_flow_levels=3, optical_flow_skip_frames=1,
    optical_flow_matching_guess_type="REPROJ_AVG_DEPTH", optical_flow_matching_default_depth=2.0,
    optical_flow_image_safe_radius=472.0, optical_flow_recall_enable=False,
    optical_flow_recall_all_cams=False, optical_flow_recall_num_points_cell=True,
    optical_flow_recall_over_tracking=False, optical_flow_recall_update_patch_viewpoint=False,
    optical_flow_recall_max_patch_dist=3.0, optical_flow_recall_max_patch_norms=[1.74, 0.96, 0.99, 0.44],
    vio_linearization_type="ABS_QR", vio_sqrt_marg=True, vio_max_states=3, vio_max_kfs=7,
    vio_min_frames_after_kf=5, vio_new_kf_keypoints_thresh=0.7, vio_debug=False,
    vio_extended_logging=False, vio_obs_std_dev=0.5, vio_obs_huber_thresh=1.0,
    vio_min_triangulation_dist=0.05, vio_max_iterations=7, vio_enforce_realtime=False,
    vio_use_lm=True, vio_lm_lambda_initial=1e-4, vio_lm_lambda_min=1e-6, vio_lm_lambda_max=1e2,
    vio_scale_jacobian=False, vio_init_pose_weight=1e8, vio_init_ba_weight=1e1,
    vio_init_bg_weight=1e2, vio_marg_lost_landmarks=True, vio_fix_long_term_keyframes=False,
    vio_kf_marg_feature_ratio=0.1, vio_kf_marg_criteria="KF_MARG_DEFAULT",
    mapper_obs_std_dev=0.25, mapper_obs_huber_thresh=1.5, mapper_detection_num_points=800,
    mapper_num_frames_to_match=30, mapper_frames_to_match_threshold=0.04, mapper_min_matches=20,
    mapper_ransac_threshold=5e-5, mapper_min_track_length=5, mapper_max_hamming_distance=70,
    mapper_second_best_test_ratio=1.2, mapper_bow_num_bits=16, mapper_min_triangulation_dist=0.07,
    mapper_no_factor_weights=False, mapper_use_factors=True, mapper_use_lm=True,
    mapper_lm_lambda_min=1e-32, mapper_lm_lambda_max=1e3)


def write_config(overrides, path):
    keys = [k.strip()[len('config.'):] for k in open(os.path.join(HERE, 'keys.txt'))]
    cfg = dict(DEFAULTS); cfg.update(overrides)
    missing = set(keys) - set(cfg); extra = set(cfg) - set(keys)
    assert not missing and not extra, (missing, extra)
    json.dump({'value0': {'config.' + k: cfg[k] for k in keys}}, open(path, 'w'), indent=1)


def proc_cpu():
    f = open('/proc/self/stat').read().rsplit(')', 1)[1].split()
    return (int(f[11]) + int(f[12])) / os.sysconf('SC_CLK_TCK')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--backend', choices=['basalt', 'rtabmap'], default='basalt')
    ap.add_argument('--seconds', type=float, default=60)
    ap.add_argument('--fps', type=float, default=20)
    ap.add_argument('--cfg', default='{}', help='JSON dict of VioConfig overrides')
    ap.add_argument('--accel-nd', type=float, default=None, help='accel noise density (continuous std)')
    ap.add_argument('--gyro-nd', type=float, default=None, help='gyro noise density (continuous std)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--direct', action='store_true', help='basalt: cameras -> BasaltVIO directly, host sync, no StereoDepth')
    ap.add_argument('--record', default='', help='dir: holistic record, raw (lossless) video')
    ap.add_argument('--replay', default='', help='recording .tar to replay instead of the live cameras/IMU')
    a = ap.parse_args()

    p = dai.Pipeline()
    if True:
        left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorResolution=(640, 480), sensorFps=a.fps)
        right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorResolution=(640, 480), sensorFps=a.fps)
        imu = p.create(dai.node.IMU)
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
        imu.setBatchReportThreshold(1); imu.setMaxBatchReports(10)
        stereo = p.create(dai.node.StereoDepth) if not a.direct else None
        if stereo is not None:
            stereo.setExtendedDisparity(False); stereo.setLeftRightCheck(True); stereo.setSubpixel(True)
            stereo.setRectifyEdgeFillColor(0); stereo.enableDistortionCorrection(True)
            stereo.initialConfig.setLeftRightCheckThreshold(10); stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)
        lo = left.requestOutput((640, 400)); ro = right.requestOutput((640, 400))
        if not a.direct:
            lo.link(stereo.left); ro.link(stereo.right)
        if a.backend == 'basalt':
            odom = p.create(dai.node.BasaltVIO)
            odom.setImuUpdateRate(200)
            cfgpath = a.out + '.vio.json'
            write_config({} if a.cfg == 'none' else json.loads(a.cfg), cfgpath)
            if a.cfg != 'none':
                odom.setConfigPath(cfgpath)
            if a.accel_nd is not None:
                odom.setAccelNoiseStd([a.accel_nd ** 2] * 3)   # depthai sqrt()s it
            if a.gyro_nd is not None:
                odom.setGyroNoiseStd([a.gyro_nd ** 2] * 3)
            if a.direct:
                odom.runSyncOnHost(True); lo.link(odom.left); ro.link(odom.right)
            else:
                stereo.syncedLeft.link(odom.left); stereo.syncedRight.link(odom.right)
        else:
            ft = p.create(dai.node.FeatureTracker)
            ft.setHardwareResources(1, 2)
            ft.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
            ft.initialConfig.setNumTargetFeatures(500)
            ft.initialConfig.setOpticalFlow(); ft.initialConfig.setMotionEstimator(True)
            ft.initialConfig.FeatureMaintainer.minimumDistanceBetweenFeatures = 49
            odom = p.create(dai.node.RTABMapVIO)
            odom.setUseFeatures(True)
            odom.setParams({'Odom/ResetCountdown': '1'})
            stereo.rectifiedLeft.link(ft.inputImage)
            ft.passthroughInputImage.link(odom.rect)
            stereo.depth.link(odom.depth)
            ft.outputFeatures.link(odom.features)
        imu.out.link(odom.imu)
        if a.record:
            rc = dai.RecordConfig(); rc.outputDir = a.record; rc.videoEncoding.enabled = False
            p.enableHolisticRecord(rc)
        if a.replay:
            p.enableHolisticReplay(a.replay)
        q = odom.transform.createOutputQueue(maxSize=200, blocking=False)
        if not a.direct:
            dq = stereo.depth.createOutputQueue(maxSize=1, blocking=False)
        if a.backend == 'basalt':
            pq = odom.passthrough.createOutputQueue(maxSize=200, blocking=False)
        else:
            pq = odom.passthroughRect.createOutputQueue(maxSize=200, blocking=False)
        ages = []
        p.start()
        rows = []
        c0 = proc_cpu(); t0 = time.monotonic()
        while p.isRunning() and time.monotonic() - t0 < a.seconds:
            for im in pq.tryGetAll():
                ages.append((dai.Clock.now() - im.getTimestamp()).total_seconds())
            m = q.tryGet()
            if m is None:
                time.sleep(0.002); continue
            tr = m.getTranslation(); qq = m.getQuaternion()
            try:
                ts = m.getTimestamp().total_seconds()
            except Exception:
                ts = float('nan')
            lat = (dai.Clock.now() - m.getTimestamp()).total_seconds()
            rows.append((time.monotonic() - t0, lat, tr.x, tr.y, tr.z, qq.qx, qq.qy, qq.qz, qq.qw))
        
        cpu = (proc_cpu() - c0) / (time.monotonic() - t0) * 100
    np.save(a.out + '.npy', np.array(rows))
    np.save(a.out + '.ages.npy', np.array(ages))
    json.dump({'cpu_pct': cpu, 'args': vars(a)}, open(a.out + '.meta.json', 'w'))
    print(f'{len(rows)} poses, cpu {cpu:.0f}%', flush=True)
    os._exit(0)


if __name__ == '__main__':
    import traceback, sys
    try:
        main()
    except BaseException:
        traceback.print_exc(); sys.stderr.flush(); os._exit(1)
