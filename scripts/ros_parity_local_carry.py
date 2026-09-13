#!/usr/bin/env python3
"""Locally cued cache-enabled carry, optionally with raw inputs. No PX4 node is launched."""
import argparse
import hashlib
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path('/home/john/autonomous_drone_px4_vio')


def archive_capture(source, destination):
    """Copy to persistent storage, verify every file, then publish the directory.

    Never remove the source. A failed archive leaves RAM and partial disk copies.
    """
    partial = destination.with_name(destination.name + '.partial')
    if destination.exists() or partial.exists():
        raise FileExistsError(f'Refusing existing archive: {destination} or {partial}')
    shutil.copytree(source, partial)
    for path in source.rglob('*'):
        if not path.is_file():
            continue
        copied = partial / path.relative_to(source)
        def digest(file):
            with file.open('rb') as stream:
                return hashlib.file_digest(stream, 'sha256').digest()
        if digest(path) != digest(copied):
            raise RuntimeError(f'Archive checksum mismatch: {path}')
        with copied.open('rb') as stream:
            os.fsync(stream.fileno())
    partial.rename(destination)
    directory_fd = os.open(destination.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)
    return destination


def record_command(out, record_inputs=False):
    topics = ['/rtabmap/vio_odometry', '/rtabmap/vio_odom_info_lite', '/rtabmap/info', '/tf_static']
    command = ['ros2', 'bag', 'record', '--output', str(out/'bag'), '--storage', 'mcap',
               '--disable-keyboard-controls']
    if record_inputs:
        topics += ['/oak/left/image_rect', '/oak/stereo/image_raw', '/oak/left/camera_info',
                   '/oak/left_rect_feature_tracker/tracked_features', '/oak/imu/data']
        command += ['--qos-profile-overrides-path',
                    str(ROOT/'ros2_ws/src/px4_vio_bridge/config/oak_parity_bag_qos.yaml')]
    return command + ['--topics', *topics]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, help='New result directory (default: timestamped under /home/john)')
    parser.add_argument('--record-inputs', action='store_true',
                        help='Also record raw RGB/depth/calibration/features/IMU for native replay; requires 3 GiB free')
    parser.add_argument('--ram', action='store_true', help='Record into /dev/shm, then verify/copy to --out after shutdown')
    parser.add_argument('--stationary', action='store_true', help='Validate recording while stationary; not a motion-parity test')
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error('Run directly in an interactive terminal, not through chat or a pipe.')
    required_free = 3 * 1024**3 if args.record_inputs else 256 * 1024**2
    out = (args.out or Path('/home/john') / ('ros_parity_local_carry_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))).resolve()
    ancestor = out.parent
    while not ancestor.exists():
        ancestor = ancestor.parent
    if shutil.disk_usage(ancestor).free < required_free:
        parser.error(f'Need at least {required_free / 1024**3:.2f} GiB free on the output filesystem; no files will be deleted.')
    if args.ram and ancestor.stat().st_dev == Path('/dev/shm').stat().st_dev:
        parser.error('--out must be persistent storage, not the RAM staging filesystem.')
    prefix = subprocess.check_output(['ros2', 'pkg', 'prefix', 'rtabmap_odom'], text=True).strip()
    if prefix != '/home/john/rtabmap_latest_frame_overlay/install/rtabmap_odom':
        parser.error('Use run_ros_parity_carry.sh to source the isolated overlay.')
    import psutil
    if args.ram and (shutil.disk_usage('/dev/shm').free < required_free or
                     psutil.virtual_memory().available < required_free + 1024**3):
        parser.error('Insufficient /dev/shm capacity or available RAM (capture budget plus 1 GiB reserve).')
    for process in psutil.process_iter(['pid', 'cmdline']):
        command = process.info['cmdline'] or []
        if any('component_container' in Path(arg).name or
               Path(arg).name in ('px4_vio_bridge_node', 'vio_bridge_node', 'ros_rtabmap_replay_frames.py', 'depthai_rtabmap_bag.py')
               for arg in command):
            parser.error(f'Existing ROS/device process {process.pid}; stop the other graph first.')
    out.mkdir(parents=True, exist_ok=False)
    archive = out
    if args.ram:
        out = Path(tempfile.mkdtemp(prefix='ros_parity_carry_', dir='/dev/shm'))
    env = os.environ.copy()
    env['RTABMAP_CACHE_CAMERA_INFO'] = '1'
    env.pop('RTABMAP_TIMING_AUDIT', None)  # Avoid high-volume per-frame disk logging.
    env['PYTHONUNBUFFERED'] = '1'
    launch_cmd = ['ros2', 'launch', 'px4_vio_bridge', 'ros_rtabmap_oak_v3_native_features.launch.py',
        'foxglove:=false', 'use_imu:=false', 'odometry_rate_hz:=0.0', 'odom_reset_countdown:=0',
        'keep_latest_frame:=true', 'odom_key_frame_thr:=0.3', 'odom_vis_keyframe_thr:=150',
        'odom_config_path:=/home/john/ros_parity_optimizer_audit_20260913/host_projection_pnp3.ini',
        'camera_config:=' + str(ROOT/'ros2_ws/src/px4_vio_bridge/config/oak_v3_rtabmap_tracked.yaml')]
    record_cmd = record_command(out, args.record_inputs)
    manifest = dict(launch=launch_cmd, record=record_cmd, cache_camera_info=True,
        timing_audit=False, raw_inputs_recorded=args.record_inputs, ram_recording=args.ram,
        stationary_only=args.stationary, persistent_directory=str(archive),
        px4_node_launched=False, complete=False, cues=[])
    (out/'commands.json').write_text(json.dumps(manifest, indent=2)+'\n')
    processes = []
    handles = []

    def start(command, name):
        handle = (out/(name+'.log')).open('w')
        handles.append(handle)
        p = subprocess.Popen(command, env=env, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p)
        return p

    def stop(p):
        if p.poll() is not None:
            return
        os.killpg(p.pid, signal.SIGINT)
        try:
            p.wait(timeout=90)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGTERM)
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait(timeout=10)

    def phase(label, seconds):
        print('\n\a' + label, flush=True)
        manifest['cues'].append(dict(label=label, utc=datetime.now(timezone.utc).isoformat(), monotonic=time.monotonic()))
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('A test process exited; see logs.')
            if shutil.disk_usage(out).free < (512 if args.record_inputs else 128)*1024**2:
                raise RuntimeError('Low disk space; stopping to preserve the recording.')
            if args.ram and psutil.virtual_memory().available < 512*1024**2:
                raise RuntimeError('Low available RAM; stopping capture.')
            print(f'\r  {max(0, int(deadline-time.monotonic()+0.999)):2d} seconds remaining  ', end='', flush=True)
            time.sleep(min(1, max(0, deadline-time.monotonic())))
        print(flush=True)

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        if args.ram:
            print(f'RAM staging: {out}\nPersistent destination: {archive}/capture\n'
                  'Do not reboot or close this terminal until ARCHIVE VERIFIED is printed.', flush=True)
        if args.stationary:
            print('STATIONARY RECORDING CHECK — do not carry the rig during this run.', flush=True)
        print(f'Results: {out}\nUNARMED HAND-CARRY ONLY. Keep PX4 VIO forwarding stopped.\n'
              + ('Schedule: stationary throughout.\n' if args.stationary else
                 'Schedule: 20 s still, 35 s slow loop, 15 s return, 20 s still.\n')
              + ('Raw inputs AND outputs will be recorded for same-input replay.' if args.record_inputs else
                 'This records outputs only; it cannot support same-input offline replay.'), flush=True)
        launch = start(launch_cmd, 'launch')
        phase('WARM-UP — keep the rig stationary.', 35)
        with (out/'effective.yaml').open('w') as stream:
            subprocess.run(['ros2','param','dump','--no-daemon','--spin-time','3','--timeout','5',
                '/rtabmap/rgbd_odometry'], env=env, stdout=stream, check=True, timeout=15)
        import yaml
        params = yaml.safe_load((out/'effective.yaml').read_text())['/rtabmap/rgbd_odometry']['ros__parameters']
        if not params.get('cache_camera_info') or not params.get('keep_latest_frame'):
            raise RuntimeError('Expected cache/latest-frame parameters are not enabled.')
        log = (out/'launch.log').read_text()
        if 'Calibration cache blocked:' in log or 'Odom: quality=' not in log:
            raise RuntimeError('Odometry not ready or cache faulted; inspect launch.log.')
        input('\nPlace the rig at its marked starting pose. Press Enter when ready: ')
        recorder = start(record_cmd, 'record')
        phase('RECORDING STARTUP — keep still.', 5)
        phase('START HOLD — keep still.', 20)
        phase('STATIONARY CHECK — keep still.' if args.stationary else
              'MOVE NOW — carry slowly around a small closed loop.', 35)
        phase('STATIONARY CHECK — keep still.' if args.stationary else
              'RETURN NOW — return to the marked position AND orientation.', 15)
        phase('HOLD STILL NOW — leave it at the starting pose.', 20)
        print('\nDONE MOVING. Flushing recording; do not close this terminal.', flush=True)
        stop(recorder)
        if recorder.returncode != 0 or not (out/'bag/metadata.yaml').exists():
            raise RuntimeError('Recorder did not finish cleanly.')
        record_log = (out/'record.log').read_text()
        if 'Total lost:' in record_log or 'Cache buffers lost messages' in record_log:
            raise RuntimeError('Recorder reported dropped messages; capture is not an acceptance pass.')
        stop(launch)
        if args.record_inputs:
            from ros_rtabmap_bag import inspect_bag
            capture = inspect_bag(out/'bag')
            (out/'input_audit.json').write_text(json.dumps(capture, indent=2)+'\n')
            if not capture['all_inputs_recorded']:
                raise RuntimeError('Missing replay inputs; see input_audit.json.')
        subprocess.run(['python3',str(ROOT/'scripts/ros_rtabmap_bag.py'),'report',str(out/'bag'),
            '--out',str(out/'report.json')], env=env, check=True, stdout=subprocess.DEVNULL)
        manifest['complete'] = True
        print(f'\nCapture complete. Report: {out / "report.json"}', flush=True)
    finally:
        print('\nSTOP MOVING. Shutting down test processes.', flush=True)
        for p in reversed(processes):
            stop(p)
        for handle in handles:
            handle.close()
        (out/'run.json').write_text(json.dumps(manifest, indent=2)+'\n')
        if not manifest['complete']:
            (out/'incomplete.json').write_text('{"complete": false, "exclude_from_acceptance": true}\n')
        if args.ram:
            print('\nGraph stopped. Copying RAM capture to disk and verifying checksums...', flush=True)
            try:
                destination = archive_capture(out, archive/'capture')
            except Exception:
                print(f'ARCHIVE FAILED: RAM data retained at {out}. Do not reboot; preserve it manually.', flush=True)
                raise
            # Only our own mkdtemp directory, after a verified persistent copy.
            if out.parent != Path('/dev/shm') or not out.name.startswith('ros_parity_carry_'):
                raise RuntimeError('Unexpected RAM staging path; refusing cleanup.')
            shutil.rmtree(out)
            print(f'ARCHIVE VERIFIED: {destination}\nTemporary RAM copy removed; disk copy retained.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print('\nCancelled. Partial results preserved.', file=sys.stderr)
        sys.exit(130)
    except Exception as error:
        print(f'\nFAILED: {error}', file=sys.stderr)
        sys.exit(1)
