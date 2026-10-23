# Autonomous Drone PX4 VIO

An indoor quadrotor that flies itself with no GPS: a Raspberry Pi 5 runs visual-inertial
odometry (VIO) from an OAK-D Lite stereo camera, builds a map, plans a route around
obstacles and sends position setpoints to a Pixhawk 4 running PX4. Built for a 2026
final-year project (John Hoang, Diego Augustincic, Eric Zhu). This README is the
handoff to the next team: what the drone is, how to rebuild it, how to fly it safely,
and what is still broken. Everything else is in [docs/](docs/INDEX.md).

> **Safety first.** Every flight mode here is experimental research code. Fly only with
> someone holding the RC transmitter with the kill switch (channel 9) under their thumb,
> in a space where a falling 1.8 kg drone hurts nobody. The kill switch cuts the motors:
> the drone drops. Every flight launch defaults to `auto_arm:=false`, so you can and
> should rehearse each one props-off first.

## Contents

1. [What it does (October 2026)](#what-it-does-october-2026)
2. [Hardware](#hardware)
3. [Software stack](#software-stack)
4. [Repository layout](#repository-layout)
5. [Rebuilding from scratch](#rebuilding-from-scratch)
6. [PX4 parameters](#px4-parameters)
7. [Flying](#flying)
8. [After a flight: data and analysis](#after-a-flight-data-and-analysis)
9. [Known problems and open work](#known-problems-and-open-work)
10. [Where everything else is](#where-everything-else-is)

## What it does (October 2026)

- **2D navigation:** A* over an RTAB-Map occupancy grid at constant height. Flown
  through rooms, a doorway about 0.8 m wide and corridors (last flown 10-04).
- **3D navigation:** A* through an OctoMap that treats unknown space as passable
  with care. Flown up a stairwell, through a 0.53 to 0.60 m gap between boxes, over
  an obstacle and along routes of about 6 m (last flown 10-09).
- **Localisation:** VIO fused into PX4's EKF2 as external vision. Against Vicon
  ground truth (10-05): 2 to 3 cm horizontal RMSE, but height drifts about 15 cm per
  minute.
- **Bump tolerance:** a carbon-fibre cage. On the 4S tune a hand push moves the drone
  about 0.27 s times the push's velocity change (for example 27 cm for 1 m/s) and it
  is back within 5 cm in about 1.4 s.
- **Not done on the real drone:** autonomous exploration (frontier exploration only ran
  in simulation), GPS, outdoor flight, landing on anything but flat floor.

## Hardware

| Part | What |
|---|---|
| Frame | F330 quad with a carbon-fibre rod cage, all-up mass 1.8 kg |
| Flight controller | Holybro Pixhawk 4 (FMU v5), PX4 v1.17.0 |
| Power | Holybro PM07 power module, Turnigy Graphene Panther 4S 5000 mAh 100C (XT90) |
| Propulsion | 1700 KV motors, BLHeli 30 A ESCs, 6-inch props |
| Companion computer | Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy, on its own BEC |
| Camera | Luxonis OAK-D Lite (stereo + IMU) |
| Link | Pi `/dev/ttyAMA0` to Pixhawk TELEM2 at 921600 baud (uXRCE-DDS); Pi USB to Pixhawk USB for MAVLink shell and log download |
| RC | GHST receiver; channel 9 = kill switch, channel 7 = flight mode (not used) |
| Not fitted | GPS, compass use, rangefinder, optical flow |

The airframe flew on 3S until mid-September; it is now 4S with a retune. Old 3S logs
and notes still appear in `docs/`.

## Software stack

The 3D flight mode, which is the most complete:

```
 OAK-D Lite (stereo, IMU, feature tracker on the camera)
    |  depthai_ros_driver_v3 (patched source build)
    v
 RTAB-Map odometry (rtabmap_ros 0.22.1, patched) --> /rtabmap/vio_odometry (~10-17 Hz)
    |                                   |
    |                                   v
    |                         vio_to_px4_odometry (ENU/FLU -> NED/FRD)
    |                                   |
    v                                   v  /fmu/in/vehicle_visual_odometry
 rtabmap_slam (1 Hz) -> OctoMap      Micro XRCE-DDS Agent (systemd, serial)
    |                                   |
    v                                   v
 3D A* planner (ros_planner_3d_monitor)    PX4 EKF2 fuses it as external vision
    |  /planner3d/reference_path            |
    v                                       |
 cpp_flight_adapter_3d  ---- trajectory setpoints, offboard mode, arm/land ----> PX4
 (offboard_planner_3d_unknown)
```

The 2D mode is the same idea with older pieces: RTAB-Map VIO and SLAM run through
DepthAI on the Pi (`rtabmap_slam_px4.launch.py`), the planner is a 2D A* on the
occupancy grid (`global_planner_monitor`) and the adapter is
`offboard_global_planner`. The C++ versions of the planner, follower and adapter are
selected with `cpp_nodes:=true` and are what flies.

Everything is in one ROS 2 package, `ros2_ws/src/px4_vio_bridge` (Python nodes in
`px4_vio_bridge/`, C++ in `src/` and `include/`, launch files in `launch/`, about 675
unit and integration tests in `test/`). New planner and flight work went into the C++
nodes; the Python versions are kept for comparison.

## Repository layout

| Path | What |
|---|---|
| `ros2_ws/src/px4_vio_bridge/` | the ROS 2 package: nodes, launch files, config, tests |
| `scripts/` | tools: PX4 shell (`nsh.py`), log download (`fetch_ulog.py`), parameter backup (`dump_params.py`), plots |
| `reports/` | one folder per investigation: the analysis scripts are tracked, their data and figures are not |
| `params_backup_*.params` | full PX4 parameter dumps, newest = current config |
| `deps.repos` | pinned versions of every workspace built outside this repo |
| `patches/` | local changes to the DepthAI driver and `rtabmap_ros` |
| `third_party/` | a hand-rebuilt message package and an environment file that exist nowhere else |
| `systemd/` | the Micro XRCE-DDS Agent service |
| `docs/` | earlier README, subsystem handoffs and dated investigation notes ([index](docs/INDEX.md)) |
| `ros2_ws/flight_logs/` | flight bags and ULogs, **not in git** (see [data](#after-a-flight-data-and-analysis)) |

**Paths:** many scripts and launch files hard-code `/home/john/...`. Put the repo at
`/home/john/autonomous_drone_px4_vio` (the Pi's user is `john`) or symlink it there,
and the same for the workspaces in `deps.repos`. Removing these paths is open work.

## Rebuilding from scratch

The quickest way to a working drone is a copy of the current Pi's SD card (power the
Pi off, then image the whole card with `dd` or Raspberry Pi Imager). To rebuild on a
fresh Raspberry Pi 5 instead:

1. **Ubuntu 24.04 + ROS 2 Jazzy** (`ros-jazzy-ros-base`, `ros-jazzy-rtabmap-ros`,
   `ros-jazzy-octomap-server`, `ros-jazzy-foxglove-bridge`, `python3-psutil`).
   Set `export ROS_DOMAIN_ID=42` in `~/.bashrc`.
2. **Micro XRCE-DDS Agent v3.0.1** built from eProsima's GitHub into
   `/usr/local/bin/MicroXRCEAgent`, then
   `sudo cp systemd/micro-xrce-agent.service /etc/systemd/system/ && sudo systemctl enable --now micro-xrce-agent`.
   The v2.4.3 build under `tools/` never completes a handshake with this PX4; do not use it.
3. **External workspaces** at the versions in `deps.repos` (`pip install vcstool`):
   ```bash
   cd ~ && vcs import < autonomous_drone_px4_vio/deps.repos
   ```
   Then, in this order, each with `source /opt/ros/jazzy/setup.bash` first:
   - `~/ros2_ws`: `colcon build --packages-select px4_msgs` (open_vins is optional).
   - `~/depthai_v3_overlay`: download the six ros-jazzy debs listed in `deps.repos`
     and `dpkg -x` each into `~/depthai_v3_overlay/root` (do not `apt install` them,
     that also upgrades systemd and udev).
   - `~/depthai_v3_compat_ws`: copy `third_party/depthai_ros_msgs_v3` into `src/`, build.
   - `~/depthai_v3_feature_ws`: `git apply` the depthai patch from `patches/` in
     `src/depthai-ros`, build `depthai_ros_driver`.
   - `~/rtabmap_latest_frame_overlay`: `git apply patches/rtabmap_ros_0.22.1_jazzy_latest_frame.patch`
     in `src/rtabmap_ros`, build `rtabmap_odom`.
   - `~/rtabmap_parity_0_21_12/base_ros_env.bash`: copy from `third_party/env/`.
4. **DepthAI Python** pinned to 3.5.0 (3.7.1 crashes this camera's CAM_B at startup):
   `python3 -m pip install --user --break-system-packages -r requirements-depthai.txt`.
5. **This package:**
   ```bash
   cd /home/john/autonomous_drone_px4_vio/ros2_ws
   source /opt/ros/jazzy/setup.bash
   source /home/john/ros2_ws/install/setup.bash
   colcon build --packages-select px4_vio_bridge
   python3 -m pytest src/px4_vio_bridge/test/ -q      # no hardware needed
   ```
   Always build from inside `ros2_ws/`. A `colcon build` at the repo root creates a
   second install space that silently shadows the real one.
6. **Tools venv** for the PX4 scripts and plots:
   `python3 -m venv .venv-mavlink && .venv-mavlink/bin/pip install pymavlink pyulog matplotlib numpy scipy`.

Build on the USB-C bench supply, not the drone battery: a full build pulled the Pi's
5 V rail down and reset it twice. Use `-j2` if you must build on the drone.

Every terminal then needs:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
```

## PX4 parameters

The current configuration is `params_backup_20261023_022612.params` (all 1191 params,
loadable in QGroundControl). Make a new backup before changing anything:
`.venv-mavlink/bin/python3 scripts/dump_params.py`. Backups older than 10-23 were
converted on 10-23: the old dump script wrote integer params as garbage floats.

The settings that matter most:

| Param | Value | Why |
|---|---|---|
| `EKF2_EV_CTRL` / `EKF2_HGT_REF` | 11 / 3 | fuse vision position and yaw; height from vision |
| `EKF2_EV_DELAY` | 225 ms | VIO latency |
| `EKF2_EV_POS_X/Y/Z` | 0.100 / -0.036 / 0.056 m | camera position on the body |
| `MC_ROLL_P` / `MC_PITCH_P` | 6.5 / 6.5 | 4S retune (was 4.0 on 3S) |
| `MC_ROLLRATE_I` / `MC_PITCHRATE_I` | 0.2 / 0.2 | |
| `MPC_XY_P` | 0.95 | |
| `MPC_XY_VEL_P_ACC` / `_I_ACC` / `_D_ACC` | 2.6 / 0.9 / 0.2 | stops and push recovery |
| `MPC_THR_HOVER` | 0.38 | 4S hover throttle |
| `MPC_Z_P` | 1.5 | already at its maximum |
| `SENS_BOARD_X_OFF` / `_Y_OFF` | -4.66 / 0.55 deg | level trim; the FC mount keeps moving, see below |
| `TC_A_ENABLE` | 1 | accelerometer thermal calibration |
| `RC_MAP_KILL_SW` | 9 | kill switch |
| `COM_LOW_BAT_ACT` | 0 | **PX4 never lands on low battery.** Only `offboard_hover` lands on its own (3.50 V per cell); the planner adapters do not. See [known problems](#known-problems-and-open-work) |
| `BAT1_N_CELLS` / `BAT1_R_INTERNAL` | 4 / 0.0082 | stops false "emergency battery" under load |
| `BAT1_CAPACITY` | 5400 | **wrong**: the pack is 5000 mAh, so PX4's remaining % is optimistic |
| `UXRCE_DDS_SYNCC` / `_SYNCT` | 0 / 0 | with time sync on, the DDS link never connects |

Integer params set over MAVLink as floats get stored as bit patterns (for example
`EKF2_EV_CTRL=1077936128`). Set integers from the PX4 shell instead
(`scripts/nsh.py "param set EKF2_EV_CTRL 11" "param save"`) and check with `param show`.

## Flying

### Before every session

1. Battery charged, cage and props checked, RC on, kill switch tested **props off**:
   arm from the PX4 shell, flip channel 9, `listener actuator_armed` shows `kill: True`.
2. Stop the Pi's clock from jumping mid-session (a clock step makes the planner reject
   every map): `sudo timedatectl set-ntp false`.
3. `systemctl is-active micro-xrce-agent` says `active`.
4. Do not rotate the drone between restarting VIO and arming: a new VIO session starts
   its heading fresh and PX4 can sit in `ARM_INHIBITED` for a minute while they agree.
5. Fly warm: a cold flight controller pushes the drone sideways. Hover or idle for a few
   minutes first.
6. Pre-flight gate (all green): `scripts/nsh.py "uxrce_dds_client status"` says
   connected; `listener vehicle_local_position 1` has `xy_valid`, `z_valid` and
   `heading_good_for_control` true.

### Ways to stop a flight

| Action | Effect |
|---|---|
| RC channel 9 | motors cut immediately, the drone falls. The only control that does not depend on the Pi |
| `L` in the adapter's terminal | AUTO.LAND |
| `K` in the adapter's terminal | immediate disarm (a kill, the drone falls) |
| Ctrl-C on the adapter | AUTO.LAND, then exit |
| Automatic | the adapters land on lost or reset VIO, stale map or pose, repeated route faults, geofence and the flight-time limit (`max_flight_time`); `offboard_hover` also on yaw-rate spikes and low battery. The exact checks differ per adapter |

### 1. Hover or square (first flight of the day)

Start the 2D stack's terminal 1 (below), then:

```bash
ros2 run px4_vio_bridge offboard_hover --ros-args -p auto_arm:=false -p hover_height:=0.30 -p hold_time:=10.0
ros2 launch px4_vio_bridge offboard_square.launch.py auto_arm:=false climb_timeout:=5.0
```

Run each props-off with `auto_arm:=false` first, then again with `auto_arm:=true`.

### 2. 2D planner flight

Three terminals. Terminal 1 (camera, VIO, SLAM, PX4 bridge, Foxglove):

```bash
ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py slam_publish_grid:=true \
  slam_grid_3d:=false slam_grid_ray_tracing:=true slam_grid_cell_size:=0.03 \
  slam_grid_footprint_radius:=0.25
```

Terminal 2 (planner and follower; cannot move the drone): the
`global_planner_monitor.launch.py cpp_nodes:=true ...` command, and terminal 3 (the
adapter, which arms): `offboard_global_planner.launch.py cpp_nodes:=true ...`. Both
take a dozen tuned arguments that must agree with each other. Copy them from
[the full README's quick start](docs/README_full_20260921.md#quick-start--the-three-terminals),
which also explains each one. Click a goal in Foxglove's 3D panel (Publish tool on
`/waypoint/clicked`, fixed frame `world`) and check `/planner/follower/valid` is true
before starting terminal 3.

### 3. 3D planner flight (the 10-09 settings)

Terminal 1 starts the ROS camera, VIO, mapping and the 3D planner (no PX4 output):

```bash
ros2 launch px4_vio_bridge ros_planner_3d_monitor.launch.py start_backend:=true foxglove:=true \
  planning_radius_xy:=8.0 min_z:=-0.5 max_z:=1.6 safety_margin:=0.07
```

Click a goal in Foxglove. Terminal 2 is the flight adapter. Props off with
`auto_arm:=false` first: it still sends real offboard commands to PX4.

```bash
ros2 launch px4_vio_bridge offboard_planner_3d_unknown.launch.py auto_arm:=true \
  geofence_radius:=8.0 max_height:=2.6 required_clearance:=0.22 \
  route_speed:=0.3 command_vertical_speed:=0.3 brake_damping:=true \
  path_carrot:=true hold_line_skip:=0.35 cross_track_gain:=0.3 cross_track_correction_max:=0.05
```

These are the values the 10-09 flights recorded on `/planner3d/flight/config` and
`/planner3d/status`. Keep the goal inside `geofence_radius - required_clearance` and the
planner's `planning_radius_xy`, or the adapter takes off and hovers with no route.
`map_timeout` was raised from 3.0 to 4.0 s after 10-09 and has not flown yet.
Design notes: [HANDOFF_3D_FLIGHT_ADAPTER.md](docs/HANDOFF_3D_FLIGHT_ADAPTER.md).

### After landing

Each flight launch records its own bag to `ros2_ws/flight_logs/<mode>_<UTC time>/`.
**Wait for `BAG SAVED to disk: ... safe to unplug` before unplugging the Pi**: an
unclean power-off loses the bag and has corrupted this git repo before.

## After a flight: data and analysis

- **Bags:** `ros2_ws/flight_logs/` (`.mcap` files). They are ignored by git: about 7.5 GB
  from July to October. A full copy (repo, bags, renders, Vicon CSVs) was uploaded on
  10-23 to Google Drive as `drone_backup/drone_data_20261023.tar`; ask John for access.
- **PX4 logs (ULog):** on the Pixhawk's SD card. Download one with
  `.venv-mavlink/bin/python3 scripts/fetch_ulog.py /fs/microsd/log/<date>/<time>.ulg <dest>`
  (slow: about 140 KB/s; for many logs take the SD card out). The FC's date and time run
  about 9 hours behind real time, so match logs to flights by content, not by name.
  Impacts and pushes only show up in the ULog (`vehicle_acceleration`).
- **Plots and renders:** `scripts/render_flight_map.py` (top view of a 2D flight),
  `reports/planner3d_track_20260929/anim/animate3d.py` and `render_global.py` (3D and
  2D flight animations), `reports/planner3d_track_20260929/gap_swing/plot_control_full.py`
  (whole-flight control stack from a ULog), `scripts/plot_bump_test.py` and
  `scripts/render_bump_video.py` (push tests, with video overlay).
- **PX4 shell:** `.venv-mavlink/bin/python3 scripts/nsh.py "<command>" ...` over USB.
  Only one program can use `/dev/ttyACM0` at a time.

## Known problems and open work

Roughly in order of how much they will bite you.

- **Height drift.** Vision height drifts about 15 cm per minute against Vicon. There is
  no rangefinder, so PX4 also ignores `MPC_LAND_SPEED` and AUTO.LAND from 1 m hits the
  floor at about 0.9 m/s (6 g). A downward rangefinder fixes both.
- **The flight controller's mount moves.** Level trim has been redone five times; the
  whole board tilts, so thermal calibration cannot fix it. Re-trim with
  `scripts/retrim_from_flights.py` when the drone drifts sideways in hover, or better,
  fix the mount.
- **VIO does not recover after tracking loss.** Covering the camera or flying at a
  blank wall loses tracking; the pose then jumps to exactly (0, 0, 0) and the drone
  lands. Calling RTAB-Map's `reset_odom` service (`/rtabmap/reset_odom`) recovers it on
  the ground. An automatic reset exists for the disarmed case only.
- **Maps close up at featureless walls.** Stereo depth on a plain far wall is noisy;
  in one corridor flight a 0.8 m doorway filled in on the map and the planner gave up.
  The OAK-D Lite also sees nothing closer than about 0.36 m.
- **Narrow gaps are marginal.** Routes keep 2 to 3 cm of spare clearance while the
  drone follows its path to about 10 to 16 cm (95th percentile), so it holds and
  re-plans in tight spots.
- **CPU.** The Pi runs at about 72% average and 97% peak in a 3D flight. One RTAB-Map
  thread pins a full core for reasons not yet understood.
- **No low-battery landing in planner flights.** PX4's own low-battery action is off
  (`COM_LOW_BAT_ACT=0`) and the 2D and 3D adapters leave battery to PX4, so nothing lands
  the drone on a flat pack. Only `offboard_hover` lands (3.50 V per cell). Until that is
  fixed (set `COM_LOW_BAT_ACT` to land, after checking it does not trigger on sag),
  keep `max_flight_time` short and watch the voltage. A hover lasts about 7 minutes on a
  full 4S pack.
- **Hard-coded paths** to `/home/john` in about 47 files.
- **Built but never flown:** `map_timeout` 4.0 s; SLAM map save and reload (DepthAI
  backend); a Basalt VIO backend (bench results in the full README); VFH local
  avoidance (`offboard_vfh`, parked).
- **Ideas for next year:** a rangefinder; onboard frontier exploration (it works in
  simulation); a better depth camera for close obstacles; moving the remaining Python
  nodes to C++.

## Where everything else is

- [docs/INDEX.md](docs/INDEX.md): every earlier handoff and investigation note, one line
  each. [docs/README_full_20260921.md](docs/README_full_20260921.md) is the previous
  README: launch reference, parameter rationale, measurements. It is current to 09-21.
- The 2026 FYP paper (method, results, figures) is not in this repo: ask John for the
  PDF, and add it to `docs/` when you have it.
- Branches: `main` is the current code. `voxel-perception-tuning` and
  `fix/velocity-aware-stop-recovery` hold experiments that never merged. Tags
  `flown-20261004` and `flown-20261009` mark the code that flew on those days.
