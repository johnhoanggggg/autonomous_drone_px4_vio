"""ROS RTAB-Map using OAK hardware feature detection/tracking.

The OAK runs StereoDepth and FeatureTracker only. A ROS OdometryROS component
constructs RTAB-Map SensorData with the RVC2 keypoints directly; neither
dai::node::RTABMapVIO nor dai::node::RTABMapSLAM is present. Nothing publishes
to /fmu/in/*.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    forwarded_defaults = {
        "foxglove": "true",
        "foxglove_port": "8765",
        "start_camera": "true",
        "use_sim_time": "false",
        "use_imu": "false",
        "filter_imu": "false",
        "separate_mapper_container": "true",
        # The Pi sustains this deterministic pre-odometry 15 Hz selection;
        # max_update_rate is disabled so it cannot apply a second, irregular gate.
        "odometry_rate_hz": "0.0",
        "mapping_rate_hz": "1.0",
        "voxel_size": "0.05",
        "range_min": "0.30",
        "range_max": "4.0",
        "max_marker_voxels": "0",
        "vis_max_features": "600",
        "vis_min_inliers": "10",
        "oak_input_stride": "2",
        # Reinitialize F2M immediately after a failed registration. This kept
        # the hard lit carry continuous without a large reset discontinuity.
        "odom_reset_countdown": "1",
        "odom_guess_motion": "true",
        "odom_guess_smoothing_delay": "0.0",
        # 2000 retained ~130 stationary inliers but aged to 5-6 Hz and ~141%
        # total CPU. 1000 retained ~110 while sustaining ~8 Hz at ~125% CPU.
        "odom_f2m_max_size": "1000",
        "vis_cor_type": "0",
        "vis_cor_guess_win_size": "40",
        "odom_strategy": "0",
        "odom_bundle_adjustment": "1",
        # Keep RTAB-Map's native BA window by default. Exposing it permits
        # bounded-workload experiments without changing shipped behavior.
        "odom_bundle_adjustment_max_frames": "10",
        "odom_key_frame_thr": "0.3",
        "odom_vis_keyframe_thr": "150",
        "odom_f2m_max_new_features": "0",
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in forwarded_defaults.items()
    ]
    package_share = FindPackageShare("px4_vio_bridge")
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                package_share, "launch", "ros_rtabmap_oak_v3.launch.py",
            ])
        ),
        launch_arguments={
            **{name: LaunchConfiguration(name) for name in forwarded_defaults},
            "camera_config": PathJoinSubstitution([
                package_share, "config", "oak_v3_rtabmap_tracked.yaml",
            ]),
            "image_topic": "/oak/left/image_rect",
            "camera_info_topic": "/oak/left/camera_info",
            "use_oak_features": "true",
        }.items(),
    )
    return LaunchDescription(arguments + [
        LogInfo(msg=(
            "Experimental OAK FeatureTracker -> ROS F2M odometry -> ROS "
            "RTAB-Map/OctoMap. No DepthAI VIO/SLAM and no PX4 output."
        )),
        base_launch,
    ])
