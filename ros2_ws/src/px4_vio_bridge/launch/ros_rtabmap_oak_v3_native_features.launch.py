"""ROS RGB-D odometry with the host feature behavior observed in DepthAI 3.5.

The standard ROS RGBDOdometry node detects GFTT/ORB features on the rectified
left image. OAK tracks remain published for bag comparisons but are not used
by this estimator. This experimental graph does not publish to PX4.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("px4_vio_bridge")
    defaults = {
        "start_camera": "true",
        "start_mapper": "true",
        "start_octomap": "true",
        "mapping_rate_hz": "1.0",
        "range_min": "0.30", "range_max": "4.0",
        "grid_depth_decimation": "8",
        "grid_noise_radius": "0.15", "grid_noise_min_neighbors": "4",
        "grid_cluster_radius": "0.20", "grid_min_cluster_size": "20",
        "test_start_clear_radius": "0.0",
        "test_start_clear_follow_disarmed": "true",
        "octomap_use_checkpoints": "true",
        "camera_position_frd_x": "0.100",
        "camera_position_frd_y": "-0.036",
        "camera_position_frd_z": "0.056",
        "multithreaded_container": "false",
        # Mapping and OctoMap must not share the odometry executor on the Pi.
        "separate_mapper_container": "true",
        "mapper_subscribe_odom_info": "true",
        "use_sim_time": "false",
        "use_imu": "false",  # Raw IMU does not aid either F2M path.
        "foxglove": "true",
        "odometry_rate_hz": "10.0",
        "max_marker_voxels": "0",
        "vis_max_features": "1000",
        "vis_min_inliers": "20",
        "odom_f2m_max_size": "2000",
        "vis_cor_type": "0",
        "odom_bundle_adjustment": "1",
        "sync_max_interval": "0.001",
        "camera_config": PathJoinSubstitution([
            share, "config", "oak_v3_rtabmap_tracked_10hz.yaml",
        ]),
        "odom_config_path": PathJoinSubstitution([share, "config", "rtabmap_native_features.ini"]),
        "keep_latest_frame": "false",
        "keep_next_frame": "false",
        "processing_queue_size": "0",
        # Bound pre-odometry latency. The inputs are hardware-synchronised and
        # constrained to a 1 ms matching window, so two samples are sufficient
        # to pair a tuple without retaining hundreds of milliseconds of work.
        "odometry_topic_queue_size": "2",
        "odometry_sync_queue_size": "2",
        "octomap_pose_update_tolerance_m": "0.01",
        "octomap_pose_update_tolerance_rad": "0.01",
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, "launch", "ros_rtabmap_oak_v3.launch.py",
            ])),
            launch_arguments={
                **{name: LaunchConfiguration(name) for name in defaults},
                "image_topic": "/oak/left/image_rect",
                "camera_info_topic": "/oak/left/camera_info",
                "use_oak_features": "false",
            }.items(),
        ),
    ])
