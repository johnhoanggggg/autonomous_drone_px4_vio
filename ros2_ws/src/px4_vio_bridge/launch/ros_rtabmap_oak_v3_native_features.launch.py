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
