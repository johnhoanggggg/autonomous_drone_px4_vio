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
        "odometry_rate_hz": "20.0",
        "mapping_rate_hz": "1.0",
        "voxel_size": "0.05",
        "range_min": "0.30",
        "range_max": "4.0",
        "max_marker_voxels": "50000",
        "vis_max_features": "600",
        "odom_f2m_max_size": "2000",
        "vis_cor_type": "0",
        "odom_bundle_adjustment": "1",
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
