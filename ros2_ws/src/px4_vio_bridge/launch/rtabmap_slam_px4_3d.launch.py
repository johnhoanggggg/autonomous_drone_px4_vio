"""Run the flown OAK/PX4 stack plus a live, observation-only 3D mapper.

The existing DepthAI RTAB-Map process remains responsible for continuous VIO
and the PX4 feed. A namespaced ROS RTAB-Map instance consumes sampled RGB-D and
raw VIO, publishes keyframe ground/obstacle/empty grids, and the existing C++
adapter rebuilds those grids into a loop-corrected OctoMap for Foxglove.
Nothing added here publishes to /fmu/in/*.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def rtabmap_string(name):
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("start_xrce_agent", default_value="false"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        DeclareLaunchArgument("oak_startup_delay", default_value="5.0"),
        DeclareLaunchArgument("rgbd_publish_hz", default_value="3.0"),
        DeclareLaunchArgument("mapping_rate_hz", default_value="1.0"),
        DeclareLaunchArgument("voxel_size", default_value="0.05"),
        DeclareLaunchArgument("range_min", default_value="0.30"),
        DeclareLaunchArgument("range_max", default_value="4.0"),
        DeclareLaunchArgument("max_marker_voxels", default_value="50000"),
    ]

    base_stack = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("px4_vio_bridge"),
                "launch",
                "rtabmap_slam_px4.launch.py",
            ])
        ),
        launch_arguments={
            "start_xrce_agent": LaunchConfiguration("start_xrce_agent"),
            "foxglove": LaunchConfiguration("foxglove"),
            "foxglove_port": LaunchConfiguration("foxglove_port"),
            "oak_startup_delay": LaunchConfiguration("oak_startup_delay"),
            "slam_publish_ros_rgbd": "true",
            "slam_ros_rgbd_publish_hz": LaunchConfiguration("rgbd_publish_hz"),
        }.items(),
    )

    mapper = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        namespace="rtabmap3d",
        name="rtabmap",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "subscribe_depth": True,
            "subscribe_rgb": False,
            "subscribe_rgbd": False,
            "subscribe_stereo": False,
            "frame_id": "rtabmap3d_camera_link",
            "map_frame_id": "rtabmap3d_map",
            "odom_frame_id": "",
            "publish_tf": True,
            "approx_sync": False,
            "topic_queue_size": 10,
            "sync_queue_size": 10,
            # The OAK bridge publishes sensor data best-effort so stale camera
            # samples cannot back-pressure continuous VIO.
            "qos_image": 2,
            "qos_camera_info": 2,
            "qos_odom": 2,
            "database_path": "/tmp/rtabmap3d_live.db",
            # RTAB-Map core parameters are declared as ROS strings, even for
            # numeric and boolean values.
            "Rtabmap/DetectionRate": rtabmap_string("mapping_rate_hz"),
            "Rtabmap/SaveWMState": "true",
            "RGBD/CreateOccupancyGrid": "true",
            "Grid/3D": "true",
            # Keep RTAB-Map's ground label for brown/red Foxglove coloring. The
            # OctoMap assembler still inserts both ground and obstacles as
            # occupied, so this does not make floor voxels traversable.
            "Grid/GroundIsObstacle": "false",
            "Grid/RayTracing": "true",
            "Grid/CellSize": rtabmap_string("voxel_size"),
            "Grid/DepthDecimation": "8",
            "Grid/RangeMin": rtabmap_string("range_min"),
            "Grid/RangeMax": rtabmap_string("range_max"),
            "Grid/PreVoxelFiltering": "true",
            "Grid/NoiseFilteringRadius": "0.15",
            "Grid/NoiseFilteringMinNeighbors": "4",
            "Grid/ClusterRadius": "0.20",
            "Grid/MinClusterSize": "20",
        }],
        remappings=[
            ("rgb/image", "/rtabmap3d/input/image"),
            ("depth/image", "/rtabmap3d/input/depth"),
            ("rgb/camera_info", "/rtabmap3d/input/camera_info"),
            ("odom", "/rtabmap3d/input/odom"),
        ],
        # Start with a fresh map. This launch is for live observation; it never
        # overwrites the DepthAI SLAM database or changes the PX4 VIO source.
        arguments=["-d"],
    )

    octomap = Node(
        package="px4_vio_bridge",
        executable="rtabmap_octomap_node",
        name="rtabmap_live_octomap",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "map_data_topic": "/rtabmap3d/mapData",
            "frame_id": "rtabmap3d_map",
            "octomap_topic": "/rtabmap3d/octomap",
            "metadata_topic": "/rtabmap3d/octomap_metadata",
            "markers_topic": "/rtabmap3d/octomap_markers",
            "max_marker_voxels": typed("max_marker_voxels", int),
        }],
    )

    return LaunchDescription(arguments + [
        LogInfo(msg=(
            "Live 3D mapping is observation-only. In Foxglove select fixed frame "
            "'rtabmap3d_map' and add /rtabmap3d/octomap_markers."
        )),
        base_stack,
        mapper,
        octomap,
    ])
