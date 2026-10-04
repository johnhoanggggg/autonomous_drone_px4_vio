"""Run the flown OAK/PX4 stack plus a live, observation-only 3D mapper.

The DepthAI process remains responsible for continuous VIO and the PX4 feed. A
namespaced ROS RTAB-Map instance consumes sampled RGB-D and raw VIO, publishes
keyframe ground/obstacle/empty grids, and the existing C++ adapter rebuilds
those grids into a loop-corrected OctoMap for Foxglove. Nothing added here
publishes to /fmu/in/*.

By default the on-device RTABMapSLAM node is switched off here, so only one
RTAB-Map builds a map instead of two. Measured on the Pi, that takes the
DepthAI process from 187.5% to 64.9% CPU, about 1.2 cores, while /basalt/pose
holds ~24 Hz. PX4 is unaffected: it is fed from the raw /rtabmap/vio_pose,
which comes from the VIO node and never passed through SLAM. Set
slam_enable_slam:=true to restore the previous two-map behaviour.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
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
        DeclareLaunchArgument("slam_enable_slam", default_value="false"),
        DeclareLaunchArgument("mapping_rate_hz", default_value="1.0"),
        DeclareLaunchArgument("voxel_size", default_value="0.05"),
        DeclareLaunchArgument("range_min", default_value="0.30"),
        DeclareLaunchArgument("range_max", default_value="4.0"),
        DeclareLaunchArgument("max_marker_voxels", default_value="50000"),
        # Parity with the on-device RTABMapSLAM as the flight stack configures
        # it (rtabmap_slam_px4.launch.py slam_grid_footprint_radius). Clears
        # obstacles under the graph; RTAB-Map's own default is 0.0, so leaving
        # it unset made the ROS map keep clutter the on-device map removed.
        DeclareLaunchArgument("grid_footprint_radius", default_value="0.25"),
        # With slam_enable_slam:=true BOTH mappers run on the same camera feed,
        # which is the controlled way to compare their loop closures: one walk,
        # identical data. Set this so the DepthAI mapper writes a database too;
        # the ROS one already writes /tmp/rtabmap3d_live.db.
        DeclareLaunchArgument("slam_database_path", default_value=""),
        # When the on-device SLAM is off, publish the mapper's products under
        # the /rtabmap names the VFH and planner nodes already subscribe to.
        # "auto" means legacy names exactly when slam_enable_slam is false, so
        # the two mappers can never publish the same topic.
        DeclareLaunchArgument("legacy_topics", default_value="auto"),
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
            # Preserve the established flight path, even if an outer scope
            # supplies backend:=ros. ROS mode is selected on the base launch.
            "backend": "depthai",
            # The 3D adapter judges heights in the map frame; keep its EV
            # unleveled until leveling has been flown there.
            "level_ev": "false",
            "start_xrce_agent": LaunchConfiguration("start_xrce_agent"),
            "foxglove": LaunchConfiguration("foxglove"),
            "foxglove_port": LaunchConfiguration("foxglove_port"),
            "oak_startup_delay": LaunchConfiguration("oak_startup_delay"),
            "slam_publish_ros_rgbd": "true",
            "slam_enable_slam": LaunchConfiguration("slam_enable_slam"),
            "slam_database_path": LaunchConfiguration("slam_database_path"),
            "slam_ros_rgbd_publish_hz": LaunchConfiguration("rgbd_publish_hz"),
        }.items(),
    )

    def _mapper(context, *_a, **_k):
        enable_slam = LaunchConfiguration("slam_enable_slam").perform(
            context).strip().lower() in ("1", "true", "yes", "on")
        legacy = LaunchConfiguration("legacy_topics").perform(context).strip().lower()
        if legacy == "auto":
            legacy = not enable_slam
        else:
            legacy = legacy in ("1", "true", "yes", "on")

        outputs = []
        if legacy:
            # The on-device node published these; with it gone the ROS mapper
            # has to, or vfh_obstacles/route_follower/global_planner go silent.
            outputs = [
                ("localization_pose", "/rtabmap/pose"),
                ("cloud_obstacles", "/rtabmap/obstacle_cloud"),
                ("cloud_ground", "/rtabmap/ground_cloud"),
                ("map", "/rtabmap/grid"),
                ("mapPath", "/rtabmap/path"),
            ]

        nodes = [Node(
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
                # The OAK bridge publishes sensor data best-effort so stale
                # camera samples cannot back-pressure continuous VIO.
                "qos_image": 2,
                "qos_camera_info": 2,
                "qos_odom": 2,
                "database_path": "/tmp/rtabmap3d_live.db",
                # RTAB-Map core parameters are declared as ROS strings, even
                # for numeric and boolean values. The set below is kept at
                # parity with the on-device RTABMapSLAM params in
                # basalt_rtabmap_slam_ros2; see the parity table in README.
                "Rtabmap/DetectionRate": rtabmap_string("mapping_rate_hz"),
                "Rtabmap/SaveWMState": "true",
                "RGBD/CreateOccupancyGrid": "true",
                # Deliberately NOT at parity: the on-device node ran Grid/3D
                # false to feed the 2D planner. 3D is required here for the
                # OctoMap, and RTAB-Map still projects a 2D map from it.
                "Grid/3D": "true",
                # Keep RTAB-Map's ground label for brown/red Foxglove coloring.
                # The OctoMap assembler still inserts both ground and obstacles
                # as occupied, so this does not make floor voxels traversable.
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
                "GridGlobal/FootprintRadius": rtabmap_string(
                    "grid_footprint_radius"),
            }],
            remappings=[
                ("rgb/image", "/rtabmap3d/input/image"),
                ("depth/image", "/rtabmap3d/input/depth"),
                ("rgb/camera_info", "/rtabmap3d/input/camera_info"),
                ("odom", "/rtabmap3d/input/odom"),
            ] + outputs,
            # Start with a fresh map. This launch is for live observation; it
            # never overwrites the DepthAI SLAM database or changes the PX4 VIO
            # source.
            arguments=["-d"],
        )]

        if legacy:
            # rtabmap_slam has no continuous corrected-pose topic: it publishes
            # localization_pose only on a localization event, which never fires
            # in SLAM mode, and exposes the correction as the map->odom TF.
            # Rebuild both topics the on-device node used to publish.
            nodes.append(Node(
                package="px4_vio_bridge",
                executable="cpp_slam_pose_relay",
                name="slam_pose_relay",
                output="screen",
                emulate_tty=True,
                parameters=[{
                    "vio_pose_topic": "/rtabmap/vio_pose",
                    "pose_topic": "/rtabmap/pose",
                    "correction_topic": "/rtabmap/odom_correction",
                    "map_frame": "rtabmap3d_map",
                    "odom_frame": "rtabmap3d_odom",
                    "output_frame": "world",
                }],
            ))

        return nodes

    mapper = OpaqueFunction(function=_mapper)

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
            "ground_markers_topic": "/rtabmap3d/octomap_ground_markers",
            "obstacle_markers_topic": "/rtabmap3d/octomap_obstacle_markers",
            "max_marker_voxels": typed("max_marker_voxels", int),
        }],
    )

    return LaunchDescription(arguments + [
        LogInfo(msg=(
            "Live 3D mapping is observation-only. In Foxglove select fixed frame "
            "'rtabmap3d_map' and add /rtabmap3d/octomap_ground_markers and "
            "/rtabmap3d/octomap_obstacle_markers."
        )),
        base_stack,
        mapper,
        octomap,
    ])
