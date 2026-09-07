"""ROS-native RTAB-Map on the depthai_ros_driver_v3 (depthai-core v3) driver.

This is the version of ros_rtabmap_oak_3d.launch.py that can actually fuse the
IMU. The v2 driver delivers /oak/imu/data ~0.4 s behind the images, so
rtabmap_odom drops nearly every frame; measured on the Pi, v3 brings that to
about 10 ms and odometry runs with zero IMU-caused drops.

depthai_ros_driver_v3 is NOT installed system-wide. Either install it
(`sudo apt install ros-jazzy-depthai-ros-driver-v3`, which also upgrades
systemd/udev), or source the local overlay first:

    export AMENT_PREFIX_PATH=/home/john/depthai_v3_overlay/root/opt/ros/jazzy:$AMENT_PREFIX_PATH
    export LD_LIBRARY_PATH=/home/john/depthai_v3_overlay/root/opt/ros/jazzy/lib:\
/home/john/depthai_v3_overlay/root/opt/ros/jazzy/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
    export PYTHONPATH=/home/john/depthai_v3_overlay/root/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH

Nothing here publishes to /fmu/in/*.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def core_string(name):
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "camera_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("px4_vio_bridge"),
                "config",
                "oak_v3_rtabmap.yaml",
            ]),
        ),
        # v3 fuses the IMU correctly, so unlike the v2 launch this defaults on.
        DeclareLaunchArgument("use_imu", default_value="true"),
        # Use keypoints detected/tracked by the OAK's RVC2 while keeping the
        # ROS OdometryROS/F2M core. No DepthAI RTABMapVIO node is created.
        DeclareLaunchArgument("use_oak_features", default_value="false"),
        DeclareLaunchArgument("imu_topic", default_value="/oak/imu/data"),
        # Keep these selectable so the same ROS odometry/SLAM graph can be
        # compared on either the rolling-shutter RGB camera or the
        # global-shutter rectified mono camera.  The latter is the image path
        # used by DepthAI's RTABMapVIO host node.
        DeclareLaunchArgument("image_topic", default_value="/oak/rgb/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/oak/rgb/camera_info"),
        DeclareLaunchArgument("depth_topic", default_value="/oak/stereo/image_raw"),
        DeclareLaunchArgument("odometry_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("mapping_rate_hz", default_value="1.0"),
        DeclareLaunchArgument("voxel_size", default_value="0.05"),
        DeclareLaunchArgument("range_min", default_value="0.30"),
        DeclareLaunchArgument("range_max", default_value="4.0"),
        DeclareLaunchArgument("max_marker_voxels", default_value="50000"),
        # Bundle adjustment is what buys the accuracy: with it off this
        # odometry sits at 3.3-3.8 mm stationary rms regardless of
        # correspondence type, and with it on it reaches 0.47 mm, matching the
        # DepthAI-host RTABMapVIO's 0.52 mm at the same rate. BA cannot run with
        # optical-flow correspondences, hence Vis/CorType 0. Raising features to
        # 1000 and the local map to 3000 gives 0.44 mm and a smoother path but
        # drops the rate to ~10 Hz.
        DeclareLaunchArgument("vis_max_features", default_value="600"),
        DeclareLaunchArgument("vis_cor_type", default_value="0"),
        DeclareLaunchArgument("odom_f2m_max_size", default_value="2000"),
        DeclareLaunchArgument("odom_bundle_adjustment", default_value="1"),
        DeclareLaunchArgument("sync_max_interval", default_value="0.05"),
        DeclareLaunchArgument("oak_feature_sync_max_interval", default_value="0.001"),
        DeclareLaunchArgument("sensor_qos", default_value="2"),
    ]

    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("depthai_ros_driver_v3"),
                "launch",
                "driver.launch.py",
            ])
        ),
        condition=IfCondition(LaunchConfiguration("start_camera")),
        launch_arguments={
            "name": "oak",
            "camera_model": "OAK-D-LITE",
            "params_file": LaunchConfiguration("camera_config"),
            "use_rviz": "false",
        }.items(),
    )

    # Bag replay supplies the sensor topics and calibrated static TF. Keep
    # odometry and mapping composed together without opening the OAK device.
    replay_container = ComposableNodeContainer(
        condition=UnlessCondition(LaunchConfiguration("start_camera")),
        package="rclcpp_components",
        executable="component_container",
        name="oak_container",
        namespace="",
        output="screen",
    )

    # The default camera profile publishes depth in the colour optical frame.
    # A mono-parity profile can instead align it to CAM_B and select
    # /oak/left/image_rect here without changing the RTAB-Map graph.
    sensor_remappings = [
        ("rgb/image", LaunchConfiguration("image_topic")),
        ("depth/image", LaunchConfiguration("depth_topic")),
        ("rgb/camera_info", LaunchConfiguration("camera_info_topic")),
    ]

    odometry_parameters = {
        "use_sim_time": typed("use_sim_time", bool),
        "frame_id": "oak",
        "odom_frame_id": "rtabmap_odom",
        "publish_tf": True,
        # A null odometry message is an identity pose. Publishing it on a
        # brief tracking loss looks like a teleport to the origin to an
        # unsynchronised consumer. Pause pose output instead; OdomInfo and
        # diagnostics still report the loss, so a future PX4 adapter can
        # fail stale rather than forward an invalid pose.
        "publish_null_when_lost": False,
        "wait_imu_to_init": typed("use_imu", bool),
        "always_check_imu_tf": typed("use_imu", bool),
        "qos": typed("sensor_qos", int),
        "expected_update_rate": typed("odometry_rate_hz", float),
        "max_update_rate": typed("odometry_rate_hz", float),
        # RTAB-Map core parameters are declared as strings; passing them as
        # numbers makes the node abort at startup.
        "Odom/Strategy": "0",
        # Preserve the odometry frame through a temporary registration
        # failure. The default is zero too, but make this flight-critical
        # behavior explicit.
        "Odom/ResetCountdown": "0",
        "Vis/MaxFeatures": core_string("vis_max_features"),
        "Vis/CorType": core_string("vis_cor_type"),
        "OdomF2M/MaxSize": core_string("odom_f2m_max_size"),
        "OdomF2M/BundleAdjustment": core_string("odom_bundle_adjustment"),
    }
    odometry_remappings = [
        ("imu", LaunchConfiguration("imu_topic")),
        ("odom", "/rtabmap/vio_odometry"),
        ("odom_info", "/rtabmap/vio_odom_info"),
    ]

    odometry = ComposableNode(
        package="rtabmap_odom",
        plugin="rtabmap_odom::RGBDOdometry",
        namespace="rtabmap",
        name="rgbd_odometry",
        parameters=[odometry_parameters | {
            "approx_sync": True,
            "approx_sync_max_interval": typed("sync_max_interval", float),
            "topic_queue_size": 10,
            "sync_queue_size": 10,
            "qos_camera_info": typed("sensor_qos", int),
            "subscribe_rgbd": False,
        }],
        remappings=sensor_remappings + odometry_remappings,
    )

    mapper = ComposableNode(
        package="rtabmap_slam",
        plugin="rtabmap_slam::CoreWrapper",
        namespace="rtabmap",
        name="rtabmap",
        parameters=[{
            "use_sim_time": typed("use_sim_time", bool),
            "subscribe_depth": True,
            "subscribe_rgb": False,
            "subscribe_rgbd": False,
            "subscribe_stereo": False,
            "subscribe_odom_info": True,
            "frame_id": "oak",
            "map_frame_id": "world",
            "publish_tf": True,
            "approx_sync": True,
            "approx_sync_max_interval": typed("sync_max_interval", float),
            "topic_queue_size": 10,
            "sync_queue_size": 10,
            "qos_image": typed("sensor_qos", int),
            "qos_camera_info": typed("sensor_qos", int),
            "qos_odom": typed("sensor_qos", int),
            "database_path": "",
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "false",
            "Rtabmap/DetectionRate": core_string("mapping_rate_hz"),
            "Rtabmap/SaveWMState": "false",
            "RGBD/CreateOccupancyGrid": "true",
            "Kp/MaxFeatures": "400",
            "Grid/3D": "true",
            "Grid/GroundIsObstacle": "false",
            "Grid/RayTracing": "true",
            "Grid/CellSize": core_string("voxel_size"),
            "Grid/DepthDecimation": "8",
            "Grid/RangeMin": core_string("range_min"),
            "Grid/RangeMax": core_string("range_max"),
            "Grid/PreVoxelFiltering": "true",
            "Grid/NoiseFilteringRadius": "0.15",
            "Grid/NoiseFilteringMinNeighbors": "4",
            "Grid/ClusterRadius": "0.20",
            "Grid/MinClusterSize": "20",
        }],
        remappings=sensor_remappings + [
            ("odom", "/rtabmap/vio_odometry"),
            ("odom_info", "/rtabmap/vio_odom_info"),
        ],
    )

    standard_odometry = LoadComposableNodes(
        target_container="/oak_container",
        condition=UnlessCondition(LaunchConfiguration("use_oak_features")),
        composable_node_descriptions=[odometry],
    )

    tracked_odometry = LoadComposableNodes(
        target_container="/oak_container",
        condition=IfCondition(LaunchConfiguration("use_oak_features")),
        composable_node_descriptions=[ComposableNode(
            package="px4_vio_bridge",
            plugin="px4_vio_bridge::OakTrackedOdometry",
            namespace="rtabmap",
            name="oak_tracked_odometry",
            parameters=[odometry_parameters | {
                "image_topic": LaunchConfiguration("image_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "features_topic": (
                    "/oak/left_rect_feature_tracker/tracked_features"
                ),
                "input_queue_size": 10,
                "input_max_interval": typed("oak_feature_sync_max_interval", float),
            }],
            remappings=odometry_remappings,
        )],
    )

    mapper_component = LoadComposableNodes(
        target_container="/oak_container",
        composable_node_descriptions=[mapper],
    )

    octomap = Node(
        package="px4_vio_bridge",
        executable="rtabmap_octomap_node",
        name="ros_rtabmap_octomap",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": typed("use_sim_time", bool),
            "map_data_topic": "/rtabmap/mapData",
            "frame_id": "world",
            "octomap_topic": "/rtabmap/octomap",
            "metadata_topic": "/rtabmap/octomap_metadata",
            "markers_topic": "/rtabmap/octomap_markers",
            "ground_markers_topic": "/rtabmap/octomap_ground_markers",
            "obstacle_markers_topic": "/rtabmap/octomap_obstacle_markers",
            "max_marker_voxels": typed("max_marker_voxels", int),
        }],
    )

    foxglove = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            ])
        ),
        condition=IfCondition(LaunchConfiguration("foxglove")),
        launch_arguments={
            "port": LaunchConfiguration("foxglove_port"),
            "topic_whitelist": (
                "['^/tf(_static)?$', "
                "'^/oak/(left/(image_rect(/compressed)?|camera_info)|stereo/image_raw|"
                "left_rect_feature_tracker/tracked_features|imu/data)$', "
                "'^/rtabmap/(vio_odometry|vio_odom_info|oak_feature_count|mapPath|localization_pose|info|"
                "octomap_(ground_|obstacle_)?markers|octomap_metadata)$']"
            ),
            "service_whitelist": "['^$']",
            "param_whitelist": "['^$']",
            "client_topic_whitelist": "['^$']",
            "capabilities": "[connectionGraph]",
            "min_qos_depth": "1",
            "max_qos_depth": "1",
        }.items(),
    )

    return LaunchDescription(arguments + [
        LogInfo(msg=(
            "ROS-native RTAB-Map on depthai_ros_driver_v3, IMU fusion enabled. "
            "No node publishes to /fmu/in/*. Foxglove fixed frame is 'world'. "
            "Requires the v3 driver on AMENT_PREFIX_PATH; see the module docstring."
        )),
        camera,
        replay_container,
        standard_odometry,
        tracked_odometry,
        mapper_component,
        octomap,
        foxglove,
    ])
