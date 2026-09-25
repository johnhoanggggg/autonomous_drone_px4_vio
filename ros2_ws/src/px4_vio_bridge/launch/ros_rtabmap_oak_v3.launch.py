"""ROS-native RTAB-Map on the depthai_ros_driver_v3 (depthai-core v3) driver.

The v2 driver delivers /oak/imu/data ~0.4 s behind the images, so
OdometryROS drops nearly every frame while waiting for IMU. The v3 driver fixes
that timing. F2M needs an orientation quaternion to use the IMU: bare raw
acceleration/gyro messages are ignored by this core. Set filter_imu:=true to
experiment with Madgwick orientation aiding (not tightly coupled VIO).

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
from launch.substitutions import (
    IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution,
    PythonExpression,
)
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
        # v3 provides current IMU stamps, so waiting for IMU can default on.
        DeclareLaunchArgument("use_imu", default_value="true"),
        DeclareLaunchArgument("start_mapper", default_value="true"),
        DeclareLaunchArgument("start_octomap", default_value="true"),
        DeclareLaunchArgument("test_start_clear_radius", default_value="0.0"),
        DeclareLaunchArgument("octomap_use_checkpoints", default_value="true"),
        DeclareLaunchArgument("camera_position_frd_x", default_value="0.100"),
        DeclareLaunchArgument("camera_position_frd_y", default_value="-0.036"),
        DeclareLaunchArgument("camera_position_frd_z", default_value="0.056"),
        DeclareLaunchArgument("multithreaded_container", default_value="false"),
        DeclareLaunchArgument("separate_mapper_container", default_value="false"),
        DeclareLaunchArgument("mapper_subscribe_odom_info", default_value="true"),
        DeclareLaunchArgument("base_frame_id", default_value="oak"),
        # F2M consumes orientation, not bare gyro/accelerometer measurements.
        # Keep this selectable for controlled comparisons with the historical
        # raw-IMU path. It is orientation aiding, not tightly coupled VIO.
        DeclareLaunchArgument("filter_imu", default_value="false"),
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
        DeclareLaunchArgument("grid_depth_decimation", default_value="8"),
        DeclareLaunchArgument("grid_noise_radius", default_value="0.15"),
        DeclareLaunchArgument("grid_noise_min_neighbors", default_value="4"),
        DeclareLaunchArgument("grid_cluster_radius", default_value="0.20"),
        DeclareLaunchArgument("grid_min_cluster_size", default_value="20"),
        # Full-tree marker traversal causes estimator starvation as maps grow.
        # The binary OctoMap remains published; opt in to markers for short
        # visualization sessions only.
        DeclareLaunchArgument("max_marker_voxels", default_value="0"),
        DeclareLaunchArgument("octomap_pose_update_tolerance_m", default_value="0.01"),
        DeclareLaunchArgument("octomap_pose_update_tolerance_rad", default_value="0.01"),
        # Bundle adjustment is what buys the accuracy: with it off this
        # odometry sits at 3.3-3.8 mm stationary rms regardless of
        # correspondence type, and with it on it reaches 0.47 mm, matching the
        # DepthAI-host RTABMapVIO's 0.52 mm at the same rate. BA cannot run with
        # optical-flow correspondences, hence Vis/CorType 0. Raising features to
        # 1000 and the local map to 3000 gives 0.44 mm and a smoother path but
        # drops the rate to ~10 Hz.
        DeclareLaunchArgument("vis_max_features", default_value="600"),
        DeclareLaunchArgument("vis_min_inliers", default_value="20"),
        DeclareLaunchArgument("vis_cor_type", default_value="0"),
        DeclareLaunchArgument("odom_strategy", default_value="0"),
        DeclareLaunchArgument("odom_f2m_max_size", default_value="2000"),
        DeclareLaunchArgument("odom_reset_countdown", default_value="0"),
        DeclareLaunchArgument("odom_guess_motion", default_value="true"),
        DeclareLaunchArgument("odom_guess_smoothing_delay", default_value="0.0"),
        DeclareLaunchArgument("odom_bundle_adjustment", default_value="1"),
        DeclareLaunchArgument("odom_bundle_adjustment_max_frames", default_value="10"),
        DeclareLaunchArgument("odom_key_frame_thr", default_value="0.3"),
        DeclareLaunchArgument("odom_vis_keyframe_thr", default_value="150"),
        DeclareLaunchArgument("odom_f2m_max_new_features", default_value="0"),
        DeclareLaunchArgument("odom_bundle_min_motion", default_value="0.0"),
        DeclareLaunchArgument("vis_cor_guess_win_size", default_value="40"),
        DeclareLaunchArgument("odom_config_path", default_value=""),
        # Available only in the isolated patched rtabmap_odom overlay. Keep
        # false for the stock package; diagnostic launches can retain one
        # latest complete tuple while the odometry worker is busy.
        DeclareLaunchArgument("keep_latest_frame", default_value="false"),
        DeclareLaunchArgument("keep_next_frame", default_value="false"),
        DeclareLaunchArgument("processing_queue_size", default_value="0"),
        # Keep synchronizer backlog independently tunable. A ten-frame queue
        # can add 0.33 s at 30 Hz before odometry even starts; flight profiles
        # use two so overload discards old sensor tuples instead of forwarding
        # stale visual odometry.
        DeclareLaunchArgument("odometry_topic_queue_size", default_value="10"),
        DeclareLaunchArgument("odometry_sync_queue_size", default_value="10"),
        DeclareLaunchArgument("sync_max_interval", default_value="0.05"),
        DeclareLaunchArgument("oak_feature_sync_max_interval", default_value="0.001"),
        DeclareLaunchArgument("oak_input_stride", default_value="1"),
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
        executable=IfElseSubstitution(
            LaunchConfiguration("multithreaded_container"),
            "component_container_mt", "component_container"),
        name="oak_container",
        namespace="",
        output="screen",
    )

    imu_filter = LoadComposableNodes(
        target_container="/oak_container",
        condition=IfCondition(LaunchConfiguration("filter_imu")),
        composable_node_descriptions=[ComposableNode(
            package="imu_filter_madgwick",
            plugin="ImuFilterMadgwickRos",
            name="oak_imu_orientation",
            parameters=[{
                "use_sim_time": typed("use_sim_time", bool),
                "use_mag": False,
                "publish_tf": False,
                "world_frame": "enu",
            }],
            remappings=[
                ("imu/data_raw", LaunchConfiguration("imu_topic")),
                ("imu/data", "/rtabmap/imu_orientation"),
            ],
        )],
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
        "config_path": LaunchConfiguration("odom_config_path"),
        "use_sim_time": typed("use_sim_time", bool),
        "frame_id": LaunchConfiguration("base_frame_id"),
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
        "keep_latest_frame": typed("keep_latest_frame", bool),
        "keep_next_frame": typed("keep_next_frame", bool),
        "processing_queue_size": typed("processing_queue_size", int),
        # RTAB-Map core parameters are declared as strings; passing them as
        # numbers makes the node abort at startup.
        "Odom/Strategy": core_string("odom_strategy"),
        # Preserve the odometry frame through a temporary registration
        # failure. The default is zero too, but make this flight-critical
        # behavior explicit.
        "Odom/ResetCountdown": core_string("odom_reset_countdown"),
        "Odom/GuessMotion": core_string("odom_guess_motion"),
        "Odom/GuessSmoothingDelay": core_string("odom_guess_smoothing_delay"),
        "Vis/MaxFeatures": core_string("vis_max_features"),
        "Vis/MinInliers": core_string("vis_min_inliers"),
        "Vis/CorType": core_string("vis_cor_type"),
        "Vis/CorGuessWinSize": core_string("vis_cor_guess_win_size"),
        "OdomF2M/MaxSize": core_string("odom_f2m_max_size"),
        "OdomF2M/BundleAdjustment": core_string("odom_bundle_adjustment"),
        "OdomF2M/BundleAdjustmentMaxFrames": core_string(
            "odom_bundle_adjustment_max_frames"
        ),
        "Odom/KeyFrameThr": core_string("odom_key_frame_thr"),
        "Odom/VisKeyFrameThr": core_string("odom_vis_keyframe_thr"),
        "OdomF2M/MaxNewFeatures": core_string("odom_f2m_max_new_features"),
        "OdomF2M/BundleAdjustmentMinMotion": core_string("odom_bundle_min_motion"),
    }
    odometry_remappings = [
        ("imu", IfElseSubstitution(
            LaunchConfiguration("use_imu"),
            IfElseSubstitution(
                LaunchConfiguration("filter_imu"),
                "/rtabmap/imu_orientation", LaunchConfiguration("imu_topic")),
            "/rtabmap/imu_disabled")),
        ("odom", "/rtabmap/vio_odometry"),
        ("odom_info", "/rtabmap/vio_odom_info"),
        ("odom_info_lite", "/rtabmap/vio_odom_info_lite"),
    ]

    odometry = ComposableNode(
        package="rtabmap_odom",
        plugin="rtabmap_odom::RGBDOdometry",
        namespace="rtabmap",
        name="rgbd_odometry",
        parameters=[odometry_parameters | {
            "approx_sync": True,
            "approx_sync_max_interval": typed("sync_max_interval", float),
            "topic_queue_size": typed("odometry_topic_queue_size", int),
            "sync_queue_size": typed("odometry_sync_queue_size", int),
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
            "subscribe_odom_info": typed("mapper_subscribe_odom_info", bool),
            "frame_id": LaunchConfiguration("base_frame_id"),
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
            "Grid/DepthDecimation": core_string("grid_depth_decimation"),
            "Grid/RangeMin": core_string("range_min"),
            "Grid/RangeMax": core_string("range_max"),
            "Grid/PreVoxelFiltering": "true",
            "Grid/NoiseFilteringRadius": core_string("grid_noise_radius"),
            "Grid/NoiseFilteringMinNeighbors": core_string("grid_noise_min_neighbors"),
            "Grid/ClusterRadius": core_string("grid_cluster_radius"),
            "Grid/MinClusterSize": core_string("grid_min_cluster_size"),
        }],
        remappings=sensor_remappings + [
            ("odom", "/rtabmap/vio_odometry"),
            # CoreWrapper consumes the scalar health, covariance and timing
            # fields. The full F2M local bundle grows with every keyframe and
            # needlessly serializes hundreds of KiB per odometry update.
            ("odom_info", "/rtabmap/vio_odom_info_lite"),
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
                "input_stride": typed("oak_input_stride", int),
            }],
            remappings=odometry_remappings,
        )],
    )

    mapper_component = LoadComposableNodes(
        target_container="/oak_container",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("start_mapper"), "' == 'true' and '",
            LaunchConfiguration("separate_mapper_container"), "' != 'true'",
        ])),
        composable_node_descriptions=[mapper],
    )

    mapper_container = ComposableNodeContainer(
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("start_mapper"), "' == 'true' and '",
            LaunchConfiguration("separate_mapper_container"), "' == 'true'",
        ])),
        package="rclcpp_components",
        executable="component_container",
        name="rtabmap_mapper_container",
        namespace="",
        output="screen",
        composable_node_descriptions=[mapper],
    )

    octomap = Node(
        package="px4_vio_bridge",
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("start_mapper"), "' == 'true' and '",
            LaunchConfiguration("start_octomap"), "' == 'true'",
        ])),
        executable="rtabmap_octomap_node",
        name="ros_rtabmap_octomap",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": typed("use_sim_time", bool),
            "map_data_topic": "/rtabmap/mapData",
            "frame_id": "world",
            "octomap_topic": "/rtabmap/octomap",
            "test_start_clear_radius": typed("test_start_clear_radius", float),
            "octomap_use_checkpoints": typed("octomap_use_checkpoints", bool),
            **{key: typed(key, float) for key in (
                "camera_position_frd_x", "camera_position_frd_y", "camera_position_frd_z")},
            "metadata_topic": "/rtabmap/octomap_metadata",
            "markers_topic": "/rtabmap/octomap_markers",
            "ground_markers_topic": "/rtabmap/octomap_ground_markers",
            "obstacle_markers_topic": "/rtabmap/octomap_obstacle_markers",
            "max_marker_voxels": typed("max_marker_voxels", int),
            "pose_update_tolerance_m": typed("octomap_pose_update_tolerance_m", float),
            "pose_update_tolerance_rad": typed("octomap_pose_update_tolerance_rad", float),
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
            "ROS-native RTAB-Map on depthai_ros_driver_v3. filter_imu selects orientation aiding. "
            "No node publishes to /fmu/in/*. Foxglove fixed frame is 'world'. "
            "Requires the v3 driver on AMENT_PREFIX_PATH; see the module docstring."
        )),
        camera,
        replay_container,
        imu_filter,
        standard_odometry,
        tracked_odometry,
        mapper_component,
        mapper_container,
        octomap,
        foxglove,
    ])
