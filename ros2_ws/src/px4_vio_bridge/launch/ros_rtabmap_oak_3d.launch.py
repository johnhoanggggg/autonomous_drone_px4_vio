from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node
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
        DeclareLaunchArgument(
            "camera_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("px4_vio_bridge"),
                "config",
                "oak_ros_rtabmap.yaml",
            ]),
        ),
        DeclareLaunchArgument("rectify_rgb", default_value="true"),
        DeclareLaunchArgument(
            "visual_image_topic", default_value="/oak/rgb/image_rect"
        ),
        DeclareLaunchArgument(
            "visual_camera_info_topic", default_value="/oak/rgb/camera_info"
        ),
        DeclareLaunchArgument("odometry_rate_hz", default_value="10.0"),
        # Odometry cost knobs. Defaults are the measured-best set from the
        # sweep: optical-flow correspondences with bundle adjustment off cost
        # 47.6 ms per frame against 97.6 ms for the 1000-feature frame-to-map
        # default, which is what lifts odometry from 5.8 Hz to a full 10 Hz.
        DeclareLaunchArgument("odom_strategy", default_value="0"),
        DeclareLaunchArgument("vis_max_features", default_value="600"),
        DeclareLaunchArgument("vis_cor_type", default_value="1"),
        DeclareLaunchArgument("odom_f2m_max_size", default_value="1000"),
        DeclareLaunchArgument("odom_image_decimation", default_value="1"),
        DeclareLaunchArgument("odom_bundle_adjustment", default_value="0"),
        DeclareLaunchArgument("odom_key_frame_thr", default_value="0.3"),
        # Sync/QoS knobs. The device rect-pair and depth streams do not carry
        # the same stamps as the colour pipeline, so the tolerance has to be
        # settable per topology.
        DeclareLaunchArgument("sync_max_interval", default_value="0.05"),
        DeclareLaunchArgument("sensor_qos", default_value="2"),
        # The OAK IMU stream reaches the host well behind the image stream.
        # RTAB-Map requires an IMU sample newer than each image, so a lagging
        # IMU makes it drop frames. These allow a vision-only comparison.
        DeclareLaunchArgument("use_imu", default_value="true"),
        DeclareLaunchArgument("imu_topic", default_value="/oak/imu/data"),
        DeclareLaunchArgument("mapping_rate_hz", default_value="1.0"),
        DeclareLaunchArgument("voxel_size", default_value="0.05"),
        DeclareLaunchArgument("range_min", default_value="0.30"),
        DeclareLaunchArgument("range_max", default_value="4.0"),
        DeclareLaunchArgument("max_marker_voxels", default_value="50000"),
    ]

    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("depthai_ros_driver"), "launch", "camera.launch.py"
            ])
        ),
        launch_arguments={
            "name": "oak",
            "camera_model": "OAK-D-LITE",
            "override_cam_model": "true",
            "publish_tf_from_calibration": "true",
            # Let the driver publish the calibrated IMU transform. The
            # installed OAK-D-Lite URDF description has no IMU link.
            "imu_from_descr": "false",
            "params_file": LaunchConfiguration("camera_config"),
            "rectify_rgb": LaunchConfiguration("rectify_rgb"),
            "pointcloud.enable": "false",
            "use_rviz": "false",
        }.items(),
    )

    sensor_remappings = [
        ("rgb/image", LaunchConfiguration("visual_image_topic")),
        ("depth/image", "/oak/stereo/image_raw"),
        ("rgb/camera_info", LaunchConfiguration("visual_camera_info_topic")),
    ]
    odometry = ComposableNode(
        package="rtabmap_odom",
        plugin="rtabmap_odom::RGBDOdometry",
        namespace="rtabmap",
        name="rgbd_odometry",
        parameters=[{
            "frame_id": "oak",
            "odom_frame_id": "rtabmap_odom",
            "publish_tf": True,
            "publish_null_when_lost": True,
            "wait_imu_to_init": typed("use_imu", bool),
            "always_check_imu_tf": typed("use_imu", bool),
            "approx_sync": True,
            "approx_sync_max_interval": typed("sync_max_interval", float),
            "topic_queue_size": 10,
            "sync_queue_size": 10,
            "qos": typed("sensor_qos", int),
            "qos_camera_info": typed("sensor_qos", int),
            "expected_update_rate": typed("odometry_rate_hz", float),
            "max_update_rate": typed("odometry_rate_hz", float),
            "Odom/Strategy": core_string("odom_strategy"),
            "Vis/MaxFeatures": core_string("vis_max_features"),
            "Vis/CorType": core_string("vis_cor_type"),
            "OdomF2M/MaxSize": core_string("odom_f2m_max_size"),
            "Odom/ImageDecimation": core_string("odom_image_decimation"),
            "OdomF2M/BundleAdjustment": core_string("odom_bundle_adjustment"),
            "Odom/KeyFrameThr": core_string("odom_key_frame_thr"),
        }],
        remappings=sensor_remappings + [
            ("imu", LaunchConfiguration("imu_topic")),
            ("odom", "/rtabmap/vio_odometry"),
            ("odom_info", "/rtabmap/vio_odom_info"),
        ],
    )
    mapper = ComposableNode(
        package="rtabmap_slam",
        plugin="rtabmap_slam::CoreWrapper",
        namespace="rtabmap",
        name="rtabmap",
        parameters=[{
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
            # An empty path uses RTAB-Map's in-memory database. The benchmark
            # never loads or overwrites a flight map.
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
    ros_mapping_components = LoadComposableNodes(
        target_container="/oak_container",
        composable_node_descriptions=[odometry, mapper],
    )

    octomap = Node(
        package="px4_vio_bridge",
        executable="rtabmap_octomap_node",
        name="ros_rtabmap_octomap",
        output="screen",
        emulate_tty=True,
        parameters=[{
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
                "'^/oak/((rgb|right)/(image_rect(/compressed)?|camera_info)|stereo/image_raw|imu/data)$', "
                "'^/rtabmap/(vio_odometry|vio_odom_info|mapPath|localization_pose|info|"
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
            "ROS-native RTAB-Map migration benchmark: no node publishes to /fmu/in/*. "
            "Foxglove fixed frame is 'world'. depthai_ros_driver 2.12.2 delivers "
            "/oak/imu/data ~0.4 s behind the images, which makes rtabmap_odom drop "
            "nearly every frame; pass use_imu:=false for a vision-only run until the "
            "v3 driver lands. Vision-only odometry is too noisy to fly on."
        )),
        camera,
        ros_mapping_components,
        octomap,
        foxglove,
    ])
