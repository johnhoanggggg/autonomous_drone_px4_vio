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
        DeclareLaunchArgument("odometry_rate_hz", default_value="10.0"),
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
            "params_file": PathJoinSubstitution([
                FindPackageShare("px4_vio_bridge"), "config", "oak_ros_rtabmap.yaml"
            ]),
            "rectify_rgb": "true",
            "pointcloud.enable": "false",
            "use_rviz": "false",
        }.items(),
    )

    sensor_remappings = [
        ("rgb/image", "/oak/rgb/image_rect"),
        ("depth/image", "/oak/stereo/image_raw"),
        ("rgb/camera_info", "/oak/rgb/camera_info"),
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
            "wait_imu_to_init": True,
            "always_check_imu_tf": True,
            "approx_sync": True,
            "approx_sync_max_interval": 0.20,
            "topic_queue_size": 5,
            "sync_queue_size": 5,
            "qos": 2,
            "qos_camera_info": 2,
            "expected_update_rate": typed("odometry_rate_hz", float),
            "max_update_rate": typed("odometry_rate_hz", float),
            "Odom/Strategy": "0",
            "Vis/MaxFeatures": "1000",
            "OdomF2M/MaxSize": "2000",
        }],
        remappings=sensor_remappings + [
            ("imu", "/oak/imu/data"),
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
            "approx_sync_max_interval": 0.20,
            "topic_queue_size": 5,
            "sync_queue_size": 5,
            "qos_image": 2,
            "qos_camera_info": 2,
            "qos_odom": 2,
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
                "'^/oak/(rgb/(image_rect(/compressed)?|camera_info)|stereo/image_raw|imu/data)$', "
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
            "Foxglove fixed frame is 'world'."
        )),
        camera,
        ros_mapping_components,
        octomap,
        foxglove,
    ])
