from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    input_pose_topic = LaunchConfiguration("input_pose_topic")
    output_odometry_topic = LaunchConfiguration("output_odometry_topic")
    frame_transform = LaunchConfiguration("frame_transform")
    vio_yaw_offset_deg = LaunchConfiguration("vio_yaw_offset_deg")
    px4_local_path_size = LaunchConfiguration("px4_local_path_size")
    px4_local_path_publish_stride = LaunchConfiguration(
        "px4_local_path_publish_stride"
    )
    start_xrce_agent = LaunchConfiguration("start_xrce_agent")
    xrce_agent = LaunchConfiguration("xrce_agent")
    xrce_device = LaunchConfiguration("xrce_device")
    xrce_baud = LaunchConfiguration("xrce_baud")
    xrce_verbosity = LaunchConfiguration("xrce_verbosity")
    oak_startup_delay = LaunchConfiguration("oak_startup_delay")
    px4_startup_delay = LaunchConfiguration("px4_startup_delay")
    slam_publish_image = LaunchConfiguration("slam_publish_image")
    slam_publish_depth = LaunchConfiguration("slam_publish_depth")
    slam_depth_publish_hz = LaunchConfiguration("slam_depth_publish_hz")
    slam_publish_clouds = LaunchConfiguration("slam_publish_clouds")
    slam_num_features = LaunchConfiguration("slam_num_features")
    foxglove = LaunchConfiguration("foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")
    foxglove_topic_whitelist = LaunchConfiguration("foxglove_topic_whitelist")
    foxglove_capabilities = LaunchConfiguration("foxglove_capabilities")

    return LaunchDescription(
        [
            # Use continuous VIO for EKF2. The loop-corrected /rtabmap/pose can
            # jump when a loop closes, so it is intended for mapping by default.
            DeclareLaunchArgument(
                "input_pose_topic", default_value="/rtabmap/vio_pose"
            ),
            DeclareLaunchArgument(
                "output_odometry_topic",
                default_value="/fmu/in/vehicle_visual_odometry",
            ),
            DeclareLaunchArgument(
                "frame_transform", default_value="enu_flu_to_ned_frd"
            ),
            DeclareLaunchArgument("vio_yaw_offset_deg", default_value="0.0"),
            DeclareLaunchArgument("px4_local_path_size", default_value="300"),
            DeclareLaunchArgument(
                "px4_local_path_publish_stride", default_value="10"
            ),
            # System v3.0.1 is the version confirmed to handshake with PX4 over
            # this Raspberry Pi serial link. The systemd service owns the agent
            # by default; start_xrce_agent is a manual fallback.
            DeclareLaunchArgument("start_xrce_agent", default_value="false"),
            DeclareLaunchArgument(
                "xrce_agent", default_value="/usr/local/bin/MicroXRCEAgent"
            ),
            DeclareLaunchArgument("xrce_device", default_value="/dev/ttyAMA0"),
            DeclareLaunchArgument("xrce_baud", default_value="921600"),
            DeclareLaunchArgument("xrce_verbosity", default_value="4"),
            DeclareLaunchArgument("oak_startup_delay", default_value="5.0"),
            DeclareLaunchArgument("px4_startup_delay", default_value="0.0"),
            DeclareLaunchArgument("slam_publish_image", default_value="true"),
            DeclareLaunchArgument("slam_publish_depth", default_value="false"),
            DeclareLaunchArgument("slam_depth_publish_hz", default_value="3.0"),
            DeclareLaunchArgument("slam_publish_clouds", default_value="false"),
            DeclareLaunchArgument("slam_num_features", default_value="400"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            DeclareLaunchArgument(
                "foxglove_topic_whitelist",
                default_value=(
                    "['^/tf$', "
                    "'^/rtabmap/(vio_pose|pose|odometry|path)$', "
                    "'^/rtabmap/image(/compressed)?$', "
                    "'^/rtabmap/(depth|camera_info)$', "
                    "'^/rtabmap/(obstacle_cloud|ground_cloud)$', "
                    "'^/vio/yaw_offset/(pose|odometry|path)$', "
                    "'^/px4/local_position/(pose|odometry|path)$', "
                    "'^/fmu/in/vehicle_visual_odometry$', "
                    "'^/fmu/out/(vehicle_local_position_v1|vehicle_odometry|estimator_status_flags)$']"
                ),
            ),
            DeclareLaunchArgument(
                "foxglove_capabilities", default_value="[connectionGraph]"
            ),
            # Keep the optional launch-owned agent independent of all delayed
            # camera, bridge, and Foxglove startup work.
            ExecuteProcess(
                cmd=[
                    xrce_agent,
                    "serial",
                    "-D",
                    xrce_device,
                    "-b",
                    xrce_baud,
                    "-v",
                    xrce_verbosity,
                ],
                name="micro_xrce_agent",
                output="screen",
                condition=IfCondition(start_xrce_agent),
            ),
            TimerAction(
                period=oak_startup_delay,
                actions=[
                    GroupAction(
                        scoped=True,
                        actions=[
                            IncludeLaunchDescription(
                                AnyLaunchDescriptionSource(
                                    PathJoinSubstitution(
                                        [
                                            FindPackageShare("px4_vio_bridge"),
                                            "launch",
                                            "rtabmap_vio_slam_foxglove.launch.py",
                                        ]
                                    )
                                ),
                                # This launch owns the single combined Foxglove bridge below.
                                launch_arguments={
                                    "foxglove": "false",
                                    "slam_publish_image": slam_publish_image,
                                    "slam_publish_depth": slam_publish_depth,
                                    "slam_depth_publish_hz": slam_depth_publish_hz,
                                    "slam_publish_clouds": slam_publish_clouds,
                                    "slam_num_features": slam_num_features,
                                }.items(),
                            )
                        ],
                    )
                ],
            ),
            # PX4 conversion nodes and Foxglove have their own independent delay.
            TimerAction(
                period=px4_startup_delay,
                actions=[
                    Node(
                        package="px4_vio_bridge",
                        executable="vio_to_px4_odometry",
                        name="vio_to_px4_odometry",
                        output="screen",
                        parameters=[
                            {
                                "input_pose_topic": input_pose_topic,
                                "output_odometry_topic": output_odometry_topic,
                                "frame_transform": frame_transform,
                                "vio_yaw_offset_deg": vio_yaw_offset_deg,
                            }
                        ],
                    ),
                    Node(
                        package="px4_vio_bridge",
                        executable="px4_local_position_to_ros",
                        name="px4_local_position_to_ros",
                        output="screen",
                        parameters=[
                            {
                                "path_size": px4_local_path_size,
                                "path_publish_stride": px4_local_path_publish_stride,
                            }
                        ],
                    ),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("foxglove_bridge"),
                                    "launch",
                                    "foxglove_bridge_launch.xml",
                                ]
                            )
                        ),
                        condition=IfCondition(foxglove),
                        launch_arguments={
                            "port": foxglove_port,
                            "topic_whitelist": foxglove_topic_whitelist,
                            "service_whitelist": "['^$']",
                            "param_whitelist": "['^$']",
                            "client_topic_whitelist": "['^$']",
                            "capabilities": foxglove_capabilities,
                            "min_qos_depth": "1",
                            "max_qos_depth": "1",
                        }.items(),
                    ),
                ],
            ),
        ]
    )
