from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rtabmap_script = LaunchConfiguration("rtabmap_script")
    rtabmap_fps = LaunchConfiguration("rtabmap_fps")
    rtabmap_width = LaunchConfiguration("rtabmap_width")
    rtabmap_height = LaunchConfiguration("rtabmap_height")
    rtabmap_publish_image = LaunchConfiguration("rtabmap_publish_image")
    rtabmap_image_format = LaunchConfiguration("rtabmap_image_format")
    rtabmap_image_publish_stride = LaunchConfiguration("rtabmap_image_publish_stride")
    rtabmap_image_jpeg_quality = LaunchConfiguration("rtabmap_image_jpeg_quality")
    rtabmap_num_features = LaunchConfiguration("rtabmap_num_features")
    rtabmap_path_publish_stride = LaunchConfiguration("rtabmap_path_publish_stride")
    rtabmap_path_size = LaunchConfiguration("rtabmap_path_size")
    rtabmap_trajectory_log = LaunchConfiguration("rtabmap_trajectory_log")
    rtabmap_trajectory_flush_stride = LaunchConfiguration("rtabmap_trajectory_flush_stride")
    input_pose_topic = LaunchConfiguration("input_pose_topic")
    output_odometry_topic = LaunchConfiguration("output_odometry_topic")
    frame_transform = LaunchConfiguration("frame_transform")
    vio_yaw_offset_deg = LaunchConfiguration("vio_yaw_offset_deg")
    px4_local_path_size = LaunchConfiguration("px4_local_path_size")
    px4_local_path_publish_stride = LaunchConfiguration("px4_local_path_publish_stride")
    xrce_agent = LaunchConfiguration("xrce_agent")
    xrce_device = LaunchConfiguration("xrce_device")
    xrce_baud = LaunchConfiguration("xrce_baud")
    xrce_verbosity = LaunchConfiguration("xrce_verbosity")
    foxglove = LaunchConfiguration("foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")
    foxglove_topic_whitelist = LaunchConfiguration("foxglove_topic_whitelist")
    foxglove_capabilities = LaunchConfiguration("foxglove_capabilities")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rtabmap_script",
                default_value="/home/john/autonomous_drone_px4_vio/scripts/rtabmap_vio_ros2.py",
            ),
            DeclareLaunchArgument("rtabmap_fps", default_value="30"),
            DeclareLaunchArgument("rtabmap_width", default_value="640"),
            DeclareLaunchArgument("rtabmap_height", default_value="400"),
            DeclareLaunchArgument("rtabmap_publish_image", default_value="false"),
            DeclareLaunchArgument("rtabmap_image_format", default_value="jpeg"),
            DeclareLaunchArgument("rtabmap_image_publish_stride", default_value="1"),
            DeclareLaunchArgument("rtabmap_image_jpeg_quality", default_value="60"),
            DeclareLaunchArgument("rtabmap_num_features", default_value="1000"),
            DeclareLaunchArgument("rtabmap_path_publish_stride", default_value="10"),
            DeclareLaunchArgument("rtabmap_path_size", default_value="1000"),
            DeclareLaunchArgument("rtabmap_trajectory_log", default_value="none"),
            DeclareLaunchArgument("rtabmap_trajectory_flush_stride", default_value="30"),
            DeclareLaunchArgument("input_pose_topic", default_value="/basalt/pose"),
            DeclareLaunchArgument("output_odometry_topic", default_value="/fmu/in/vehicle_visual_odometry"),
            DeclareLaunchArgument("frame_transform", default_value="enu_flu_to_ned_frd"),
            DeclareLaunchArgument("vio_yaw_offset_deg", default_value="0.0"),
            DeclareLaunchArgument("px4_local_path_size", default_value="300"),
            DeclareLaunchArgument("px4_local_path_publish_stride", default_value="10"),
            # System v3.0.1 agent is the confirmed-working one on this serial link.
            # The project-local v2.4.3 build did not complete a PX4 handshake (2026-07-07).
            DeclareLaunchArgument(
                "xrce_agent",
                default_value="/usr/local/bin/MicroXRCEAgent",
            ),
            DeclareLaunchArgument("xrce_device", default_value="/dev/ttyAMA0"),
            DeclareLaunchArgument("xrce_baud", default_value="921600"),
            DeclareLaunchArgument("xrce_verbosity", default_value="4"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            DeclareLaunchArgument(
                "foxglove_topic_whitelist",
                default_value="['^/rtabmap/(path|odometry)$', '^/rtabmap/image/compressed$', '^/basalt/pose$', '^/vio/yaw_offset/(pose|odometry|path)$', '^/px4/local_position/(pose|odometry|path)$', '^/fmu/in/vehicle_visual_odometry$', '^/fmu/out/(vehicle_local_position_v1|vehicle_odometry|estimator_status_flags)$']",
            ),
            DeclareLaunchArgument("foxglove_capabilities", default_value="[connectionGraph]"),
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
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    rtabmap_script,
                    "--fps",
                    rtabmap_fps,
                    "--width",
                    rtabmap_width,
                    "--height",
                    rtabmap_height,
                    "--publish-image",
                    rtabmap_publish_image,
                    "--image-format",
                    rtabmap_image_format,
                    "--image-publish-stride",
                    rtabmap_image_publish_stride,
                    "--image-jpeg-quality",
                    rtabmap_image_jpeg_quality,
                    "--num-features",
                    rtabmap_num_features,
                    "--path-publish-stride",
                    rtabmap_path_publish_stride,
                    "--path-size",
                    rtabmap_path_size,
                    "--trajectory-log",
                    rtabmap_trajectory_log,
                    "--trajectory-flush-stride",
                    rtabmap_trajectory_flush_stride,
                ],
                name="rtabmap_vio_ros2",
                output="screen",
            ),
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
        ]
    )
