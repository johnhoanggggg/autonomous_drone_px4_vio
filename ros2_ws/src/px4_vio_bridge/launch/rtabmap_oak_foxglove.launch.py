from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    foxglove = LaunchConfiguration("foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rtabmap_script",
                default_value="/home/john/autonomous_drone_px4_vio/scripts/rtabmap_vio_ros2.py",
            ),
            DeclareLaunchArgument("rtabmap_fps", default_value="30"),
            DeclareLaunchArgument("rtabmap_width", default_value="640"),
            DeclareLaunchArgument("rtabmap_height", default_value="400"),
            DeclareLaunchArgument("rtabmap_publish_image", default_value="true"),
            DeclareLaunchArgument("rtabmap_image_format", default_value="jpeg"),
            DeclareLaunchArgument("rtabmap_image_publish_stride", default_value="1"),
            DeclareLaunchArgument("rtabmap_image_jpeg_quality", default_value="60"),
            DeclareLaunchArgument("rtabmap_num_features", default_value="1000"),
            DeclareLaunchArgument("rtabmap_path_publish_stride", default_value="1"),
            DeclareLaunchArgument("rtabmap_path_size", default_value="5000"),
            DeclareLaunchArgument("rtabmap_trajectory_log", default_value="vslam_trajectory.txt"),
            DeclareLaunchArgument("rtabmap_trajectory_flush_stride", default_value="30"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
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
                    "topic_whitelist": "['^/rtabmap/(path|image|odometry)$', '^/rtabmap/image/compressed$', '^/basalt/pose$']",
                    "service_whitelist": "['^$']",
                    "param_whitelist": "['^$']",
                    "client_topic_whitelist": "['^$']",
                    "capabilities": "[connectionGraph]",
                    "min_qos_depth": "1",
                    "max_qos_depth": "1",
                }.items(),
            ),
        ]
    )
