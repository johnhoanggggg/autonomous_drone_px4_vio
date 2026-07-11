from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    basalt_script = LaunchConfiguration("basalt_script")
    basalt_fps = LaunchConfiguration("basalt_fps")
    basalt_width = LaunchConfiguration("basalt_width")
    basalt_height = LaunchConfiguration("basalt_height")
    basalt_usb_speed = LaunchConfiguration("basalt_usb_speed")
    basalt_imu_mode = LaunchConfiguration("basalt_imu_mode")
    basalt_publish_image = LaunchConfiguration("basalt_publish_image")
    basalt_image_format = LaunchConfiguration("basalt_image_format")
    basalt_image_publish_stride = LaunchConfiguration("basalt_image_publish_stride")
    basalt_image_jpeg_quality = LaunchConfiguration("basalt_image_jpeg_quality")
    basalt_path_publish_stride = LaunchConfiguration("basalt_path_publish_stride")
    basalt_path_size = LaunchConfiguration("basalt_path_size")
    foxglove = LaunchConfiguration("foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "basalt_script",
                default_value="/home/john/oak_d_vins_cpp/depthai-core/examples/python/RVC2/VSLAM/basalt/basalt_vio_ros2.py",
            ),
            DeclareLaunchArgument("basalt_fps", default_value="30"),
            DeclareLaunchArgument("basalt_width", default_value="640"),
            DeclareLaunchArgument("basalt_height", default_value="400"),
            DeclareLaunchArgument("basalt_usb_speed", default_value="high"),
            DeclareLaunchArgument("basalt_imu_mode", default_value="raw"),
            DeclareLaunchArgument("basalt_publish_image", default_value="true"),
            DeclareLaunchArgument("basalt_image_format", default_value="jpeg"),
            DeclareLaunchArgument("basalt_image_publish_stride", default_value="1"),
            DeclareLaunchArgument("basalt_image_jpeg_quality", default_value="60"),
            DeclareLaunchArgument("basalt_path_publish_stride", default_value="10"),
            DeclareLaunchArgument("basalt_path_size", default_value="1000"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            ExecuteProcess(
                cmd=[
                    "python3",
                    basalt_script,
                    "--fps",
                    basalt_fps,
                    "--width",
                    basalt_width,
                    "--height",
                    basalt_height,
                    "--usb-speed",
                    basalt_usb_speed,
                    "--imu-mode",
                    basalt_imu_mode,
                    "--publish-image",
                    basalt_publish_image,
                    "--image-format",
                    basalt_image_format,
                    "--image-publish-stride",
                    basalt_image_publish_stride,
                    "--image-jpeg-quality",
                    basalt_image_jpeg_quality,
                    "--path-publish-stride",
                    basalt_path_publish_stride,
                    "--path-size",
                    basalt_path_size,
                ],
                name="basalt_vio_ros2",
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
                    "topic_whitelist": "['^/basalt/(pose|odometry|path)$', '^/basalt/image/compressed$', '^/basalt/image$']",
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
