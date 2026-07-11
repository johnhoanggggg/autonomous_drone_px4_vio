from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_fps = LaunchConfiguration("slam_fps")
    slam_width = LaunchConfiguration("slam_width")
    slam_height = LaunchConfiguration("slam_height")
    slam_publish_image = LaunchConfiguration("slam_publish_image")
    slam_image_format = LaunchConfiguration("slam_image_format")
    slam_image_publish_stride = LaunchConfiguration("slam_image_publish_stride")
    slam_image_jpeg_quality = LaunchConfiguration("slam_image_jpeg_quality")
    slam_path_publish_stride = LaunchConfiguration("slam_path_publish_stride")
    slam_path_size = LaunchConfiguration("slam_path_size")
    slam_publish_clouds = LaunchConfiguration("slam_publish_clouds")
    foxglove = LaunchConfiguration("foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("slam_fps", default_value="30"),
            DeclareLaunchArgument("slam_width", default_value="640"),
            DeclareLaunchArgument("slam_height", default_value="400"),
            DeclareLaunchArgument("slam_publish_image", default_value="true"),
            DeclareLaunchArgument("slam_image_format", default_value="jpeg"),
            DeclareLaunchArgument("slam_image_publish_stride", default_value="1"),
            DeclareLaunchArgument("slam_image_jpeg_quality", default_value="60"),
            DeclareLaunchArgument("slam_path_publish_stride", default_value="10"),
            DeclareLaunchArgument("slam_path_size", default_value="1000"),
            DeclareLaunchArgument("slam_publish_clouds", default_value="false"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            Node(
                package="px4_vio_bridge",
                executable="basalt_rtabmap_slam_ros2",
                arguments=[
                    "--fps",
                    slam_fps,
                    "--width",
                    slam_width,
                    "--height",
                    slam_height,
                    "--publish-image",
                    slam_publish_image,
                    "--image-format",
                    slam_image_format,
                    "--image-publish-stride",
                    slam_image_publish_stride,
                    "--image-jpeg-quality",
                    slam_image_jpeg_quality,
                    "--path-publish-stride",
                    slam_path_publish_stride,
                    "--path-size",
                    slam_path_size,
                    "--publish-clouds",
                    slam_publish_clouds,
                ],
                name="basalt_rtabmap_slam_ros2",
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
                    "topic_whitelist": "['^/basalt/pose$', '^/rtabmap/(pose|odometry|path)$', '^/rtabmap/image/compressed$', '^/rtabmap/image$', '^/rtabmap/(obstacle_cloud|ground_cloud)$']",
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
