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
    slam_publish_depth = LaunchConfiguration("slam_publish_depth")
    slam_depth_publish_hz = LaunchConfiguration("slam_depth_publish_hz")
    slam_num_features = LaunchConfiguration("slam_num_features")
    slam_path_publish_stride = LaunchConfiguration("slam_path_publish_stride")
    slam_path_size = LaunchConfiguration("slam_path_size")
    slam_publish_clouds = LaunchConfiguration("slam_publish_clouds")
    slam_grid_3d = LaunchConfiguration("slam_grid_3d")
    slam_cloud_coordinate_limit = LaunchConfiguration("slam_cloud_coordinate_limit")
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
            DeclareLaunchArgument("slam_publish_depth", default_value="true"),
            DeclareLaunchArgument("slam_depth_publish_hz", default_value="3.0"),
            DeclareLaunchArgument("slam_num_features", default_value="400"),
            DeclareLaunchArgument("slam_path_publish_stride", default_value="10"),
            DeclareLaunchArgument("slam_path_size", default_value="1000"),
            DeclareLaunchArgument("slam_publish_clouds", default_value="false"),
            DeclareLaunchArgument("slam_grid_3d", default_value="true"),
            DeclareLaunchArgument("slam_cloud_coordinate_limit", default_value="100.0"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            Node(
                package="px4_vio_bridge",
                executable="basalt_rtabmap_slam_ros2",
                arguments=[
                    "--vio-backend", "rtabmap",
                    "--vio-pose-topic", "/rtabmap/vio_pose",
                    "--fps", slam_fps,
                    "--width", slam_width,
                    "--height", slam_height,
                    "--publish-image", slam_publish_image,
                    "--image-format", slam_image_format,
                    "--image-publish-stride", slam_image_publish_stride,
                    "--image-jpeg-quality", slam_image_jpeg_quality,
                    "--publish-depth", slam_publish_depth,
                    "--depth-publish-hz", slam_depth_publish_hz,
                    "--num-features", slam_num_features,
                    "--path-publish-stride", slam_path_publish_stride,
                    "--path-size", slam_path_size,
                    "--publish-clouds", slam_publish_clouds,
                    "--grid-3d", slam_grid_3d,
                    "--cloud-coordinate-limit", slam_cloud_coordinate_limit,
                ],
                name="rtabmap_vio_slam_ros2",
                output="screen",
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"]
                    )
                ),
                condition=IfCondition(foxglove),
                launch_arguments={
                    "port": foxglove_port,
                    "topic_whitelist": "['^/tf$', '^/rtabmap/(vio_pose|pose|odometry|path|depth|camera_info)$', '^/rtabmap/image/compressed$', '^/rtabmap/image$', '^/rtabmap/(obstacle_cloud|ground_cloud)$']",
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
