from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    detection_fps = LaunchConfiguration("detection_fps")
    detection_confidence = LaunchConfiguration("detection_confidence")
    detection_shaves = LaunchConfiguration("detection_shaves")
    person_model = LaunchConfiguration("person_model")
    person_depth_band_m = LaunchConfiguration("person_depth_band_m")
    person_point_stride = LaunchConfiguration("person_point_stride")
    slam_publish_clouds = LaunchConfiguration("slam_publish_clouds")
    slam_fps = LaunchConfiguration("slam_fps")
    slam_num_features = LaunchConfiguration("slam_num_features")
    foxglove = LaunchConfiguration("foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("detection_fps", default_value="1.4"),
            DeclareLaunchArgument("detection_confidence", default_value="0.5"),
            DeclareLaunchArgument("detection_shaves", default_value="1"),
            DeclareLaunchArgument(
                "person_model",
                default_value="luxonis/yolov6-nano:r2-coco-512x288",
            ),
            DeclareLaunchArgument("person_depth_band_m", default_value="0.45"),
            DeclareLaunchArgument("person_point_stride", default_value="2"),
            DeclareLaunchArgument("slam_publish_clouds", default_value="true"),
            DeclareLaunchArgument("slam_fps", default_value="15"),
            DeclareLaunchArgument("slam_num_features", default_value="300"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            Node(
                package="px4_vio_bridge",
                executable="basalt_rtabmap_slam_ros2",
                arguments=[
                    "--vio-backend", "rtabmap",
                    "--vio-pose-topic", "/rtabmap/vio_pose",
                    "--fps", slam_fps,
                    "--width", "640",
                    "--height", "400",
                    "--publish-image", "true",
                    "--image-format", "jpeg",
                    "--image-publish-stride", "1",
                    "--image-jpeg-quality", "60",
                    "--publish-depth", "false",
                    "--num-features", slam_num_features,
                    "--path-publish-stride", "10",
                    "--path-size", "1000",
                    "--publish-clouds", slam_publish_clouds,
                    "--grid-3d", "true",
                    "--cloud-coordinate-limit", "100.0",
                    "--detect-people", "true",
                    "--person-model", person_model,
                    "--person-detection-fps", detection_fps,
                    "--person-nn-shaves", detection_shaves,
                    "--person-confidence", detection_confidence,
                    "--person-depth-band-m", person_depth_band_m,
                    "--person-point-stride", person_point_stride,
                ],
                name="rtabmap_slam_person_detection",
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
                    "topic_whitelist": (
                        "['^/tf$', "
                        "'^/rtabmap/(vio_pose|pose|odometry|path|vio_feature_count)$', "
                        "'^/rtabmap/image/compressed$', "
                        "'^/rtabmap/(obstacle_cloud|ground_cloud)$', "
                        "'^/person/(image/compressed|points|count)$']"
                    ),
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
