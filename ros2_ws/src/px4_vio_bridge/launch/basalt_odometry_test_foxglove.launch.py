from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fps = LaunchConfiguration("fps")
    frame_id = LaunchConfiguration("frame_id")
    child_frame_id = LaunchConfiguration("child_frame_id")
    path_size = LaunchConfiguration("path_size")
    foxglove = LaunchConfiguration("foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("fps", default_value="30.0"),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("child_frame_id", default_value="basalt_body"),
            DeclareLaunchArgument("path_size", default_value="900"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("foxglove_port", default_value="8765"),
            Node(
                package="px4_vio_bridge",
                executable="basalt_odometry_test",
                name="basalt_odometry_test",
                output="screen",
                parameters=[
                    {
                        "fps": ParameterValue(fps, value_type=float),
                        "frame_id": frame_id,
                        "child_frame_id": child_frame_id,
                        "path_size": ParameterValue(path_size, value_type=int),
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
                    "topic_whitelist": "['^/basalt/(pose|odometry|path)$']",
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
