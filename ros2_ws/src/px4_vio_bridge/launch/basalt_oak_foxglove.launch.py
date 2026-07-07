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
                    "topic_whitelist": "['^/basalt/(pose|odometry|path|image)$']",
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
