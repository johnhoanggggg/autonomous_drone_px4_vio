"""Launch the bounded PX4 offboard position-hold/yaw test."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("hover_height", default_value="0.3"),
        DeclareLaunchArgument("hold_time", default_value="10.0"),
        DeclareLaunchArgument("yaw_angle_deg", default_value="45.0"),
        DeclareLaunchArgument("yaw_rate_deg", default_value="20.0"),
        DeclareLaunchArgument("yaw_tolerance_deg", default_value="5.0"),
        DeclareLaunchArgument("yaw_timeout", default_value="8.0"),
        DeclareLaunchArgument("return_hold_time", default_value="2.0"),
        DeclareLaunchArgument("climb_timeout", default_value="8.0"),
        DeclareLaunchArgument("max_flight_time", default_value="30.0"),
        DeclareLaunchArgument("tracking_loss_land", default_value="true"),
        DeclareLaunchArgument("keyboard_kill", default_value="true"),
        DeclareLaunchArgument("keyboard_land", default_value="true"),
    ]
    node = Node(
        package="px4_vio_bridge",
        executable="offboard_hold_yaw",
        name="offboard_hold_yaw",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "auto_arm": typed("auto_arm", bool),
                "hover_height": typed("hover_height", float),
                "hold_time": typed("hold_time", float),
                "yaw_angle_deg": typed("yaw_angle_deg", float),
                "yaw_rate_deg": typed("yaw_rate_deg", float),
                "yaw_tolerance_deg": typed("yaw_tolerance_deg", float),
                "yaw_timeout": typed("yaw_timeout", float),
                "return_hold_time": typed("return_hold_time", float),
                "climb_timeout": typed("climb_timeout", float),
                "max_flight_time": typed("max_flight_time", float),
                "tracking_loss_land": typed("tracking_loss_land", bool),
                "keyboard_kill": typed("keyboard_kill", bool),
                "keyboard_land": typed("keyboard_land", bool),
            }
        ],
    )
    return LaunchDescription(arguments + [node])
