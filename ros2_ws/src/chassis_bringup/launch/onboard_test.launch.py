"""All-in-one real-hardware test launch for the rover Jetson.

Starts the real ODESC/CAN ros2_control stack, diff_drive_controller, local
joystick teleop, RViz, and Foxglove on one computer.  This is for bench and
short field tests only; normal operations should run the joystick and viewers
on the separate ground station.

    ros2 launch chassis_bringup onboard_test.launch.py
    ros2 launch chassis_bringup onboard_test.launch.py can_interface:=mock
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    can_interface = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="SocketCAN interface passed to the ODESC hardware backend.",
    )
    gear_ratio = DeclareLaunchArgument(
        "gear_ratio",
        default_value="48.0",
        description="Motor-shaft turns per wheel turn.",
    )
    rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Start RViz locally."
    )
    joy_control = DeclareLaunchArgument(
        "joy_control",
        default_value="true",
        description="Start the locally connected gamepad and tank-drive teleop node.",
    )

    rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("chassis_bringup"), "launch", "rover.launch.py"]
            )
        ),
        launch_arguments={
            "mode": "real",
            "can_interface": LaunchConfiguration("can_interface"),
            "gear_ratio": LaunchConfiguration("gear_ratio"),
            "rviz": LaunchConfiguration("rviz"),
            "joy_control": LaunchConfiguration("joy_control"),
        }.items(),
    )

    return LaunchDescription([can_interface, gear_ratio, rviz, joy_control, rover])
