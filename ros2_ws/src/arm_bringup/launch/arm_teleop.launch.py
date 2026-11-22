"""Joystick joint-jog teleop for the arm (run arm.launch.py first).

    ros2 launch arm_bringup arm_teleop.launch.py
    ros2 launch arm_bringup arm_teleop.launch.py joy:=false   # joy_node already running
                                                             # (e.g. from teleop.launch.py)
See scripts/arm_joy_teleop.py for the controls.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("joy", default_value="true",
                              description="Start a joy_node (false if one is already running)."),
        DeclareLaunchArgument("device_id", default_value="0"),
        DeclareLaunchArgument("speed_scale", default_value="1.0",
                              description="Full-stick speed as a fraction of test_velocity."),
        DeclareLaunchArgument("deadzone", default_value="0.1",
                              description="Stick travel ignored around centre (0-1) by the arm "
                              "jog; raise it if a joint creeps with the stick let go."),
        Node(
            package="joy",
            executable="joy_node",
            condition=IfCondition(LaunchConfiguration("joy")),
            # autorepeat keeps /joy flowing while the stick is held still; the
            # teleop's watchdog stops the arm if it goes quiet.
            parameters=[{"device_id": LaunchConfiguration("device_id"),
                         "deadzone": 0.05,
                         "autorepeat_rate": 20.0}],
        ),
        Node(
            package="arm_bringup",
            executable="arm_joy_teleop.py",
            # As floats: "1" would otherwise arrive as an integer and fail the float parameter.
            parameters=[{"speed_scale": ParameterValue(LaunchConfiguration("speed_scale"),
                                                       value_type=float),
                         "deadzone": ParameterValue(LaunchConfiguration("deadzone"),
                                                    value_type=float)}],
            output="screen",
        ),
    ])
