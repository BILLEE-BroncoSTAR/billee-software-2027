"""One-command ground-station bring-up: joystick + RViz + Foxglove bridge.

    ros2 launch chassis_bringup ground_station.launch.py
    ros2 launch chassis_bringup ground_station.launch.py use_sim_time:=false   # real hardware
    ros2 launch chassis_bringup ground_station.launch.py use_sim_time:=false view:=openpose
    ros2 launch chassis_bringup ground_station.launch.py fprime_gds:=true

Starts:
  - viz.launch.py  -> rviz2 (view:= picks drivetrain/cloud/openpose) + foxglove_bridge :8765
  - teleop.launch.py -> joy_node + joy_tank_drive  (publishes cmd_vel over DDS);
    skip with joystick:=false (e.g. headless / Mac container / Foxglove-only station)
  - optionally opens the F' GDS web UI in a browser (fprime_gds:=true) at fprime_gds_url:=

The ground station shares ROS_DOMAIN_ID (pixi.toml) and LAN with the rover, so the local
bridge exposes the rover's topics; point Foxglove Studio at ws://localhost:8765.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

declare_args = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="true when viewing the Gazebo sim (TF timestamps track /clock); "
        "false for real hardware. Applies to RViz + the bridge, not teleop.",
    ),
    DeclareLaunchArgument("rviz", default_value="true", description="Start RViz2."),
    DeclareLaunchArgument(
        "foxglove",
        default_value="true",
        description="Start the local foxglove_bridge WebSocket server on :8765.",
    ),
    DeclareLaunchArgument(
        "joystick",
        default_value="true",
        description="Start teleop (joy_node + joy_tank_drive). Set false for a "
        "view-only station or where no /dev/input gamepad exists (Mac container, CI).",
    ),
    DeclareLaunchArgument(
        "view",
        default_value="drivetrain",
        choices=["drivetrain", "cloud", "openpose"],
        description="RViz layout: drivetrain (fixed frame odom), cloud (ZED point cloud) or "
        "openpose (point cloud + skeletons). Match the rover's camera:= option.",
    ),
    DeclareLaunchArgument(
        "fprime_gds",
        default_value="false",
        description="Open the F' GDS web UI in a browser.",
    ),
    DeclareLaunchArgument(
        "fprime_gds_url",
        default_value="http://192.168.4.73:5000",
        description="Address opened when fprime_gds:=true.",
    ),
]


RVIZ_CONFIGS = {  # view:= -> (package, file under share/<package>/rviz/)
    "drivetrain": ("chassis_bringup", "drivetrain.rviz"),
    "cloud": ("people", "zed_points.rviz"),
    "openpose": ("people", "people_rover.rviz"),
}


def _viz(context):
    package, config = RVIZ_CONFIGS[LaunchConfiguration("view").perform(context)]
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("chassis_bringup"), "launch", "viz.launch.py"])
        ),
        launch_arguments={
            "rviz": LaunchConfiguration("rviz"),
            "foxglove": LaunchConfiguration("foxglove"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "rviz_config": PathJoinSubstitution([FindPackageShare(package), "rviz", config]),
        }.items(),
    )]


def generate_launch_description():
    viz = OpaqueFunction(function=_viz)

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("teleop"), "launch", "teleop.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("joystick")),
    )

    # `python3 -m webbrowser` honours $BROWSER and falls back through installed
    # browsers; use `xdg-open` instead if that suits the ground-station setup.
    # Non-fatal: if no browser is found it just logs and exits.
    open_gds = ExecuteProcess(
        cmd=["python3", "-m", "webbrowser", "-t", LaunchConfiguration("fprime_gds_url")],
        condition=IfCondition(LaunchConfiguration("fprime_gds")),
        output="screen",
    )

    return LaunchDescription(declare_args + [viz, teleop, open_gds])
