from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare

import glob
import os
import sys
from ament_index_python.packages import get_package_prefix, get_package_share_directory

desc_share = get_package_share_directory("robot_description")
share_root = os.path.dirname(desc_share)
ign_ros2_control_plugin_path = os.path.join(
    get_package_prefix("ign_ros2_control"), "lib"
)

# Ignition/Gazebo GUI plugin dir, discovered from the active environment prefix
# instead of a hardcoded path. `ros2 launch` runs under the Pixi env's python, so
# sys.prefix is that env; fall back to CONDA_PREFIX. The ign-gazebo-<N> version is
# globbed so a Fortress point-release bump doesn't break this.
_env_prefix = os.environ.get("CONDA_PREFIX") or sys.prefix
_gui_plugin_dirs = sorted(
    glob.glob(os.path.join(_env_prefix, "lib", "ign-gazebo-*", "plugins", "gui"))
    + glob.glob(os.path.join(_env_prefix, "lib", "gz-sim-*", "plugins", "gui"))
)
_gui_plugin_path = _gui_plugin_dirs[-1] if _gui_plugin_dirs else ""

# environment variables required for GZ to render models properly
set_env = [
    SetEnvironmentVariable("IGN_GAZEBO_MODEL_PATH", share_root),
    SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", share_root),
    SetEnvironmentVariable("GZ_SIM_MODEL_PATH", share_root),
    SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", share_root),
    # Ignition Gazebo loads model system plugins from this path.  The control
    # plugin is supplied by the Pixi / ROS environment, not this workspace.
    SetEnvironmentVariable(
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH", ign_ros2_control_plugin_path
    ),
] + (
    # Only needed for the Ignition GUI's own side panels; skipped cleanly when the
    # dir can't be found (e.g. headless under xvfb).
    [
        SetEnvironmentVariable("IGN_GUI_PLUGIN_PATH", _gui_plugin_path),
        SetEnvironmentVariable("QML2_IMPORT_PATH", _gui_plugin_path),
    ]
    if _gui_plugin_path
    else []
)

declare_args = [
        DeclareLaunchArgument(
            "description_pkg",
            default_value="robot_description",
            description="Package that contains the robot URDF/Xacro in its share/ directory.",
        ),
        DeclareLaunchArgument(
            "xacro_file",
            default_value="urdf/robot.urdf.xacro",
            description="Path to the robot Xacro (relative to the description package share).",
        ),

        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="Path to the robot world file (relative to the description package share).",
        ),
        DeclareLaunchArgument("x", default_value="0.0", description="Spawn X (m)."),
        DeclareLaunchArgument("y", default_value="0.0", description="Spawn Y (m)."),
        DeclareLaunchArgument("z", default_value="0.2", description="Spawn Z (m)."),
        DeclareLaunchArgument("yaw", default_value="0.0", description="Spawn yaw (rad)."),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Also start RViz2 with the drivetrain view (viz.launch.py). "
            "Default false keeps the sim launch behaviour unchanged.",
        ),
        DeclareLaunchArgument(
            "foxglove",
            default_value="false",
            description="Also start the foxglove_bridge WebSocket server on :8765.",
        ),
        DeclareLaunchArgument(
            "odesc_shadow",
            default_value="false",
            description=(
                "Also run a namespaced ODESC controller manager against vCAN. It receives "
                "the same cmd_vel as Gazebo, so system can exercise CAN commands while Gazebo "
                "remains the physics plant. Requires tooling/can-up vcan0 first."
            ),
        ),
]


def _launch_description(ctx):
    description_pkg = LaunchConfiguration("description_pkg")
    xacro_file = LaunchConfiguration("xacro_file")
    world_file = LaunchConfiguration("world_file")

    # Entity Start Position
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    # select + published selected urdf to /robot_description
    xacro_path = PathJoinSubstitution([
        FindPackageShare(description_pkg),
        xacro_file,
    ])

    world_path = PathJoinSubstitution([
        FindPackageShare(description_pkg),
        world_file,
    ])
    robot_description = ParameterValue(
        Command(["xacro", " ",xacro_path]),
        value_type=str,
    )

    gazebo_params_file = PathJoinSubstitution([
        FindPackageShare("chassis_bringup"),
        "config",
        "gz_params.yaml"
    ])

    # launch gazebo sim
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        # gz launch always declares gz_args as a string and PathJoinSubstiution is lazily 
        # evaluated at runtime so  we must eval here to pass as a gz_arg
        launch_arguments={
            #"gz_args": f"-r -v 4 {world_path.perform(ctx)}",
            "gz_args": f"-r -v 4 empty.sdf",
            "extra_gz_args": f"--ros-args --params-file {gazebo_params_file}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    # This appends your description package share directory to the resource path.
    set_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution([FindPackageShare(description_pkg)]),
    )

    # publish robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Keep all Gazebo/ROS topic mappings in config files so node
    # can be maintained without changing this launch file.
    bridge_config = PathJoinSubstitution([
        FindPackageShare("chassis_bringup"),
        "config",
        "config.yaml",
    ])

    zed_bridge_config = PathJoinSubstitution([
        FindPackageShare("chassis_bringup"),
        "config",
        "zed_config.yaml",
    ])

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    zed_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": zed_bridge_config}],
        output="screen",
    )

    #Spawn the robot from /robot_description into Gazebo ---
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "BILLEE_BOT",
            "-topic", "robot_description",
            "-x", x, "-y", y, "-z", z,
            "-Y", yaw,
        ],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "joint_state_broadcaster"]
    )

    # Opt-in viewers. rviz/foxglove default false -> this include adds nothing
    # unless asked, so the default sim launch graph is unchanged.
    viz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("chassis_bringup"), "launch", "viz.launch.py"
            ])
        ),
        launch_arguments={
            "rviz": LaunchConfiguration("rviz"),
            "foxglove": LaunchConfiguration("foxglove"),
            "use_sim_time": "true",
        }.items(),
    )

    # A second, namespaced controller manager mirrors the command into the
    # ODESC/vCAN path. Gazebo remains authoritative for robot physics and TF.
    odesc_shadow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("chassis_bringup"), "launch", "odesc_shadow.launch.py"
            ])
        ),
        launch_arguments={
            "description_pkg": description_pkg,
            "xacro_file": xacro_file,
            "can_interface": 'vcan0',
        }.items(),
        condition=IfCondition(LaunchConfiguration("odesc_shadow")),
    )

    return set_env + [
            set_resource_path,
            gz_sim_launch,
            gz_bridge,
            zed_gz_bridge,
            robot_state_publisher,
            spawn_entity,
            diff_drive_controller_spawner,
            odesc_shadow,
            viz,
        ]

def generate_launch_description():
    #Must use an Opaque function because there are some lazily eval expressions that must be executed
    #beforehand because the args params for launch actions depend on them (world.sdf)
    #IMPORTANT: we must declare args before passing the OpaqueFunction or else it will
    #try to reference LaunchConfiguration objs that do not yet exist (havent been delcared)
    return LaunchDescription(declare_args + [
        OpaqueFunction(function=_launch_description)
    ])
