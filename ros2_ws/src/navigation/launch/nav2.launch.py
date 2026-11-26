"""Nav2 for BILLEE, in one of two modes, from navigation/config/nav2_params.yaml.

    nav_mode:=debug  controller_server + planner_server and their costmaps, one lifecycle manager.
                     No bt_navigator, so nothing accepts a goal: drive by joystick and watch the
                     costmaps and EKFs.
    nav_mode:=full   nav2_bringup's navigation_launch.py: adds bt_navigator (NavigateToPose,
                     /goal_pose), behavior_server, velocity_smoother, smoother_server and
                     waypoint_follower, all under lifecycle_manager_navigation.

Localization is NOT started here; it is pose_estim.launch.py (sim) or rover.launch.py vio:=true
(rover). Nav2's command output is /cmd_vel in both modes; chassis_bringup's twist_mux forwards it
to diff_drive_controller below the joystick's priority.

The sim and the rover share one params file. These arguments rewrite it at launch:

    use_sim_time  every node
    global_frame  global costmap + bt_navigator. map = the sim's global EKF; odom = the rover,
                  which has no map frame yet
    bt_xml        the NavigateToPose tree (default behavior_trees/navigate_to_pose_keep_nearby.xml)
    cloud_topic   both voxel layers' PointCloud2. /depth_cam/points (sim) or /zed/points (ZED)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

# Nav2's command output (controller in debug; velocity_smoother + behavior_server in full).
NAV_CMD_VEL = "/cmd_vel"

declare_args = [
    DeclareLaunchArgument(
        "nav_mode", default_value="debug", choices=["debug", "full"],
        description="debug = planner/controller servers and costmaps only (no goals); "
        "full = the whole Nav2 navigation stack (goals via NavigateToPose or /goal_pose).",
    ),
    DeclareLaunchArgument("use_sim_time", default_value="true"),
    DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("navigation"), "config", "nav2_params.yaml"]),
        description="Nav2 parameters.",
    ),
    DeclareLaunchArgument(
        "global_frame", default_value="map", choices=["map", "odom"],
        description="Global costmap / bt_navigator frame. map needs a map -> odom publisher "
        "(the sim's global EKF).",
    ),
    DeclareLaunchArgument(
        "cloud_topic", default_value="/depth_cam/points",
        description="PointCloud2 for the costmaps' voxel layers.",
    ),
    DeclareLaunchArgument(
        "bt_xml",
        default_value=PathJoinSubstitution(
            [FindPackageShare("navigation"), "behavior_trees", "navigate_to_pose_keep_nearby.xml"]),
        description="NavigateToPose behavior tree (nav_mode:=full).",
    ),
    DeclareLaunchArgument("autostart", default_value="true"),
    DeclareLaunchArgument("log_level", default_value="info"),
]


def _setup(context):
    mode = LaunchConfiguration("nav_mode").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    cloud_topic = LaunchConfiguration("cloud_topic")
    global_frame = LaunchConfiguration("global_frame")

    params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        param_rewrites={
            "use_sim_time": use_sim_time,
            "global_costmap.global_costmap.ros__parameters.global_frame": global_frame,
            "bt_navigator.ros__parameters.global_frame": global_frame,
            "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml":
                LaunchConfiguration("bt_xml"),
            "global_costmap.global_costmap.ros__parameters.voxel_layer.pointcloud.topic":
                cloud_topic,
            "local_costmap.local_costmap.ros__parameters.voxel_layer.pointcloud.topic":
                cloud_topic,
        },
        convert_types=True,
    )

    if mode == "full":
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])),
            launch_arguments={
                "params_file": params,
                "use_sim_time": use_sim_time,
                "autostart": LaunchConfiguration("autostart"),
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        )]

    # debug: the two servers that own the costmaps, nothing that drives.
    log_args = ["--ros-args", "--log-level", LaunchConfiguration("log_level")]
    return [
        Node(
            package="nav2_controller", executable="controller_server", output="screen",
            parameters=[params], arguments=log_args,
            remappings=[("cmd_vel", NAV_CMD_VEL)],
        ),
        Node(
            package="nav2_planner", executable="planner_server", name="planner_server",
            output="screen", parameters=[params], arguments=log_args,
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen", arguments=log_args,
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": LaunchConfiguration("autostart"),
                "node_names": ["controller_server", "planner_server"],
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(declare_args + [OpaqueFunction(function=_setup)])
