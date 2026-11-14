"""ZED 2i over plain USB with CPU stereo depth (no NVIDIA GPU / ZED SDK needed).

    ros2 launch zed_usb zed_usb.launch.py                    # camera + rover model TF + Foxglove :8765
    ros2 launch zed_usb zed_usb.launch.py standalone:=false  # next to a drivetrain launch
    ros2 launch zed_usb zed_usb.launch.py flip:=true         # camera mounted upside down

Publishes /zed/points (like zed_openpose_node's point cloud) and the ZED wrapper's rectified
image / depth topics under /zed/zed_node/. tooling/zed-usb runs this in the machine's ROS env.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARGS = [
    ("standalone", "true", "Also start robot_state_publisher (rover model TF) and the Foxglove "
     "bridge on :8765. false next to a drivetrain launch, which runs both."),
    ("point_cloud", "true", "Publish /zed/points."),
    ("cloud_max_hz", "10.0", "Cap on the /zed/points rate; 0 = every frame."),
    ("cloud_step", "4", "Keep every Nth pixel in /zed/points."),
    ("resolution", "VGA", "VGA, HD720, HD1080 or HD2K. USB 2 only carries VGA; CPU depth is "
     "~60 ms/frame at VGA, 4x that at HD720."),
    ("fps", "30", "Capture rate asked of the camera."),
    ("flip", "false", "true if the camera is mounted upside down."),
    ("min_depth", "0.3", "Nearest depth kept (m); also sets the disparity search range."),
    ("max_depth", "10.0", "Farthest depth kept (m)."),
    ("device", "", "/dev/videoN; empty = the first ZED found."),
    ("calib_dir", "", "Folder with SN<serial>.conf; empty = configs/zed, /usr/local/zed/settings, "
     "then ~/.local/share/zed/settings (downloads go there)."),
    ("use_sim_time", "false", "true next to the Gazebo sim."),
    ("force", "false", "Run even where the ZED SDK is installed (normally refused)."),
]


def generate_launch_description():
    standalone = LaunchConfiguration("standalone")
    cfg = {name: LaunchConfiguration(name) for name, _, _ in ARGS}
    typed = {
        "point_cloud": bool, "cloud_max_hz": float, "cloud_step": int, "fps": int, "flip": bool,
        "min_depth": float, "max_depth": float, "use_sim_time": bool, "force": bool,
        "resolution": str, "device": str, "calib_dir": str,
    }
    params = {name: ParameterValue(cfg[name], value_type=t) for name, t in typed.items()}
    robot_description = ParameterValue(
        Command(["xacro ",
                 PathJoinSubstitution([FindPackageShare("robot_description"), "urdf", "robot.urdf.xacro"]),
                 " use_sim:=false can_interface:=mock"]),
        value_type=str)

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc) for name, default, desc in ARGS]
        + [
            Node(
                package="zed_usb",
                executable="zed_usb_node",
                name="zed_usb",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description,
                             "use_sim_time": params["use_sim_time"]}],
                condition=IfCondition(standalone),
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[{
                    "port": 8765,
                    "address": "0.0.0.0",
                    "topic_whitelist": [".*"],
                    "send_buffer_limit": 10000000,
                }],
                condition=IfCondition(standalone),
            ),
        ]
    )
