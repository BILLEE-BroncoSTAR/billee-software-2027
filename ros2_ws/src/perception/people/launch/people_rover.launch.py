"""People on the rover (Jetson, ZED 2 plugged in): zed-openpose (CUDA) -> skeletons -> gestures.

    # perception only (no drivetrain): also starts robot_state_publisher + foxglove_bridge
    ros2 launch people people_rover.launch.py

    # next to a running drivetrain (real.launch.py foxglove:=true already has both)
    ros2 launch people people_rover.launch.py standalone:=false

Skeletons are placed in base_link, i.e. relative to the rover, using the URDF's
base_link -> camera_link mount. Ground station: Foxglove -> ws://<rover>:8765 with
people/foxglove/people_rover.json, or RViz on /people/markers.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    standalone = LaunchConfiguration("standalone")
    robot_description = ParameterValue(
        Command(["xacro ",
                 PathJoinSubstitution([FindPackageShare("robot_description"),
                                       "urdf", "robot.urdf.xacro"]),
                 " use_sim:=false can_interface:=mock"]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("standalone", default_value="true",
                              description="Also start robot_state_publisher and foxglove_bridge "
                              "(false when real.launch.py is already running them)."),
        DeclareLaunchArgument("target_frame", default_value="base_link",
                              description="Frame the 3D skeletons are published in."),
        DeclareLaunchArgument("model_pose", default_value="BODY_25",
                              description="OpenPose model: BODY_25, COCO or MPI."),
        DeclareLaunchArgument("svo_path", default_value="",
                              description="Replay a ZED recording instead of the live camera."),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("zed_openpose"), "launch",
                                      "zed_openpose.launch.py"])),
            launch_arguments={
                "target_frame": LaunchConfiguration("target_frame"),
                "model_pose": LaunchConfiguration("model_pose"),
                "svo_path": LaunchConfiguration("svo_path"),
            }.items(),
        ),
        Node(
            package="people",
            executable="gesture_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([FindPackageShare("people"), "config", "gestures.yaml"]),
            ],
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
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
    ])
