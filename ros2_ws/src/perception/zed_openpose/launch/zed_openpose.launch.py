"""zed_openpose_node alone: ZED 2 + CUDA OpenPose -> /people/skeletons, markers, debug image.

    ros2 launch zed_openpose zed_openpose.launch.py [svo_path:=/path/rec.svo2] [model_pose:=COCO]
    ros2 launch zed_openpose zed_openpose.launch.py openpose:=false     # camera + /zed/points only

Usually started through `ros2 launch people people_rover.launch.py`, which adds the gesture node,
robot_state_publisher and foxglove_bridge.

With openpose:=true the node starts after the kernel's page cache is dropped (sudo -n, which the
Jetson container allows). On the Orin the GPU allocates from RAM that is actually free, not from
reclaimable cache: with most RAM held as cache (after a build or a download), OpenPose died at
startup with CUDNN_STATUS_INTERNAL_ERROR / out of memory although `free` showed GBs "available".
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _node():
    return Node(
        package="zed_openpose",
        executable="zed_openpose_node",
        name="zed_openpose",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("zed_openpose"), "config", "zed_openpose.yaml"]),
            {
                "model_folder": LaunchConfiguration("model_folder"),
                "model_pose": LaunchConfiguration("model_pose"),
                "svo_path": LaunchConfiguration("svo_path"),
                "target_frame": LaunchConfiguration("target_frame"),
                "enable_pose": LaunchConfiguration("openpose"),
                "publish_cloud": LaunchConfiguration("point_cloud"),
            },
        ],
    )


def _start(context):
    if LaunchConfiguration("openpose").perform(context).lower() not in ("true", "1"):
        return [_node()]
    drop = ExecuteProcess(
        # env -u LD_PRELOAD: under vglrun (the Jetson sim), sudo can't load VirtualGL's preloads
        cmd=["env", "-u", "LD_PRELOAD", "sudo", "-n", "sh", "-c",
             "sync && echo 3 > /proc/sys/vm/drop_caches"],
        name="drop_page_cache", output="screen")
    return [drop, RegisterEventHandler(OnProcessExit(target_action=drop, on_exit=[_node()]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_folder",
            # The trailing "" keeps the "/" OpenPose expects at the end of the folder.
            default_value=PathJoinSubstitution(
                [EnvironmentVariable("PIXI_PROJECT_ROOT", default_value="."), "models", "openpose", ""]),
            description="OpenPose models (pose/body_25, pose/coco); see "
            "people/scripts/get_openpose_models.sh.",
        ),
        DeclareLaunchArgument("model_pose", default_value="BODY_25"),
        DeclareLaunchArgument("svo_path", default_value="",
                              description="Replay a ZED recording instead of the live camera."),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("openpose", default_value="true",
                              description="Run OpenPose (/people/*). false = camera only."),
        DeclareLaunchArgument("point_cloud", default_value="true",
                              description="Publish /zed/points (base_link via TF)."),
        OpaqueFunction(function=_start),
    ])
