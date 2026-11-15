"""zed_openpose_node alone: ZED 2 + CUDA OpenPose -> /people/skeletons, markers, debug image.

    ros2 launch zed_openpose zed_openpose.launch.py [svo_path:=/path/rec.svo2] [model_pose:=COCO]
    ros2 launch zed_openpose zed_openpose.launch.py openpose:=false     # camera + /zed/points only
    ros2 launch zed_openpose zed_openpose.launch.py vio:=true           # + /zed/vio/odom (VIO)

Usually started through `ros2 launch people people_rover.launch.py`, which adds the gesture node,
robot_state_publisher and foxglove_bridge.

The node starts after the kernel's page cache is dropped (sudo -n, which the Jetson container
allows). On the Orin the GPU allocates from RAM that is actually free, not from reclaimable cache:
with most RAM held as cache, OpenPose died with CUDNN_STATUS_INTERNAL_ERROR / out of memory, and
the camera itself failed with NOT ENOUGH GPU MEMORY or CAMERA STREAM FAILED TO START, although
`free` showed GBs "available".
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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _node():
    return Node(
        package="zed_openpose",
        executable="zed_openpose_node",
        name="zed_openpose",
        output="screen",
        # Headless node: hide any X display. While its libraries load (before main), the Jetson
        # EGL stack the ZED SDK pulls in connects to $DISPLAY. In a VS Code dev-container terminal
        # that is VS Code's X11 forwarding (:1), which never answered one request, so the node
        # hung there for good: no log, no topics. Without DISPLAY it skips X and starts normally.
        prefix="env -u DISPLAY",
        exec_name="zed_openpose_node",  # launch output label, else "env" from the prefix
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
                "publish_odom": LaunchConfiguration("vio"),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"),
                                               value_type=bool),
            },
        ],
    )


def _start(context):
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
        DeclareLaunchArgument("vio", default_value="false",
                              description="Publish the ZED's visual-inertial odometry on "
                              "/zed/vio/odom (base_footprint in odom) for the EKF. Needs the "
                              "URDF's TF (robot_state_publisher)."),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="true next to the Jetson sim: stamp with sim time."),
        OpaqueFunction(function=_start),
    ])
