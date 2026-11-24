"""The arm's TF: robot_state_publisher for urdf/arm_standalone.urdf.xacro, in the /arm namespace.

    ros2 launch arm_description arm_description.launch.py                      # with arm.launch.py
    ros2 launch arm_description arm_description.launch.py joint_states:=zero   # bench, no arm
    ros2 launch arm_description arm_description.launch.py joint_states:=gui rviz:=true

It publishes base_link -> arm_base_link -> ... -> arm_tool_link -> arm_camera_mount ->
arm_cam_link from the joint angles on /arm/joint_states, so the RealSense (and every ArUco marker
and the keyboard it sees) is placed relative to the rover. The rover's own robot_state_publisher
(chassis_bringup) owns base_link's other children; the two trees meet at base_link.

joint_states:=
  arm   (default) the real joint angles, from arm_bringup's arm.launch.py (joint_state_broadcaster
        on /arm/joint_states). Until it runs, TF stops at the arm's base: the moving links have
        no transform, and the keyboard can't be placed relative to the rover.
  zero  a joint_state_publisher holding every joint at 0 rad: TF works without the arm powered
        (bench tests of the camera). Do NOT use it with arm.launch.py running (two publishers).
  gui   joint_state_publisher_gui: sliders, to check the URDF and the camera mount by eye.

use_sim_time:=true (rover.launch.py mode:=sim): the arm's TF is stamped on the sim clock, like
the rover's, so viewers can place the arm in odom. With joint_states:=arm the driver's
/arm/joint_states (wall clock) go through joint_state_restamp to /arm/joint_states/restamped,
which robot_state_publisher reads.

Mount offsets (placeholders until the new arm is measured; see the xacro files):
  mount_xyz / mount_rpy    arm base on the rover, in base_link
  camera_xyz / camera_rpy  D435i bottom screw on the end effector, in arm_tool_link
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

NAMESPACE = "arm"  # same as arm_bringup: /arm/joint_states, /arm/robot_description
# xacro arg -> launch argument. "camera" is "with_camera" here: an including launch's arguments
# reach this one (rover.launch.py's camera:= is the ZED mode, "cloud" etc.).
XACRO_ARGS = {"parent": "parent", "mount_xyz": "mount_xyz", "mount_rpy": "mount_rpy",
              "camera": "with_camera", "camera_xyz": "camera_xyz", "camera_rpy": "camera_rpy"}
JOINT_STATES = ("arm", "zero", "gui")


def _setup(context):
    lc = LaunchConfiguration
    xacro_cmd = ["xacro ", PathJoinSubstitution(
        [FindPackageShare("arm_description"), "urdf", "arm_standalone.urdf.xacro"])]
    for xacro_arg, launch_arg in XACRO_ARGS.items():
        xacro_cmd += [f" {xacro_arg}:=\"", lc(launch_arg), "\""]
    description = ParameterValue(Command(xacro_cmd), value_type=str)
    use_sim_time = lc("use_sim_time").perform(context) == "true"
    source = lc("joint_states").perform(context)
    # On sim time with the arm driver's joint states (wall clock, see joint_state_restamp.py),
    # read a copy restamped on the sim clock, so the arm's TF joins the rover's. zero / gui
    # publish on this launch's clock already.
    restamp = use_sim_time and source == "arm"

    actions = [Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=NAMESPACE,
        output="screen",
        # publish_frequency caps the moving joints' TF, counted in message time: the default
        # 20 Hz is 50 ms of sim time, ~360 ms of wall time with Gazebo at 0.14x real time on the
        # Jetson, and keyboard_node's lookups (and the arm in Foxglove) waited on it. 1000 Hz
        # lets every joint state through: the driver's rate (100 Hz) sets it.
        parameters=[{"robot_description": description, "use_sim_time": use_sim_time,
                     "publish_frequency": 1000.0}],
        remappings=[("joint_states", "joint_states/restamped")] if restamp else [],
    )]
    if restamp:
        actions.append(Node(
            package="arm_description",
            executable="joint_state_restamp",
            namespace=NAMESPACE,
            output="screen",
            parameters=[{"use_sim_time": True}],
        ))
    if source in ("zero", "gui"):
        actions.append(Node(
            package="joint_state_publisher_gui" if source == "gui" else "joint_state_publisher",
            executable="joint_state_publisher_gui" if source == "gui" else "joint_state_publisher",
            namespace=NAMESPACE,
            output="screen",
            # 100 Hz, not the default 10: on sim time that's 1.4 Hz of wall time at the Jetson's
            # real-time factor, too slow for TF lookups at the camera's stamps.
            parameters=[{"use_sim_time": use_sim_time, "rate": 100}],
        ))
    if lc("rviz").perform(context) == "true":
        actions.append(Node(
            package="rviz2", executable="rviz2", output="screen",
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare("arm_description"), "rviz", "arm.rviz"])],
            parameters=[{"use_sim_time": use_sim_time}],
        ))
    return actions


def generate_launch_description():
    defaults = {
        "parent": ("base_link", "Rover link the arm's base is fixed to."),
        "mount_xyz": ("0.25 0 0.329", "Arm base in the parent link (m). PLACEHOLDER."),
        "mount_rpy": ("0 0 0", "Arm base orientation in the parent link (rad). PLACEHOLDER."),
        "with_camera": ("true", "Add the D435i (arm_camera_mount -> arm_cam_link)."),
        "camera_xyz": ("-0.09 0 0.045", "D435i bottom screw in arm_tool_link (m). PLACEHOLDER."),
        "camera_rpy": ("0 0 3.14159265",
                       "D435i orientation in arm_tool_link (rad), x out of the lens. PLACEHOLDER."),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v, description=d)
          for k, (v, d) in defaults.items()],
        DeclareLaunchArgument("joint_states", default_value="arm", choices=list(JOINT_STATES),
                              description="arm = /arm/joint_states from arm.launch.py; zero = "
                              "hold every joint at 0; gui = sliders."),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              choices=["true", "false"]),
        OpaqueFunction(function=_setup),
    ])
