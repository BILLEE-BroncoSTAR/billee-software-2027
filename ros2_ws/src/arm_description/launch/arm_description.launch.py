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

    actions = [Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=NAMESPACE,
        output="screen",
        parameters=[{"robot_description": description, "use_sim_time": use_sim_time}],
    )]
    source = lc("joint_states").perform(context)
    if source in ("zero", "gui"):
        actions.append(Node(
            package="joint_state_publisher_gui" if source == "gui" else "joint_state_publisher",
            executable="joint_state_publisher_gui" if source == "gui" else "joint_state_publisher",
            namespace=NAMESPACE,
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
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
