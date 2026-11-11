"""Standalone bring-up of PxxSystemHardware for testing without the arm description.

    # mock backend (no CAN at all):
    ros2 launch pxx pxx_test.launch.py can_interface:=mock
    # real SocketCAN backend against the CANopen drive emulator:
    tooling/can-up vcan0
    ros2 launch pxx pxx_test.launch.py can_interface:=vcan0

Driver-only bench test with two placeholder joints. For the real arm (all axes,
RoboClaw included, constants from arm_bringup/config/arm_axes.yaml) use
`ros2 launch arm_bringup arm.launch.py`. Command the joints with:
    ros2 topic pub --once /arm_position_controller/commands \
        std_msgs/msg/Float64MultiArray "{data: [0.5, -0.25]}"
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (joint, CANopen node ID, gear_ratio = motor-shaft turns per joint turn)
JOINTS = [("joint_arm_1", 1, 121.0), ("joint_arm_2", 2, 121.0)]


def robot_description(can_interface):
    joints = "".join(
        f"""
    <joint name="{name}">
      <param name="node_id">{node_id}</param>
      <param name="gear_ratio">{gear_ratio}</param>
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>"""
        for name, node_id, gear_ratio in JOINTS
    )
    return f"""<?xml version="1.0"?>
<robot name="pxx_test">
  <link name="base_link"/>
  <ros2_control name="pxx_arm" type="system">
    <hardware>
      <plugin>pxx/PxxSystemHardware</plugin>
      <param name="can_interface">{can_interface}</param>
    </hardware>{joints}
  </ros2_control>
</robot>
"""


def launch_setup(context):
    can_interface = LaunchConfiguration("can_interface").perform(context)
    controllers = os.path.join(
        get_package_share_directory("pxx"), "config", "pxx_test_controllers.yaml")

    actions = []
    if can_interface.startswith("vcan"):
        actions.append(ExecuteProcess(
            cmd=[
                os.path.join(get_package_prefix("pxx"), "lib", "pxx", "pxx_vcan_emulator.py"),
                "--interface", can_interface,
                "--nodes", *[str(node_id) for _, node_id, _ in JOINTS],
            ],
            output="screen",
        ))
    actions += [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description(can_interface)}, controllers],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "arm_position_controller"],
            output="screen",
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "can_interface", default_value="mock",
            description="'mock' = loopback backend (no CAN); 'vcanN' = SocketCAN backend "
            "plus the CANopen drive emulator; 'canN' = real bus.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
