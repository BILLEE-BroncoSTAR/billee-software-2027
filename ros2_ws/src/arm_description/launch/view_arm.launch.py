from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory("arm_description")

    urdf_path = os.path.join(pkg_share, "urdf", "arm_model.xacro")
    robot_description = xacro.process_file(urdf_path).toxml()

    viz = LaunchConfiguration('visualize')



    return LaunchDescription([
        DeclareLaunchArgument('visualize', default_value='false', description='Open RVIZ and Publisher GUI to visualize arm'),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            condition=IfCondition(viz)
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(viz)
        ),
    ])
