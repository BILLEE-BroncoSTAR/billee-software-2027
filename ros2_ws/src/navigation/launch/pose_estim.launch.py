"""Launches the Robot Localization Node"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('navigation')

    launch_args = [
        DeclareLaunchArgument("use_sim", default_value="True"),
        DeclareLaunchArgument("sim_vio_profile", default_value="validation"),
        DeclareLaunchArgument("sim_vio_seed", default_value="42"),
        DeclareLaunchArgument("sim_vio_truth_topic", default_value="/gz/odom"),
    ]

    launch_nodes = []
    use_sim = LaunchConfiguration("use_sim")

    if use_sim:
        params_file = PathJoinSubstitution(
            [FindPackageShare("zed2i"), "config", "synthetic_vio.yaml"]
        )
        launch_nodes.append(
            Node(
                package="zed2i",
                executable="synthetic_vio_node",
                name="synthetic_vio",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": True,
                        "profile": LaunchConfiguration("sim_vio_profile"),
                        "seed": LaunchConfiguration("sim_vio_seed"),
                        "truth_topic": LaunchConfiguration("sim_vio_truth_topic"),
                    },
                ],
            ),
        )

    kf_config = 'config/ekf_sim.yaml' if use_sim else 'config/ekf.yaml'

    launch_nodes.append( 
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_share, kf_config), {'use_sim_time': LaunchConfiguration('use_sim')}]
        )
    )


    return LaunchDescription([
        *launch_args,       
        *launch_nodes
    ])
