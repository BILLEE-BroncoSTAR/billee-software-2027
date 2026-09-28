"""Launch the simulation pose estimator and local obstacle costmap."""

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
        DeclareLaunchArgument("use_sim", default_value="true"),
        DeclareLaunchArgument("sim_vio_profile", default_value="validation"),
        DeclareLaunchArgument("sim_vio_seed", default_value="42"),
        DeclareLaunchArgument("sim_vio_truth_topic", default_value="/gz/odom"),
    ]

    launch_nodes = []
    use_sim = LaunchConfiguration("use_sim")

    params_file = PathJoinSubstitution(
        [FindPackageShare("zed2i"), "config", "synthetic_vio.yaml"]
    )
    costmap_params_file = PathJoinSubstitution(
        [FindPackageShare("navigation"), "config", "costmap.yaml"]
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
                    "use_sim_time": use_sim,
                    "profile": LaunchConfiguration("sim_vio_profile"),
                    "seed": LaunchConfiguration("sim_vio_seed"),
                    "truth_topic": LaunchConfiguration("sim_vio_truth_topic"),
                },
            ],
        ),
    )

    launch_nodes.append( 
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config/ekf_sim.yaml'),
                {'use_sim_time': use_sim},
            ],
        )
    )

    # ControllerServer owns the local_costmap.
    launch_nodes.extend([
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[costmap_params_file, {'use_sim_time': use_sim}],
            # The diff-drive controller accepts geometry_msgs/Twist on this
            # topic. Nav2 uses TwistStamped by default on recent ROS releases.
            remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="controller_lifecycle_manager",
            output="screen",
            parameters=[
                {
                    'use_sim_time': use_sim,
                    'autostart': True,
                    'node_names': ['controller_server'],
                }
            ],
        ),
    ])


    return LaunchDescription([
        *launch_args,       
        *launch_nodes
    ])
