"""Simulation localization: synthetic VIO, local + global EKF, navsat and the GPS covariance relay.

Nav2 (costmaps, planner, controller, navigator) is navigation/launch/nav2.launch.py.
"""

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
    # local EKF filter for odometry node
    launch_nodes.append( 
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config/ekf_sim.yaml'),
                {'use_sim_time': use_sim},
            ],
        )
    )
    # global EKF filter for map node
    launch_nodes.append(
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config/ekf_sim.yaml'),
                {'use_sim_time': use_sim},
            ],
            remappings=[('odometry/filtered', '/odometry/global')],

        )
    )
    # Fills covariance on GPS fixes that arrive without one, republishing on /gps/with_cov.
    # Sim: the Gazebo bridge always publishes zero covariance. Real: comms/GpsNode publishes
    # zero covariance (type UNKNOWN) when no covariance message has arrived yet. The EKF
    # treats zero as exact. Fixes that already carry covariance pass through unchanged.
    launch_nodes.append(
        Node(
            package='navigation',
            executable='gps_covariance_relay',
            name='gps_covariance_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim}],
            remappings=[('gps', '/gps'), ('gps/with_cov', '/gps/with_cov')],
        )
    )

    # Navsat transform node
    launch_nodes.append(
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config/ekf_sim.yaml'),
                {'use_sim_time': use_sim},
            ],
            # No imu remap: there is no IMU, and use_odometry_yaw takes heading from
            # /odometry/global instead.
            remappings=[
                ('gps/fix', '/gps/with_cov'),
                ('odometry/filtered', '/odometry/global'),
                ('odometry/gps', '/odometry/gps'),
                ('gps/filtered', '/gps/filtered'),
            ],
        )
    )

    return LaunchDescription([
        *launch_args,
        *launch_nodes
    ])
