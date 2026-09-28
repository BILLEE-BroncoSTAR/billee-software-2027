"""Pose estimation: the robot_localization EKF, plus the synthetic VIO in simulation.

    ros2 launch navigation pose_estim.launch.py                 # sim
    ros2 launch navigation pose_estim.launch.py use_sim:=false  # real rover

This node owns the odom -> base_link transform. diff_drive_controller has
enable_odom_tf: false (robot_description/config/controllers.yaml) so that exactly one
node publishes it; starting the drivetrain WITHOUT this launch leaves the TF tree with
no odom -> base_link at all.

Sensor inputs are additive. config/ekf.yaml declares wheel odometry, VIO and an IMU;
only wheel odometry has to be there. Anything not publishing is simply not fused, and
the estimate improves as each sensor comes online. See that file for the details.

`synthetic_vio` runs only when use_sim is true. It turns Gazebo's ground-truth
odometry into a noisy, drifting VIO stream so the filter sees something with the same
error characteristics as the real ZED, rather than perfect truth.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    pkg_share = get_package_share_directory("navigation")

    # .perform(context): a bare LaunchConfiguration object is always truthy, so a
    # plain `if LaunchConfiguration("use_sim"):` is true even for use_sim:=false -
    # which would start the SIMULATED VIO on the real rover and feed the filter
    # fabricated odometry.
    use_sim = LaunchConfiguration("use_sim").perform(context).lower() in ("true", "1")
    use_vio = LaunchConfiguration("use_vio").perform(context).lower() in ("true", "1")

    nodes = []

    if use_sim and use_vio:
        nodes.append(
            Node(
                package="zed2i",
                executable="synthetic_vio_node",
                name="synthetic_vio",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("zed2i"),
                        "config",
                        "synthetic_vio.yaml",
                    ),
                    {
                        "use_sim_time": True,
                        "profile": LaunchConfiguration("sim_vio_profile"),
                        "seed": LaunchConfiguration("sim_vio_seed"),
                        "truth_topic": LaunchConfiguration("sim_vio_truth_topic"),
                    },
                ],
            )
        )

    nodes.append(
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                os.path.join(pkg_share, "config", "ekf.yaml"),
                {"use_sim_time": use_sim},
            ],
        )
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim",
                default_value="true",
                description="true in Gazebo (sets use_sim_time and enables the "
                "synthetic VIO); false on the real rover.",
            ),
            DeclareLaunchArgument(
                "use_vio",
                default_value="true",
                description="Start the synthetic VIO alongside the filter in sim. "
                "false leaves the EKF running on wheel odometry alone - useful for "
                "seeing how far the estimate drifts without a pose source. No effect "
                "when use_sim:=false; on hardware the real ZED node provides VIO.",
            ),
            DeclareLaunchArgument("sim_vio_profile", default_value="validation"),
            DeclareLaunchArgument("sim_vio_seed", default_value="42"),
            DeclareLaunchArgument("sim_vio_truth_topic", default_value="/gz/odom"),
            OpaqueFunction(function=_setup),
        ]
    )
