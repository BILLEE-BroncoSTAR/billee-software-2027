#!/usr/bin/env python3
"""Move ONE arm joint slowly by a small amount, then (optionally) back.

Speed and limits come from config/arm_axes.yaml (test_velocity, limits.*), so the
default motion is the slow bring-up speed. Requires arm.launch.py to be running.

    ros2 run arm_bringup arm_slow_jog.py joint_arm_1 0.1            # +0.1 rad at test_velocity
    ros2 run arm_bringup arm_slow_jog.py joint_arm_1 -0.1 --back    # out and back
    ros2 run arm_bringup arm_slow_jog.py joint_arm_3 0.2 --speed 0.02

pxx axes: sends a FollowJointTrajectory goal to arm_can_controller timed so the
          joint moves at a constant speed = |delta| / duration.
roboclaw: streams a constant velocity to arm_roboclaw_controller for
          |delta| / speed seconds, then zero (open loop on duration; it watches
          the position state and stops early if it would pass a limit).
Ctrl-C cancels the goal / commands zero velocity.
"""

import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint


def load_axes(path):
    """Return ({joint: axis}, [roboclaw joints in controller order])."""
    with open(path) as f:
        axes = yaml.safe_load(f)["axes"]
    # arm.launch.py fills arm_roboclaw_controller's joints in file order.
    return {a["joint"]: a for a in axes}, [a["joint"] for a in axes if a["driver"] == "roboclaw"]


class Jog(Node):
    def __init__(self):
        super().__init__("arm_slow_jog")
        self.positions = {}
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)

    def _on_state(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def wait_position(self, joint, timeout=5.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and joint not in self.positions and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if joint not in self.positions:
            raise RuntimeError(f"no /joint_states for '{joint}' (is arm.launch.py running?)")
        return self.positions[joint]

    def move_pxx(self, joint, start, target, speed):
        client = ActionClient(self, FollowJointTrajectory,
                              "/arm_can_controller/follow_joint_trajectory")
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("arm_can_controller action server not available")
        duration = max(abs(target - start) / speed, 0.5)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [joint]
        # One position-only point: JTC interpolates linearly from the current state,
        # i.e. a constant-speed move of |delta| / duration.
        point = JointTrajectoryPoint(positions=[target])
        point.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points.append(point)
        self.get_logger().info(
            f"{joint}: {start:+.4f} -> {target:+.4f} rad over {duration:.1f} s "
            f"({speed:.3f} rad/s)")
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError("trajectory goal rejected")
        result = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result)
        except KeyboardInterrupt:
            self.get_logger().warn("cancelling goal")
            rclpy.spin_until_future_complete(self, handle.cancel_goal_async())
            raise
        code = result.result().result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f"trajectory failed, error_code {code}: "
                               f"{result.result().result.error_string}")

    def move_roboclaw(self, joint, rc_joints, start, target, speed, limits):
        pub = self.create_publisher(Float64MultiArray, "/arm_roboclaw_controller/commands", 10)
        index = rc_joints.index(joint)
        velocity = math.copysign(speed, target - start)
        duration = abs(target - start) / speed

        def send(v):
            cmd = [0.0] * len(rc_joints)
            cmd[index] = v
            pub.publish(Float64MultiArray(data=cmd))

        self.get_logger().info(
            f"{joint}: {velocity:+.3f} rad/s for {duration:.1f} s "
            f"(from {start:+.4f}, aiming for {target:+.4f} rad)")
        end = time.monotonic() + duration
        try:
            while rclpy.ok() and time.monotonic() < end:
                pos = self.positions.get(joint, start)
                if not limits[0] <= pos <= limits[1]:
                    self.get_logger().error(f"{joint} at {pos:+.4f} left its limits, stopping")
                    break
                send(velocity)
                rclpy.spin_once(self, timeout_sec=0.05)
        finally:
            for _ in range(5):  # make sure the zero gets through
                send(0.0)
                rclpy.spin_once(self, timeout_sec=0.02)
        self.get_logger().info(f"{joint} stopped at {self.positions.get(joint, float('nan')):+.4f}")


def main():
    default_axes = os.path.join(
        get_package_share_directory("arm_bringup"), "config", "arm_axes.yaml")
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("joint")
    parser.add_argument("delta", type=float, help="relative move in rad (at the joint)")
    parser.add_argument("--speed", type=float, default=None,
                        help="rad/s (default: the axis's test_velocity; capped at "
                             "limits.max_velocity)")
    parser.add_argument("--back", action="store_true", help="return to the start afterwards")
    parser.add_argument("--axes-file", default=default_axes)
    args, ros_args = parser.parse_known_args()

    axes, rc_joints = load_axes(args.axes_file)
    if args.joint not in axes:
        sys.exit(f"unknown joint '{args.joint}'; known: {', '.join(axes)}")
    axis = axes[args.joint]
    limits = (axis["limits"]["min_position"], axis["limits"]["max_position"])
    speed = args.speed if args.speed is not None else axis["test_velocity"]
    if not 0 < speed <= axis["limits"]["max_velocity"]:
        sys.exit(f"speed {speed} must be > 0 and <= limits.max_velocity "
                 f"({axis['limits']['max_velocity']})")

    rclpy.init(args=ros_args)
    node = Jog()
    try:
        start = node.wait_position(args.joint)
        target = start + args.delta
        if not limits[0] <= target <= limits[1]:
            sys.exit(f"target {target:+.4f} outside limits [{limits[0]}, {limits[1]}] "
                     "from arm_axes.yaml")
        moves = [(start, target)] + ([(target, start)] if args.back else [])
        for a, b in moves:
            if axis["driver"] == "pxx":
                node.move_pxx(args.joint, a, b, speed)
            else:
                node.move_roboclaw(args.joint, rc_joints, a, b, speed, limits)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        node.get_logger().error(str(e))
        sys.exit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
