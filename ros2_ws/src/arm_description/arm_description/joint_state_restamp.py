"""Republish the arm's joint states stamped with this node's clock (mode:=sim).

    ros2 run arm_description joint_state_restamp --ros-args -r __ns:=/arm -p use_sim_time:=true

In mode:=sim the rover's TF runs on Gazebo's clock (/clock), and so do the viewers (Foxglove
through the bridge, RViz with use_sim_time). The arm driver (arm_bringup's arm.launch.py, real or
mock) stays on the wall clock, because it drives real time-based hardware, so its
/arm/joint_states carry wall-clock stamps. robot_state_publisher stamps the arm's moving joints
with those, and a viewer can't join them to odom: odom -> base_link is at ~100 s, the arm at
~1.8e9 s, so the arm, the RealSense on it and every ArUco marker vanish.

This node subscribes to joint_states and publishes the same message on joint_states/restamped
with header.stamp = now() on its own clock (sim time with use_sim_time). The arm's
robot_state_publisher reads that in mode:=sim (arm_description.launch.py, joint_states:=arm with
use_sim_time:=true), so the arm's TF lands on the sim clock. The joint angles are unchanged; the
stamp is when they arrived, which is within one driver cycle of when they were measured.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRestamp(Node):

    def __init__(self):
        super().__init__("joint_state_restamp")
        self.pub = self.create_publisher(JointState, "joint_states/restamped", 10)
        self.create_subscription(JointState, "joint_states", self.on_joint_states, 10)
        self._waiting_logged = False

    def on_joint_states(self, msg: JointState):
        now = self.get_clock().now()
        if now.nanoseconds == 0:  # use_sim_time and no /clock yet: a 0 stamp would poison TF
            if not self._waiting_logged:
                self.get_logger().info("joint states arriving, waiting for /clock")
                self._waiting_logged = True
            return
        msg.header.stamp = now.to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(JointStateRestamp())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
