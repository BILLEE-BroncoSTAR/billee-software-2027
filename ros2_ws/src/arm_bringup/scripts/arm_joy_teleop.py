#!/usr/bin/env python3
"""Bare-minimum joystick teleop for the arm: jog ONE joint at a time.

Joint list, speeds and limits come from config/arm_axes.yaml, so re-wiring or
renaming axes is a YAML edit only. Requires arm.launch.py and a joy_node
(arm_teleop.launch.py starts both teleop and joy_node).

Controls (defaults, raw-joystick Xbox layout, same as teleop/config/joystick.yaml):
    hold LB (button 4)      deadman: nothing moves unless it is held
    B (button 1)            select next joint
    X (button 2)            select previous joint
    left stick vertical     jog the selected joint (full stick = test_velocity)

The drive teleop uses RB as its deadman, so arm and drive never share a button.

Safety behaviour:
  - Releasing the deadman, losing /joy for joy_timeout seconds or changing the
    selected joint stops the arm.
  - Commands never leave limits.min_position / max_position from arm_axes.yaml.
  - pxx axes (arm_can_controller, position): a reference position is integrated
    from the stick and streamed as short trajectories; it is re-synced to the
    measured position whenever a jog starts and is not allowed to run more than
    max_lag ahead of the measured position.
  - roboclaw axes (arm_roboclaw_controller, velocity): the velocity is streamed
    while jogging and zero is sent on every stop. Note that a
    JointGroupVelocityController holds its last command, which is why the /joy
    watchdog exists.

Parameters (ros2 run ... --ros-args -p name:=value): axes_file, deadman_button,
next_button, prev_button, jog_axis, invert_axis, speed_scale (fraction of
test_velocity at full stick, capped at limits.max_velocity), deadzone,
joy_timeout, rate, max_lag.
"""

import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration as DurationMsg
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

LOOKAHEAD = 0.1  # s: each streamed pxx trajectory point is this far ahead


class ArmJoyTeleop(Node):
    def __init__(self):
        super().__init__("arm_joy_teleop")
        default_axes = os.path.join(
            get_package_share_directory("arm_bringup"), "config", "arm_axes.yaml")
        p = self.declare_parameter
        axes_file = p("axes_file", default_axes).value
        self.deadman = p("deadman_button", 4).value
        self.next_btn = p("next_button", 1).value
        self.prev_btn = p("prev_button", 2).value
        self.jog_axis = p("jog_axis", 1).value
        self.invert = p("invert_axis", False).value
        self.speed_scale = p("speed_scale", 1.0).value
        self.deadzone = p("deadzone", 0.1).value
        self.joy_timeout = p("joy_timeout", 0.3).value
        self.max_lag = p("max_lag", 0.1).value
        rate = p("rate", 50.0).value

        with open(axes_file) as f:
            self.axes = yaml.safe_load(f)["axes"]
        if not self.axes:
            raise RuntimeError(f"{axes_file}: no axes")
        # arm.launch.py fills arm_roboclaw_controller's joints in file order.
        self.rc_joints = [a["joint"] for a in self.axes if a["driver"] == "roboclaw"]

        self.selected = 0
        self.positions = {}
        self.joy = None
        self.joy_time = None
        self.prev_buttons = []
        self.active = False      # was jogging on the previous tick
        self.reference = None    # pxx reference position while jogging
        self.dt = 1.0 / rate

        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory, "/arm_can_controller/joint_trajectory", 10)
        self.vel_pub = self.create_publisher(
            Float64MultiArray, "/arm_roboclaw_controller/commands", 10)
        self.create_timer(self.dt, self._tick)

        for a in self.axes:
            if not a.get("verified", False):
                self.get_logger().warn(f"axis '{a['joint']}' is not verified in arm_axes.yaml")
        self.get_logger().info(
            f"hold button {self.deadman} to jog; buttons {self.next_btn}/{self.prev_btn} "
            f"select joint; axis {self.jog_axis} moves it. Joints: "
            f"{[a['joint'] for a in self.axes]}")
        self._announce()

    # ------------------------------------------------------------------ input
    def _on_state(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def _on_joy(self, msg):
        def pressed(i):  # rising edge only
            return (0 <= i < len(msg.buttons) and msg.buttons[i]
                    and not (i < len(self.prev_buttons) and self.prev_buttons[i]))

        step = int(pressed(self.next_btn)) - int(pressed(self.prev_btn))
        if step:
            self._stop()
            self.selected = (self.selected + step) % len(self.axes)
            self._announce()
        self.prev_buttons = list(msg.buttons)
        self.joy = msg
        self.joy_time = self.get_clock().now()

    def _announce(self):
        a = self.axes[self.selected]
        lim = a["limits"]
        pos = self.positions.get(a["joint"])
        pos = f"{pos:+.3f}" if pos is not None else "?"
        self.get_logger().info(
            f"selected [{self.selected + 1}/{len(self.axes)}] {a['joint']} ({a['driver']}) "
            f"pos {pos} rad, limits [{lim['min_position']}, {lim['max_position']}]")

    def _command(self):
        """Stick value in [-1, 1] if jogging is allowed right now, else None."""
        if self.joy is None or self.joy_time is None:
            return None
        age = (self.get_clock().now() - self.joy_time).nanoseconds * 1e-9
        if age > self.joy_timeout:
            return None
        b, ax = self.joy.buttons, self.joy.axes
        if not (0 <= self.deadman < len(b) and b[self.deadman]):
            return None
        if not 0 <= self.jog_axis < len(ax):
            return None
        value = -ax[self.jog_axis] if self.invert else ax[self.jog_axis]
        return 0.0 if abs(value) < self.deadzone else max(-1.0, min(1.0, value))

    # ----------------------------------------------------------------- output
    def _send_pxx(self, joint, position):
        point = JointTrajectoryPoint(positions=[position])
        point.time_from_start = DurationMsg(sec=0, nanosec=int(LOOKAHEAD * 1e9))
        self.traj_pub.publish(JointTrajectory(joint_names=[joint], points=[point]))

    def _send_rc(self, joint, velocity):
        cmd = [0.0] * len(self.rc_joints)
        if joint is not None:
            cmd[self.rc_joints.index(joint)] = velocity
        self.vel_pub.publish(Float64MultiArray(data=cmd))

    def _stop(self):
        if not self.active:
            return
        a = self.axes[self.selected]
        if a["driver"] == "pxx":
            if self.reference is not None:
                self._send_pxx(a["joint"], self.reference)  # hold where we are
        else:
            for _ in range(3):  # make sure the zero gets through
                self._send_rc(None, 0.0)
        self.active = False
        self.reference = None
        self.get_logger().info(f"{a['joint']} stopped at "
                               f"{self.positions.get(a['joint'], float('nan')):+.3f} rad")

    def _tick(self):
        stick = self._command()
        a = self.axes[self.selected]
        joint = a["joint"]
        pos = self.positions.get(joint)
        if stick is None or pos is None:
            if stick is not None and pos is None:
                self.get_logger().warn(f"no /joint_states for {joint}",
                                       throttle_duration_sec=2.0)
            self._stop()
            return

        lim = a["limits"]
        lo, hi = lim["min_position"], lim["max_position"]
        speed = min(a["test_velocity"] * self.speed_scale, lim["max_velocity"])
        velocity = stick * speed

        if not self.active:
            if velocity == 0.0:
                return
            self.active = True
            self.reference = pos
            self.get_logger().info(f"jogging {joint} from {pos:+.3f} rad")

        if a["driver"] == "pxx":
            ref = self.reference + velocity * self.dt
            # Don't let the reference run away from a drive that is lagging or stuck.
            ref = max(pos - self.max_lag, min(pos + self.max_lag, ref))
            self.reference = max(lo, min(hi, ref))
            target = max(lo, min(hi, self.reference + velocity * LOOKAHEAD))
            self._send_pxx(joint, target)
        else:
            if (velocity > 0 and pos >= hi) or (velocity < 0 and pos <= lo):
                self.get_logger().warn(f"{joint} at limit ({pos:+.3f} rad)",
                                       throttle_duration_sec=1.0)
                velocity = 0.0
            self._send_rc(joint, velocity)


def main():
    rclpy.init()
    node = ArmJoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
