"""status_light: drives the blink(1) USB mode light from the rover state machine's state.

    ros2 run status_light status_light --ros-args -p pattern:=teleop
    ros2 launch chassis_bringup rover.launch.py mode:=real status_light:=teleop   # the usual way

Parameters
    pattern       idle | teleop | autonomous | arrived | off   (default idle; aliases in patterns.py)
    serial        blink(1) serial number, for when several are plugged in (default: the first)
    off_on_exit   turn the light off when the node stops (default true)

Topics
    /status_light/pattern  (std_msgs/String, in)   switch pattern at run time, e.g. the autonomous
                                                    mode sending "arrived" when it reaches its goal
    /status_light/state    (std_msgs/String, out, latched)   the pattern now showing

Without a blink(1) plugged in (or without permission to open it) the node keeps running, logs a
warning every 30 s and retries every 3 s, so a missing light never stops a launch.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from status_light.patterns import Player

RETRY_S = 3.0
WARN_EVERY_S = 30.0
# What the blink1 driver raises: Blink1ConnectionFailed is a RuntimeError, USB/HID errors are
# OSError (IOError), a bad colour is a ValueError; ImportError when the driver isn't installed.
DEVICE_ERRORS = (ImportError, RuntimeError, OSError, ValueError)


def open_blink1(serial):
    # Imported here so the node (and its tests) load without the driver installed.
    from blink1.blink1 import Blink1
    return Blink1(serial_number=serial or None)


class StatusLight(Node):
    def __init__(self, open_device=open_blink1):
        super().__init__('status_light')
        self.open_device = open_device
        pattern = self.declare_parameter('pattern', 'idle').value
        self.serial = self.declare_parameter('serial', '').value
        self.off_on_exit = self.declare_parameter('off_on_exit', True).value
        try:
            self.player = Player(pattern)
        except ValueError as e:
            self.get_logger().error(f'{e}; showing idle')
            self.player = Player('idle')
        self.dev = None
        self.next_open = 0.0
        self.last_warn = -WARN_EVERY_S
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/status_light/state', latched)
        self.create_subscription(String, '/status_light/pattern', self.on_pattern, 10)
        self.create_timer(0.05, self.tick)
        self.publish_state()
        self.get_logger().info(f'mode light: {self.player.pattern}')

    def publish_state(self):
        self.state_pub.publish(String(data=self.player.pattern))

    def on_pattern(self, msg):
        try:
            self.player.set(msg.data)
        except ValueError as e:
            self.get_logger().warn(str(e))
            return
        self.get_logger().info(f'mode light: {self.player.pattern}')
        self.publish_state()

    def warn(self, text):
        now = time.monotonic()
        if now - self.last_warn >= WARN_EVERY_S:
            self.last_warn = now
            self.get_logger().warn(text)

    def device(self):
        if self.dev is None and time.monotonic() >= self.next_open:
            try:
                self.dev = self.open_device(self.serial)
                self.get_logger().info('blink(1) connected')
                self.player.resend()
            except DEVICE_ERRORS as e:
                self.next_open = time.monotonic() + RETRY_S
                hint = ('pixi install (PyPI blink1)' if isinstance(e, ImportError) else
                        'plugged in? udev rule? (docker/blink1_udev.sh)')
                self.warn(f'no blink(1): {e} [{hint}]; retrying every {RETRY_S:.0f} s')
        return self.dev

    def tick(self):
        dev = self.device()
        if dev is None:
            return
        out = self.player.step(time.monotonic())
        if out is None:
            return
        rgb, fade_ms = out
        try:
            dev.fade_to_color(fade_ms, rgb)
        except DEVICE_ERRORS as e:  # unplugged: reopen on a later tick
            self.warn(f'blink(1) write failed: {e}; reconnecting')
            self.close()
            self.next_open = time.monotonic() + RETRY_S

    def close(self, off=False):
        if self.dev is None:
            return
        try:
            if off:
                self.dev.fade_to_color(200, (0, 0, 0))
            self.dev.close()
        except DEVICE_ERRORS as e:  # already unplugged: nothing left to close
            self.get_logger().debug(f'blink(1) close: {e}')
        self.dev = None


def main(args=None):
    rclpy.init(args=args)
    node = StatusLight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close(off=node.off_on_exit)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
