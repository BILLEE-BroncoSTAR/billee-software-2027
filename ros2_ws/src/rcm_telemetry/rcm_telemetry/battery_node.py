"""The four RCM battery packs as ROS topics, from the RCM's F´ GDS (see battery.py).

    ros2 run rcm_telemetry battery_node --ros-args -p gds_url:=http://localhost:5000

Publishes, per pack (drive, logic, science, arm), once a second:
  /battery/<pack>            sensor_msgs/BatteryState: voltage, current, percentage (0..1),
                             present (false without a valid reading in stale_s)
  /battery/<pack>/percent    std_msgs/Float32, 0..100 (NaN = no reading): Foxglove's gauges
"""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

from rcm_telemetry.battery import GdsClient, apply_update, default_packs, now


class BatteryNode(Node):

    def __init__(self):
        super().__init__("rcm_battery")
        p = self.declare_parameter
        self.client = GdsClient(str(p("gds_url", "http://localhost:5000").value),
                                timeout=float(p("timeout", 2.0).value))
        self.full_cell_voltage = float(p("full_cell_voltage", 4.2).value)
        self.stale_s = float(p("stale_s", 5.0).value)
        self.packs = default_packs()
        for pack in self.packs.values():  # 0 % until the RCM reports its own bounds
            pack.empty_voltage = float(p(f"{pack.name}_empty_voltage", pack.empty_voltage).value)
            pack.cells = int(p(f"{pack.name}_cells", pack.cells).value)
        self.state_pubs = {n: self.create_publisher(BatteryState, f"battery/{n}", 10)
                           for n in self.packs}
        self.percent_pubs = {n: self.create_publisher(Float32, f"battery/{n}/percent", 10)
                             for n in self.packs}
        self._connected = False
        self.create_timer(float(p("period", 1.0).value), self.tick)
        self.get_logger().info(
            f"battery from the RCM GDS at {self.client.url}: 0 % at the undervoltage bound, "
            f"100 % at {self.full_cell_voltage:g} V/cell (" + ", ".join(
                f"{k} {v.cells}S {v.empty_voltage:g}-{v.full_voltage(self.full_cell_voltage):g} V"
                for k, v in self.packs.items()) + ")")

    def tick(self):
        t = now()
        try:
            for name, val in self.client.poll():
                apply_update(self.packs, name, val, t)
            if not self._connected:
                self._connected = True
                self.get_logger().info(f"connected to the GDS ({len(self.client.names)} channels)")
        except (OSError, ValueError, KeyError) as e:
            self.client.session = None  # the GDS may have restarted: new session next time
            if self._connected:
                self.get_logger().warning(f"lost the GDS at {self.client.url}: {e}")
            else:
                self.get_logger().warning(f"no GDS at {self.client.url} ({e}); retrying",
                                          throttle_duration_sec=60.0)
            self._connected = False
        stamp = self.get_clock().now().to_msg()
        for name, pack in self.packs.items():
            fresh = t - pack.updated <= self.stale_s
            pct = pack.percentage(self.full_cell_voltage) if fresh else math.nan
            msg = BatteryState(
                voltage=pack.voltage if fresh else math.nan,
                current=pack.current if fresh else math.nan,
                percentage=pct,
                design_capacity=math.nan, capacity=math.nan, charge=math.nan,
                power_supply_status=BatteryState.POWER_SUPPLY_STATUS_UNKNOWN,
                power_supply_technology=BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO,
                present=fresh, location=name, serial_number="")
            msg.header.stamp, msg.header.frame_id = stamp, "base_link"
            self.state_pubs[name].publish(msg)
            self.percent_pubs[name].publish(Float32(data=100.0 * pct))


def main():
    rclpy.init()
    try:
        rclpy.spin(BatteryNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
