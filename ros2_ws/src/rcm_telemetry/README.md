# rcm_telemetry

The Rover Control Module's F´ telemetry as ROS topics. For now: the charge of the four battery
packs, shown as gauges in both Foxglove layouts (`chassis_bringup/foxglove/rover.json`,
`rover_arm.json`).

`rover.launch.py` starts `battery_node` in every mode (`battery:=false` to skip it). By hand:

```bash
ros2 run rcm_telemetry battery_node --ros-args --params-file src/rcm_telemetry/config/battery.yaml
```

| Topic | Type | |
|---|---|---|
| `/battery/<pack>` | `sensor_msgs/BatteryState` | voltage, current, `percentage` 0..1, `present` |
| `/battery/<pack>/percent` | `std_msgs/Float32` | 0..100, NaN without a reading (the gauges) |

`<pack>` is `drive`, `logic`, `science` or `arm`, the RCM's four battery inputs (billee-hardware-2027
README: DRIVE 4S, the others 6S).

## Where the numbers come from

The RCM's F´ GDS runs on the Jetson host and serves its telemetry over HTTP (port 5000, the web
UI's API). The node polls it once a second. It needs no change on the F´ side and only reads.

- **Voltage:** each pack's INA780B (`inaManager.<SUB>_POWER`). DRIVE has one per wheel channel;
  the pack's voltage is the highest valid one (a channel switched off reads ~0 V), its current
  the sum. A reading with `valid: false` (the chip doesn't answer) is ignored.
- **Charge:** linear from the pack's undervoltage fault bound (0 %) to a full charge,
  cells × 4.2 V (100 %). The bound is fpManager's `<SUB>_BOUNDS` parameter (fprime-billee,
  `Components/FPManager/FPManager.fpp`: 13.6 V DRIVE, 20.4 V the 6S packs, i.e. 3.4 V/cell),
  read live from the RCM's `fpManager.*_POWER_BOUNDS` telemetry, and `config/battery.yaml`'s
  copies until that arrives. The upper bound (`vbusFaultHigh`, 35 V) is an overvoltage guard,
  far above a full pack, so it isn't used as 100 %.
- Voltage sags under load: the gauge dips while a subsystem works and recovers at rest. It's a
  dashboard meter, not a fuel gauge (that would need the RCM to count current).

Without a reading (no GDS, the RCM disconnected, a board without the INA780B chips, or nothing
valid for `stale_s`) a gauge shows no value and `present` is false: never a made-up 0 %.

## Tests

```bash
cd src/rcm_telemetry && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q test
python3 test/fake_gds.py 5099   # a fake GDS with readings, to run the node against: -p gds_url:=http://127.0.0.1:5099
```
