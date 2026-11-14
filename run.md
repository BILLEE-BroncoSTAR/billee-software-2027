On the **Jetson host** (SSH into the Jetson, not the container; the container has no `ip`
command), bring up the CAN bus. Execute `can-up`; don't `source` it.

```
cd ~/billee-software-2027
tooling/can-up
tooling/can-up status
```

Then in the **Jetson container**:

```
cd ~/billee-software-2027/ros2_ws
pixi run --environment l4t ros2 launch chassis_bringup rover.launch.py mode:=real
```

That launches the real ODESC/CAN drivetrain plus Foxglove bridge on port `8765`.

In a second terminal, verify:

```
cd ~/billee-software-2027/ros2_ws
pixi run --environment l4t ros2 control list_hardware_components
pixi run --environment l4t ros2 control list_controllers
```

Expect `Robot`, `diff_drive_controller`, and `joint_state_broadcaster` to be `active`.

Then start teleop on the ground station:

```
cd ~/billee-software-2027/ros2_ws
pixi run --environment default ros2 launch teleop teleop.launch.py
```

Keep the rover supported / E-stop available for the first command.

The drivetrain bus is `can3`: it is the default for both `tooling/can-up` and the
launch files, so the commands above need no interface argument. To use a different
interface, pass it to both:

```
# Jetson host, not the container:
cd ~/billee-software-2027
tooling/can-up <iface>

# Jetson container:
cd ~/billee-software-2027/ros2_ws
pixi run --environment l4t ros2 launch chassis_bringup rover.launch.py \
  mode:=real can_interface:=<iface>
```
