On the Jetson, run:

```
cd ~/billee-software-2027/ros2_ws
../tooling/can-up
../tooling/can-up status can0
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
```
  12:19 PM
```
does this do default can2  

No. The default is `can0`.

For your real bus on `can2`, run:

```
cd ~/billee-software-2027/ros2_ws
../tooling/can-up can2
pixi run --environment l4t ros2 launch chassis_bringup rover.launch.py \
  mode:=real can_interface:=can2
```

`can_interface:=can2` is required for the driver to use that network.
