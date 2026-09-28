# BILLEE Simulation Run Guide

## Overview

This guide details how to run the gazebo simulator for the software stack.

## Prerequisites

1. `make setup <platform>` has been run, and the devcontainer is up (not needed for `linux-aarch64`, which runs natively)
2. You have a controller connected
3. You have a foxglove account or the foxglove desktop app
4. (CAN Sim Only): `kmod` and `can-utils` are installed on your host computer

## Getting the CAN interface set up

On your host machine:

1. run `./tooling/can-up vcan0`

## Running the Simulator

1. run `./tooling/sim-up`
    - this will start the gazebo simulator with the foxglove bridge and controller by default
    - `make sim` from the repo root does the same, inside the platform's container where there is one
    - it uses the Pixi environment for the platform you ran `make setup <platform>` with (see the top-level README → Setup); override with `BILLEE_PLATFORM=<mac|linux-aarch64|l4t|x86> ./tooling/sim-up`
    - viewer: on Linux with a display it also opens **RViz** (`RVIZ=false` to skip); on the Mac, and anywhere without a display, Gazebo runs under `xvfb-run -a` and you view in Foxglove (force with `HEADLESS=1`)
    - the ODESC shadow on `vcan0` is added automatically when `vcan0` exists (`SHADOW=true|false` to force)
    - controls: hold the right bumper (RB) as the safety button, then right trigger (RT) drives forward, left trigger (LT) reverses and the left stick steers left/right. Without RB held, the robot will not drive. Triggers read as released until they are pulled once
    - if you would like to remap this please edit `teleop/config/joystick.yaml` (the same file teleop.launch.py uses); `scheme: arcade` or `scheme: tank` picks the control scheme
2. Go to your foxglove web dashboard or desktop app:
    1. Select "Open New Connection"
    2. Select "Foxglove Websocket"
    3. Leave the default URL
    4. Select "Open"
3. In foxglove: Layouts → Import from file → `ros2_ws/src/chassis_bringup/foxglove/drivetrain.json`
    - it already has the grid, TF, odometry trail and the robot model from the `robot_description` topic, with the mesh up-axis set to z-up
    - to drive without a gamepad (e.g. on a Mac), add a Teleop panel publishing to `/diff_drive_controller/cmd_vel_unstamped`
