# BILLEE Simulation Run Guide

## Overview

This guide details how to run the gazebo simulator for the software stack.

## Prerequisites

1. The devcontainer is up and running on your machine
2. You have a controller connected
3. You have a foxglove account or the foxglove desktop app
4. (CAN Sim Only): `kmod` and `can-utils` are installed on your host computer

## Getting the CAN interface set up

On your host machine:

1. run `./tooling/can-up vcan0`

## Running the Simulator

1. run `./tooling/sim-up`
    - this will start the gazebo simulator with the foxglove bridge and controller by default
    - the controls are currently left stick for left speed and right stick for right speed of drivetrain with the right bumper as the safety button. without the safety button pressed, the robot will not drive
    - if you would like to remap this please edit `chassis_bringup/config/tele_params.yaml`
2. Go to your foxglove web dashboard or desktop app:
    1. Select "Open New Connection"
    2. Select "Foxglove Websocket"
    3. Leave the default URL
    4. Select "Open"
3. In foxglove:
    1. On the 3D Panel Settings, under "Custom Layers" open the 3 dots and click "Add URDF"
    2. For source select "Topic" and then for the topic select "robot_description"
    3. The model will probably be rendered weird so under "Scene" change the Mesh-up axis to "z-up"