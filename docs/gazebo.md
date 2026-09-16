# Gazebo

## GZ Bringup PKGs

- for a file that launches gz_sim it needs to have ament_cmake as a compiler
- if textures are not loading properly, link the QML env variables

## URDF SDF Debugging

- convert the xacro to URDF and inspect

## Gazebo GPU Forwarding

- verify `glxinfo | grep "OpenGL renderer"` shows your Graphics Card
- by default new gpus are not forwarded into Ubuntu 22.04 properly must add the following to docker file to make backwards-compatible

```
RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository -y ppa:kisak/kisak-mesa \
    && apt-get update \
    && apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*
```

## Headless / no GPU (Apple-Silicon Docker, CI)

Same kisak-mesa PPA plus `xvfb`, then run launches under `xvfb-run -a` — the mesa
`llvmpipe` software renderer stands in for a GPU (`glxinfo | grep "OpenGL renderer"`
should say `llvmpipe`). Set `LIBGL_ALWAYS_SOFTWARE=1` / `GALLIUM_DRIVER=llvmpipe` to skip
GPU probing. `docker/Dockerfile.desktop.mac.humble` does this. If `ogre2` still fails,
`export IGN_GAZEBO_RENDER_ENGINE=ogre`, or skip Gazebo entirely with
`real.launch.py can_interface:=mock`.