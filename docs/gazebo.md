# Gazebo notes

The sim is Gazebo Fortress (Ignition 6), started by `chassis_bringup/launch/sim_gz.launch.py`.
Always launch through that file, not `ign gazebo` directly, because it sets the resource, plugin
and QML paths Gazebo needs to find the meshes and `ign_ros2_control`.

## GPU and rendering

- Check what's rendering with `glxinfo | grep "OpenGL renderer"`. It should name your GPU (x86) or
  `llvmpipe` (Mac/headless).
- Ubuntu 22.04's mesa is too old for newer GPUs. The Dockerfiles add the `ppa:kisak/kisak-mesa`
  PPA.
- **Mac / CI (no GPU):** run under `xvfb-run -a`. The Mac container sets
  `LIBGL_ALWAYS_SOFTWARE=1` and `GALLIUM_DRIVER=llvmpipe`. If `ogre2` still fails, set
  `export IGN_GAZEBO_RENDER_ENGINE=ogre`, or skip Gazebo with
  `real.launch.py can_interface:=mock`.
- If textures don't load, the QML/GUI plugin paths are wrong. `sim_gz.launch.py` finds them from
  the active pixi env.

## Headless GPU rendering (Jetson)

The rover has no monitor, but Gazebo always needs an X display. The simulated depth camera is
rendered in the Gazebo server even with `gui:=false`, and pixi's Gazebo renderer (OGRE-Next
2.2.6 from conda-forge) only reaches OpenGL through GLX. It has no EGL build, so
`--headless-rendering` doesn't work. Without a usable display the render thread crashes
(`OGRE EXCEPTION ... Couldn't open X display`), the sim stops stepping, and the controller
spawner times out.

The Jetson image therefore includes Xvfb (a virtual X server) and
[VirtualGL](https://virtualgl.org), which runs the OpenGL calls on the GPU through EGL:

```bash
xvfb-run -a vglrun -d egl ros2 launch chassis_bringup sim_gz.launch.py gui:=false
```

`tooling/sim-up` does this when there's no working display. A stale `$DISPLAY` (set, but nothing
answers) counts as none; `xdpyinfo` tells you which. Without `vglrun`, Xvfb renders on the CPU
(`llvmpipe`). To check which one Gazebo used, run `grep GL_RENDERER ~/.ignition/rendering/ogre2.log`;
it should say `NVIDIA Tegra Orin`.

**Speed:** on the dev Orin Nano the sim runs at about 0.33× real time, with or without VirtualGL.
That was measured before `navigation` defaulted to `true`. The EKF and Nav2 servers it adds
haven't been measured on the Nano yet; `navigation:=false` removes them.
Gazebo slows sim time until each 10 Hz camera frame (1280×720 RGB-D, image + depth + point cloud)
is done. The time goes to CPU work in the Gazebo server, about 1.5 cores, not to rendering. The
camera publishes about 3 fps of wall-clock time, which is its full 10 Hz in sim time.

`vglrun` needs the GPU device groups. A terminal opened after the container starts has them
(`docker/gpu_groups.sh`). Check with `id`: you should see `video`.

## Model

- To debug the model, expand it and inspect the result:
  `xacro src/robot_description/urdf/robot.urdf.xacro > /tmp/robot.urdf`.
- **Camera frame.** The Gazebo depth sensor is attached to `camera_link`, but its output is
  labelled `camera_link_optical`. The `camera_optical_joint` in `camera.urdf.xacro` supplies the
  optical rotation. If the frame name and the TF tree disagree, the point cloud shows up rotated
  90° in RViz.
- **Ground plane meshes.** Author them in Blender rather than converting from FBX. Scale the UVs
  beyond 0–1 with Repeat wrapping to get tiling, export Collada (`.dae`) with Z-up, and keep the
  texture path relative to `meshes/`. Use an SDF `<plane>` for collision.

## Harmless warnings

| Warning | Meaning |
|---|---|
| `Trying to serialize/deserialize component ... World` | The GUI can't save the World component. Physics is unaffected. |
| `Broken filename passed to function` | Qt got an empty resource name (GUI plugin config) |
| `TypeError: Cannot read property ... of null` (many) | GUI theme not initialised. The toolbar or dialogs may look broken; rendering is fine. |
