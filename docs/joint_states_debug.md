# Robot TF / Odometry Troubleshooting Summary

## Initial Problem

The user had a URDF (a 6-wheel rocker-bogie-style rover, with a
`base_footprint` root, `base_link`, two-stage suspension links, and six
wheel links) and was seeing TF errors: **transforms did not exist between
the wheels and `/odom`**, even though transforms *did* exist between
`base_footprint`, `base_link`, and the suspension links.

## Root Cause

1. **Fixed joints publish automatically.** In the URDF, `dummy_joint`
   (`base_footprint` → `base_link`) and the "Revolute 1–4" joints
   (despite their names, all `type="fixed"`) connecting `base_link` to
   the suspensions are all fixed joints. `robot_state_publisher`
   computes and broadcasts fixed-joint transforms automatically as soon
   as it parses the URDF — no external input needed. This is why that
   part of the chain always worked.

2. **Non-fixed joints need `/joint_states`.** The six wheel joints
   (`joint_wheel_r1/r2/r3`, `joint_wheel_l1/l2/l3`) are `type="continuous"`.
   `robot_state_publisher` will **not** publish transforms for these
   unless something publishes a `sensor_msgs/JointState` message on
   `/joint_states` containing an entry for each wheel joint name.

3. **`/odom` didn't exist at all.** `/odom` isn't a link in the URDF —
   it's expected to come from a separate odometry source (e.g. a
   diff-drive plugin or odometry node) that publishes `odom → base_footprint`
   on TF. Nothing in the file was providing that.

4. **The original file's Gazebo include was commented out**, which is
   why nothing was populating `/joint_states` or `/odom`:
   ```xml
   <!--xacro:include filename="$(find chassis_description)/urdf/chassis_model.gazebo" /-->
   ```

## Comparing Against `articubot_one` (Josh Newans' tutorial robot)

Initially assumed (incorrectly) that this repo used `ros2_control` +
`joint_state_broadcaster`. After actually pulling the real files, the
repo turned out to use a much simpler, **classic-Gazebo plugin only**
approach — no `ros2_control` involved at all:

```xml
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.35</wheel_separation>
    <wheel_diameter>0.1</wheel_diameter>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <publish_wheel_tf>true</publish_wheel_tf>
  </plugin>
</gazebo>
```

Key insight: `libgazebo_ros_diff_drive.so`, when `publish_wheel_tf` and
`publish_odom_tf` are `true`, reads wheel joint angles **directly from
the physics engine** and broadcasts the wheel and `odom → base_link`
transforms itself — completely bypassing `/joint_states` and
`robot_state_publisher` for those particular transforms. This plugin
only supports exactly one left + one right wheel joint, though, so it
doesn't map cleanly onto a 6-wheel-per-side rover.

## Chosen Path Forward

Since the user's robot has 6 wheels (not 2), the plan is:

- **Don't** rely on `diff_drive`'s built-in `publish_wheel_tf` (it only
  handles one wheel pair).
- **Do** run a separate joint-state-publishing plugin that lists *all
  six* wheel joints, so `robot_state_publisher` handles wheel TF
  uniformly through the normal `/joint_states` pipeline.

## Gazebo Classic vs. New Gazebo (gz-sim)

Gazebo Classic was discontinued in 2025; the user is moving to the new
Gazebo (Ignition/gz-sim), which uses different plugin names:

| Purpose | Gazebo Classic | New Gazebo (gz-sim) |
|---|---|---|
| Joint state publishing | `libgazebo_ros_joint_state_publisher.so` | `gz-sim-joint-state-publisher-system` / `gz::sim::systems::JointStatePublisher` |
| Differential drive | `libgazebo_ros_diff_drive.so` | `gz-sim-diff-drive-system` / `gz::sim::systems::DiffDrive` |

Important new-Gazebo detail: `JointStatePublisher` publishes onto the
**Gazebo transport layer**, not directly to ROS's `/joint_states` topic.
A `ros_gz_bridge` config is needed to bridge it into ROS 2.

### Final joint-state-publisher config for this robot

```xml
<gazebo>
  <plugin filename="gz-sim-joint-state-publisher-system"
          name="gz::sim::systems::JointStatePublisher">
    <joint_name>joint_wheel_r1</joint_name>
    <joint_name>joint_wheel_r2</joint_name>
    <joint_name>joint_wheel_r3</joint_name>
    <joint_name>joint_wheel_l1</joint_name>
    <joint_name>joint_wheel_l2</joint_name>
    <joint_name>joint_wheel_l3</joint_name>
  </plugin>
</gazebo>
```

## Which Joints to List: Final Answer

**Only non-fixed joints** need to be listed in `JointStatePublisher` —
in this robot's case, that's just the six wheel joints. Fixed joints
(`dummy_joint`, `Revolute 1`–`4`, and any suspension joints) don't need
entries because `robot_state_publisher` already computes their
transforms automatically from the URDF, with no runtime state required.

- Listing a fixed joint is harmless but pointless (no DOF to report).
- **Omitting** a genuinely non-fixed joint reproduces the original
  "transform does not exist" error for that joint.
- Any future non-fixed joints added to the robot (steering, arm,
  sensor pan/tilt, etc.) would need to be added to this list too.

## Continued Debugging: Same "no transform wheel → odom" Error Persisted

After correcting the plugin naming (matching `ignition::gazebo::systems::*`
/ `libignition-gazebo-*-system.so` consistently for both `DiffDrive` and
`JointStatePublisher`), the error still occurred. Diagnosis moved to
checking whether `JointStatePublisher`'s output was actually reaching
ROS.

### Key finding: gz-sim plugins publish on Gazebo transport, not ROS topics directly

`JointStatePublisher` publishes onto the **Gazebo transport layer**
(not directly to ROS's `/joint_states` topic). Without a `ros_gz_bridge`
node bridging that Gazebo topic into ROS, `robot_state_publisher` never
receives any joint data — even though the plugin itself runs fine in
Gazebo. This matched the symptom: `odom → base_footprint` worked (since
`DiffDrive` publishes straight into ROS/tf, no bridge needed) but the
wheels still failed.

### Why the JointStatePublisher topic looked different from DiffDrive's

By default, `JointStatePublisher` (with no `<topic>` override) publishes
on a scoped topic derived from the simulation entity hierarchy:
```
/world/<world_name>/model/<model_name>/joint_state
```
whereas `DiffDrive` uses short, hardcoded defaults like
`/model/<model_name>/cmd_vel` and `/model/<model_name>/odometry`. This
is simply a difference in each plugin's default topic-naming behavior,
not a bug.

**Fix:** add an explicit `<topic>` override to `JointStatePublisher` so
its topic name doesn't depend on the current world's name (avoids
breaking the bridge config every time the world file changes):
```xml
<gazebo>
  <plugin name="ignition::gazebo::systems::JointStatePublisher"
          filename="libignition-gazebo-joint-state-publisher-system.so">
    <topic>joint_states</topic>
    <joint_name>joint_wheel_r1</joint_name>
    <joint_name>joint_wheel_r2</joint_name>
    <joint_name>joint_wheel_r3</joint_name>
    <joint_name>joint_wheel_l1</joint_name>
    <joint_name>joint_wheel_l2</joint_name>
    <joint_name>joint_wheel_l3</joint_name>
  </plugin>
</gazebo>
```

### Bridging into ROS: launch file `Node`

Rather than running `ros2 run ros_gz_bridge parameter_bridge` by hand,
the bridge was added as a `Node` action in the Python launch file
(the standard/idiomatic approach), e.g.:
```python
joint_state_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        'joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
    ],
    output='screen'
)
```

## Root Cause of the Final "Still Not Working" Report

After adding the correct plugin config and building `joint_state_bridge`
in the launch file, the transform error still occurred. On inspection of
the full launch file, the bug was **not** in the URDF, plugin config, or
bridge arguments at all — it was a plain launch-file mistake:

**`joint_state_bridge` was constructed but never included in the list of
nodes returned by `_launch_description`.** In ROS 2 launch, a `Node(...)`
object that isn't part of the returned `LaunchDescription`/list is inert
— it's simply never started. The `return` statement included
`odom_bridge`, `tf_bridge`, `cmd_vel_bridge`, `robot_state_publisher`,
and `spawn_entity`, but omitted `joint_state_bridge`.

### Fix

```python
return set_env + [
        set_resource_path,
        gz_sim_launch,
        odom_bridge,
        tf_bridge,
        cmd_vel_bridge,
        joint_state_bridge,   # <-- was missing
        robot_state_publisher,
        spawn_entity,
    ]
```

**This resolved the issue.** Adding the missing node to the launch
description allowed the joint-state bridge process to actually start,
letting `robot_state_publisher` receive `/joint_states` and correctly
publish the wheel transforms, resolving the original "no transform from
wheel to odom" error.

## Still Open

- Confirm exact ROS 2 distro / Gazebo version in use, since plugin
  names differed during the Ignition → gz-sim naming transition
  (`ignition-gazebo-*-system` vs. `gz-sim-*-system`).
- Decide how the other 4 wheels (beyond whichever pair might get driven)
  receive velocity commands, since `DiffDrive` only drives one
  left/right pair — likely needs a joint-controller system or custom
  node to mirror commands across all 6 wheels.
- The launch file's `gz_sim_launch` hardcodes `"gz_args": f"-r -v 4 empty.sdf"`
  regardless of the `world_file` launch argument — worth fixing so
  different worlds can actually be selected, especially since
  `joint_state_bridge`'s topic no longer depends on `world_name` after
  the `<topic>` override, reducing (but not eliminating) the impact of
  this mismatch.