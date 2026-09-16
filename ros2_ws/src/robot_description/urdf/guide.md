## Gazebo camera frame issue

The camera was publishing data, but the image and point cloud were rotated 90 degrees in RViz. The root cause was a frame mismatch between the Gazebo sensor output and the ROS optical frame expected by RViz.

The fix was to keep the sensor attached to `camera_link` in the Gazebo URDF, while also keeping the URDF fixed joint from `camera_link` to `camera_link_optical`. That lets TF provide the optical transform automatically instead of rotating the data by hand.

The working setup was:

- `camera.urdf.xacro` defines `camera_link` and `camera_link_optical`
- `camera_optical_joint` provides the standard optical rotation
- `robot_gz.urdf.xacro` publishes the depth camera under `camera_link`
- the sensor output is labeled with `camera_link_optical` so RViz interprets it correctly

Important detail: the issue was not that the optical frame should be removed. It was that the published frame name and the TF tree had to agree. Once they matched, the point cloud stopped appearing rotated.

## Ground plane export note

Yeah, honestly for a flat ground plane that's probably the cleanest option — you skip all the conversion gotchas entirely since you control the export format and settings from the start.

**Basic approach in Blender**

1. **Geometry**: Add a plane (or grid if you want subdivisions for collision purposes), scale it to your world size. Keep it dead simple — a single quad is fine for a flat ground visual; you don't need geometric detail if the "tile" look is coming from a texture.

2. **UV mapping**: This is the part that actually matters for tiling. Unwrap the plane (`U` → Unwrap, or just Smart UV Project for a flat plane), then in the UV editor scale the UV coordinates up beyond 0-1 (e.g. if you want the tile to repeat 20 times across the ground, scale UVs so they span 0-20 instead of 0-1). Combined with a texture set to "Repeat" wrapping, that's what actually gives you the tiled look — not the mesh geometry.

3. **Material**: Add a Principled BSDF, plug your tile photo into the Base Color via an Image Texture node. Set the image's wrap mode to Repeat (default in Blender). If you have a normal/roughness map too, wire those in for more realism, but base color alone will look fine for most sim purposes.

4. **Export as DAE**: File → Export → Collada (.dae). Blender's COLLADA exporter is solid and writes proper `<library_materials>`/`<library_images>`/`<up_axis>` metadata — this is exactly the stuff that was uncertain/lossy in the FBX conversion path. Just double check the exporter's "Y up"/"Z up" option matches what your SDF/Gazebo setup expects (Gazebo wants Z-up).

**A couple of practical notes**

- **Export a separate collision mesh** if you care about physics performance — either the same plane at low poly, or an SDF `<plane>` primitive instead of a mesh at all, which is much cheaper than doing mesh-vs-mesh collision for a flat ground. You don't need the DAE for that part.
- **Texture packing**: when exporting, Blender by default writes a relative path to the image next to the .dae. If you're moving files into a ROS/Gazebo package structure afterward, either "Pack" the texture into the .blend first and re-export, or just make sure the final texture path lines up with wherever the .dae ends up relative to your `meshes/` folder — that's the same relative-path issue mentioned earlier, but now you have full control over it instead of inheriting whatever the FBX exporter did.
- **Scale**: since you're building it in Blender at whatever units you set the scene to (Scene Properties → Units, meters by default), your SDF `<scale>1 1 1</scale>` should just work with no fudging — that was the whole pain point with the FBX round-trip that goes away when you author it directly.

This is a very reasonable shortcut for a ground plane specifically — it's simple enough geometry that hand-authoring is genuinely less work than debugging a conversion pipeline.

| Warning | Meaning | Effect |
|---|---|---|
| `Trying to serialize/deserialize component ... World` | Gazebo’s GUI state system can’t save/restore the `World` ECS component as text. | Benign; simulation physics is unaffected. |
| `Broken filename passed to function` (4×) | Qt received an invalid/empty resource filename. | Usually a GUI-plugin/config issue. |
| `TypeError: Cannot read property ... of null` (59×) | The Ignition GUI QML theme/settings object was not initialized; toolbar, drawer, and exit-dialog settings are null. | Causes incomplete/broken GUI chrome, not robot material rendering. |
| No `material`, `texture`, mesh-load, or rendering error | The log never reports failed color/material loading. | These warnings do not explain white robot links. |
