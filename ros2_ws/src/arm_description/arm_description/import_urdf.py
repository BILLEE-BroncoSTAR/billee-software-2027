"""Turn an exported arm URDF (SolidWorks / Fusion / Onshape exporter) into arm_model.urdf.xacro.

    ros2 run arm_description import_urdf path/to/exported.urdf --tip-link <end effector link> \\
        --joint-map joint_axis_1:joint_arm_1 joint_axis_2:joint_arm_2 ... \\
        --meshes path/to/exported/meshes

    # without building: python3 arm_description/import_urdf.py ... (from this package's directory)

The rest of the rover only relies on what this writes, so a new arm is a re-run of this command:

  - a xacro macro `arm_model(prefix)` whose links are named ${prefix}<exported name> (the rover
    already has a `base_link`, so the arm's root becomes `arm_base_link`);
  - the properties `arm_model_root_link` / `arm_model_tip_link` (exported names, no prefix):
    urdf/arm.urdf.xacro attaches the root to the rover and the RealSense to the tip;
  - joints renamed with --joint-map to the names arm_bringup publishes on /arm/joint_states
    (config/arm_axes.yaml: joint_arm_1 ... joint_arm_5), and NOT prefixed, so
    robot_state_publisher moves them from the real encoders. A movable joint left out of the map
    becomes fixed (nothing publishes its state, and robot_state_publisher would leave its child
    links out of TF), unless --keep-unmapped;
  - mesh paths rewritten to package://arm_description/meshes/<file>; --meshes copies them there;
  - <ros2_control>, <gazebo>, <transmission> and xacro args dropped (arm_bringup generates its
    own ros2_control from arm_axes.yaml), material names prefixed (the rover's URDF has its own).

Input: a .urdf, or a .xacro (expanded with xacro first; `$(find <pkg>)` resolves to the folder
above the file's urdf/ directory, which is how exporters lay packages out).
"""

from __future__ import annotations

import argparse
import datetime
import os
import re
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

MOVABLE = ("revolute", "continuous", "prismatic", "planar", "floating")
DROP = ("ros2_control", "gazebo", "transmission")
XACRO_NS = "http://www.ros.org/wiki/xacro"


def load_urdf(path: str) -> ET.Element:
    """Root <robot> element of a URDF, or of a xacro file once expanded."""
    with open(path) as f:
        text = f.read()
    if path.endswith(".xacro") or f"xmlns:xacro=\"{XACRO_NS}\"" in text:
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(path)))
        text = re.sub(r"\$\(find [^)]+\)", pkg_dir.replace("\\", "/"), text)
        import xacro  # ros-humble-xacro (pixi), or `pip install xacro`

        with tempfile.NamedTemporaryFile("w", suffix=".xacro", dir=os.path.dirname(
                os.path.abspath(path)), delete=False) as tmp:
            tmp.write(text)
        try:
            text = xacro.process_file(tmp.name).toxml()
        finally:
            os.unlink(tmp.name)
    return ET.fromstring(text)


def convert(robot: ET.Element, tip_link: str, joint_map: dict[str, str],
            keep_unmapped: bool = False, source: str = "") -> tuple[str, list[str], list[str]]:
    """-> (xacro text, mesh file names referenced, notes for the user)."""
    links = [e for e in robot if e.tag == "link"]
    joints = [e for e in robot if e.tag == "joint"]
    link_names = {e.get("name") for e in links}
    children = {j.find("child").get("link") for j in joints}
    roots = sorted(link_names - children)
    if len(roots) != 1:
        raise ValueError(f"expected one root link, found {roots}")
    if tip_link not in link_names:
        raise ValueError(f"--tip-link {tip_link!r} is not a link; links: {sorted(link_names)}")
    unknown = set(joint_map) - {j.get("name") for j in joints}
    if unknown:
        raise ValueError(f"--joint-map names joints that don't exist: {sorted(unknown)}")

    notes, meshes = [], []
    materials = {m.get("name") for m in robot.iter("material") if m.get("name")}
    out = ET.Element("xacro:macro", {"name": "arm_model", "params": "prefix"})
    for m in [e for e in robot if e.tag == "material"]:
        out.append(m)
    for e in links + joints:
        out.append(e)

    for link in links:
        link.set("name", "${prefix}" + link.get("name"))
    for joint in joints:
        name = joint.get("name")
        for tag in ("parent", "child"):
            el = joint.find(tag)
            el.set("link", "${prefix}" + el.get("link"))
        if name in joint_map:
            joint.set("name", joint_map[name])
        else:
            joint.set("name", "${prefix}" + name)
            if joint.get("type") in MOVABLE and not keep_unmapped:
                joint.set("type", "fixed")
                for tag in ("axis", "limit", "dynamics", "mimic", "safety_controller"):
                    for el in joint.findall(tag):
                        joint.remove(el)
                notes.append(f"joint {name} is not in the joint map: made fixed")
    for m in out.iter("material"):
        if m.get("name") in materials:
            m.set("name", "${prefix}" + m.get("name"))
    for mesh in out.iter("mesh"):
        filename = os.path.basename(mesh.get("filename", ""))
        meshes.append(filename)
        mesh.set("filename", f"package://arm_description/meshes/{filename}")

    ET.indent(out, space="  ", level=1)
    body = ET.tostring(out, encoding="unicode")
    stamp = datetime.date.today().isoformat()
    mapping = "".join(f"\n       {k} -> {v}" for k, v in joint_map.items()) or " none"
    # "--" can't appear inside an XML comment.
    notes_text = "".join(f"     {n.replace('--', '-')}\n" for n in notes)
    header = (
        '<?xml version="1.0"?>\n'
        "<!-- GENERATED by arm_description/import_urdf.py: re-run it for a new arm rather than\n"
        "     editing this file (docs/arm-camera.md, \"Arm URDF\").\n"
        f"     Source: {source or 'unknown'} ({stamp})\n"
        f"     Joints renamed:{mapping}\n"
        f"{notes_text}-->\n"
        f'<robot xmlns:xacro="{XACRO_NS}">\n'
        f'  <xacro:property name="arm_model_root_link" value="{roots[0]}"/>\n'
        f'  <xacro:property name="arm_model_tip_link" value="{tip_link}"/>\n'
        f"  {body.strip()}\n"
        "</robot>\n")
    return header, sorted(set(meshes)), notes


def strip_dropped(robot: ET.Element):
    for e in list(robot):
        if e.tag in DROP or e.tag.startswith(f"{{{XACRO_NS}}}") or e.tag is ET.Comment:
            robot.remove(e)


def main(argv=None):
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("urdf", help="exported .urdf or .xacro")
    ap.add_argument("--tip-link", required=True,
                    help="link the RealSense mounts on (the end effector), exported name")
    ap.add_argument("--joint-map", nargs="*", default=[], metavar="EXPORTED:REAL",
                    help="rename joints to arm_bringup's names (config/arm_axes.yaml)")
    ap.add_argument("--keep-unmapped", action="store_true",
                    help="keep movable joints that aren't in --joint-map movable")
    ap.add_argument("--meshes", help="copy these meshes into arm_description/meshes")
    ap.add_argument("--output", default=os.path.join(here, "urdf", "arm_model.urdf.xacro"))
    args = ap.parse_args(argv)

    joint_map = {}
    for item in args.joint_map:
        old, sep, new = item.partition(":")
        if not sep or not old or not new:
            ap.error(f"--joint-map entry {item!r}: expected EXPORTED:REAL")
        joint_map[old] = new
    robot = load_urdf(args.urdf)
    strip_dropped(robot)
    text, meshes, notes = convert(robot, args.tip_link, joint_map, args.keep_unmapped,
                                  source=os.path.basename(args.urdf))
    with open(args.output, "w") as f:
        f.write(text)
    print(f"wrote {args.output}")
    for n in notes:
        print(f"  {n}")

    mesh_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(args.output))),
                            "meshes")
    if args.meshes:
        os.makedirs(mesh_dir, exist_ok=True)
        for m in meshes:
            shutil.copy2(os.path.join(args.meshes, m), os.path.join(mesh_dir, m))
        print(f"copied {len(meshes)} meshes to {mesh_dir}")
    missing = [m for m in meshes if not os.path.exists(os.path.join(mesh_dir, m))]
    if missing:
        print(f"WARNING: not in {mesh_dir}: {', '.join(missing)} (pass --meshes)",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
