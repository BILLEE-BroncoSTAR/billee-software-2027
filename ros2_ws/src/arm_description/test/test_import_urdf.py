"""import_urdf.py, and the generated arm model against arm_bringup's joint names."""

import os
import re
import xml.etree.ElementTree as ET

import pytest
from arm_description.import_urdf import convert, main, strip_dropped

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ARM_AXES = os.path.join(PKG, "..", "arm_bringup", "config", "arm_axes.yaml")

EXPORTED = """<?xml version="1.0"?>
<robot name="exported">
  <material name="silver"><color rgba="0.7 0.7 0.7 1"/></material>
  <link name="base_link">
    <visual><geometry>
      <mesh filename="package://exported_description/meshes/base_link.stl"/>
    </geometry><material name="silver"/></visual>
  </link>
  <link name="upper"/>
  <link name="hand"/>
  <link name="finger"/>
  <joint name="shoulder" type="revolute">
    <parent link="base_link"/><child link="upper"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="wrist" type="continuous">
    <parent link="upper"/><child link="hand"/><axis xyz="0 1 0"/>
  </joint>
  <joint name="grip" type="prismatic">
    <parent link="hand"/><child link="finger"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="0.02" effort="1" velocity="1"/>
  </joint>
  <ros2_control name="x" type="system"><hardware><plugin>p</plugin></hardware></ros2_control>
  <gazebo><plugin filename="f" name="n"/></gazebo>
</robot>
"""


def converted(**kw):
    robot = ET.fromstring(EXPORTED)
    strip_dropped(robot)
    return convert(robot, kw.pop("tip", "hand"),
                   kw.pop("joint_map", {"shoulder": "joint_arm_1", "wrist": "joint_arm_2"}),
                   **kw)


def test_convert_names_and_joints():
    text, meshes, notes = converted()
    assert '<xacro:property name="arm_model_root_link" value="base_link"/>' in text
    assert '<xacro:property name="arm_model_tip_link" value="hand"/>' in text
    assert 'name="${prefix}base_link"' in text and 'link="${prefix}upper"' in text
    assert '<joint name="joint_arm_1" type="revolute">' in text      # mapped: no prefix
    assert '<joint name="${prefix}grip" type="fixed">' in text      # unmapped: fixed
    assert "ros2_control" not in text and "gazebo" not in text
    assert 'filename="package://arm_description/meshes/base_link.stl"' in text
    assert 'material name="${prefix}silver"' in text
    assert meshes == ["base_link.stl"]
    assert notes == ["joint grip is not in the joint map: made fixed"]
    ET.fromstring(text.split("?>", 1)[1])  # well-formed (and no "--" in the comment)


def test_keep_unmapped():
    text, _, notes = converted(keep_unmapped=True)
    assert '<joint name="${prefix}grip" type="prismatic">' in text and not notes


def test_convert_errors():
    with pytest.raises(ValueError, match="tip-link"):
        converted(tip="nope")
    with pytest.raises(ValueError, match="don't exist"):
        converted(joint_map={"elbow": "joint_arm_3"})


def test_main_writes_file(tmp_path):
    src = tmp_path / "exported.urdf"
    src.write_text(EXPORTED)
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "base_link.stl").write_bytes(b"solid")
    out = tmp_path / "out" / "urdf" / "arm_model.urdf.xacro"
    out.parent.mkdir(parents=True)
    assert main([str(src), "--tip-link", "hand", "--joint-map", "shoulder:joint_arm_1",
                 "--meshes", str(tmp_path / "meshes"), "--output", str(out)]) == 0
    assert out.exists()
    assert (tmp_path / "out" / "meshes" / "base_link.stl").exists()


def test_model_joints_match_arm_bringup():
    """robot_state_publisher moves a joint only if /arm/joint_states names it: the model's
    movable joints must be arm_bringup's joint names (config/arm_axes.yaml)."""
    with open(os.path.join(PKG, "urdf", "arm_model.urdf.xacro")) as f:
        model = f.read()
    movable = set(re.findall(r'<joint name="([^"]+)" type="(?:revolute|continuous|prismatic)"',
                             model))
    assert movable == {f"joint_arm_{i}" for i in range(1, 6)}
    if not os.path.exists(ARM_AXES):
        pytest.skip("arm_bringup not next to arm_description")
    yaml = pytest.importorskip("yaml")
    with open(ARM_AXES) as f:
        axes = {a["joint"] for a in yaml.safe_load(f)["axes"]}
    assert movable <= axes


def test_meshes_present():
    with open(os.path.join(PKG, "urdf", "arm_model.urdf.xacro")) as f:
        names = re.findall(r"package://arm_description/meshes/([^\"]+)", f.read())
    assert names
    for n in set(names):
        assert os.path.exists(os.path.join(PKG, "meshes", n)), n
