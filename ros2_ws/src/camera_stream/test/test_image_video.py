"""image_video_publisher's x264 path: raw frames in, one H.264 access unit per frame out, each
keyframe carrying SPS/PPS (a viewer can join there). Needs ffmpeg with libx264 (pixi's)."""

import shutil
import subprocess

import pytest
from camera_stream.csi_compressed_video_publisher import AUD, split_access_units
from camera_stream.image_video_publisher import encoder_command, find_ffmpeg

W, H, N = 64, 48, 40
GOP = 10


@pytest.fixture(scope="module")
def ffmpeg():
    path = find_ffmpeg() or shutil.which("ffmpeg")
    if not path:
        pytest.skip("no ffmpeg")
    if b"libx264" not in subprocess.run([path, "-hide_banner", "-encoders"],
                                        capture_output=True, check=False).stdout:
        pytest.skip("ffmpeg without libx264")
    return path


def nal_types(unit: bytes):
    out, i = [], unit.find(b"\x00\x00\x01")
    while i != -1:
        out.append(unit[i + 3] & 0x1F)
        i = unit.find(b"\x00\x00\x01", i + 3)
    return out


def test_x264_one_access_unit_per_frame(ffmpeg):
    frames = b"".join(bytes([(k * 6 + x) % 256 for x in range(W * H * 3)]) for k in range(N))
    argv = encoder_command("x264", W, H, "rgb8", 30, 200_000, GOP, ffmpeg)
    out = subprocess.run(argv, input=frames, capture_output=True, timeout=60, check=False)
    assert out.returncode == 0, out.stderr.decode()
    buf = bytearray(out.stdout) + AUD["h264"]  # a trailing delimiter flushes the last unit
    units = split_access_units(buf, AUD["h264"])
    assert len(units) == N
    keyframes = [k for k, u in enumerate(units) if 5 in nal_types(u)]  # 5 = IDR slice
    assert keyframes == list(range(0, N, GOP))
    for k in keyframes:
        assert {7, 8} <= set(nal_types(units[k]))  # SPS + PPS with every keyframe


def test_rejects_unknown_encoder():
    with pytest.raises(ValueError):
        encoder_command("vp9", W, H, "rgb8", 30, 1, 1)
