#!/usr/bin/env bash
# Download OpenPose's body models into its own folder layout (pose/body_25, pose/coco) for
# zed_openpose_node's `model_folder`. OpenPose's getModels.sh points at CMU's model server, which is
# offline, so the networks come from OpenPose itself (pinned commit) and the weights from a pinned
# Hugging Face mirror; every file is checksum-verified.
# Usage (inside `pixi shell`): src/perception/zed_openpose/scripts/get_openpose_models.sh [dest_dir]
set -euo pipefail

dest="${1:-${PIXI_PROJECT_ROOT:?run inside pixi shell, or pass a destination directory}/models/openpose}"
openpose="https://raw.githubusercontent.com/CMU-Perceptual-Computing-Lab/openpose/5c5d96523ef917bd30301245fdc8343937cae48d/models"
mirror="https://huggingface.co/camenduru/openpose/resolve/8fadec2b54f35105fc9929a484af4e739ea1f8d3/models"

fetch() {  # fetch <base url> <path under models/> <sha256>
  mkdir -p "$dest/$(dirname "$2")"
  [ -s "$dest/$2" ] || curl -fL --progress-bar -o "$dest/$2" "$1/$2"
  echo "$3  $dest/$2" | sha256sum -c -
}

# BODY_25: zed-openpose's default model (25 keypoints)
fetch "$openpose" pose/body_25/pose_deploy.prototxt         44d6ed3a5268d8d41ca59b3a040491277d876975c3234d82cf7ec0539b4b1f61
fetch "$mirror"   pose/body_25/pose_iter_584000.caffemodel  44e3d7ebd8c8b62d4366d67127f1b562611a9e8fd0f4f3cdeeb4bb4a6ed12be6
# COCO: model_pose:=COCO (18 keypoints)
fetch "$openpose" pose/coco/pose_deploy_linevec.prototxt    17051b87f709aa094e09c5da7b78e9016a1f37b2b452ed1f190fe74cce70b1ad
fetch "$mirror"   pose/coco/pose_iter_440000.caffemodel     b4cf475576abd7b15d5316f1ee65eb492b5c9f5865e70a2e7882ed31fb682549
echo "OpenPose models ready in $dest"
