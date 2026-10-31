#!/usr/bin/env bash
# Download the OpenPose COCO body model (network definition + ~200 MB weights) for openpose_node.
# The CMU model server is offline, so the weights come from a pinned Hugging Face mirror.
# Usage (inside `pixi shell`): src/perception/people/scripts/get_openpose_models.sh [dest_dir]
set -euo pipefail

dest="${1:-${PIXI_PROJECT_ROOT:?run inside pixi shell, or pass a destination directory}/models/openpose}"
prototxt_url="https://raw.githubusercontent.com/CMU-Perceptual-Computing-Lab/openpose/5c5d96523ef917bd30301245fdc8343937cae48d/models/pose/coco/pose_deploy_linevec.prototxt"
weights_url="https://huggingface.co/camenduru/openpose/resolve/8fadec2b54f35105fc9929a484af4e739ea1f8d3/models/pose/coco/pose_iter_440000.caffemodel"

mkdir -p "$dest"
cd "$dest"
[ -s pose_deploy_linevec.prototxt ] || curl -fsSL -o pose_deploy_linevec.prototxt "$prototxt_url"
[ -s pose_iter_440000.caffemodel ] || curl -fL --progress-bar -o pose_iter_440000.caffemodel "$weights_url"
sha256sum -c - <<'EOF'
17051b87f709aa094e09c5da7b78e9016a1f37b2b452ed1f190fe74cce70b1ad  pose_deploy_linevec.prototxt
b4cf475576abd7b15d5316f1ee65eb492b5c9f5865e70a2e7882ed31fb682549  pose_iter_440000.caffemodel
EOF
echo "OpenPose model ready in $dest"
