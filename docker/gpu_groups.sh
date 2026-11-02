#!/bin/sh
# Add a user (default ros_user) to the groups that own the Jetson GPU device nodes, by gid.
#
# CUDA as a non-root user needs read/write on /dev/nvmap, /dev/nvgpu/igpu0/*, /dev/nvhost-* and
# /dev/dri/*, owned by host groups (video 44, render, ...) whose gids differ between hosts and
# from the image's /etc/group. CDI (`--device nvidia.com/gpu=all`) adds them only to the
# container's first process; `docker exec` (every VS Code terminal and task) builds the user's
# groups from the container's /etc/group, so they are written there. Run as root at container
# start (the devcontainer's postStartCommand); new exec sessions pick the groups up.
set -eu
user=${1:-ros_user}
for gid in $(stat -c %g /dev/nvmap /dev/nvgpu/igpu0/* /dev/nvhost* /dev/host1x* /dev/dri/* \
               2>/dev/null | sort -un); do
  [ "$gid" = 0 ] && continue
  group=$(getent group "$gid" | cut -d: -f1)
  if [ -z "$group" ]; then
    group=host_gpu_$gid
    groupadd -g "$gid" "$group"
  fi
  usermod -aG "$group" "$user"
done
