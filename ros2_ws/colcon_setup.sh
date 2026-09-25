#!/usr/bin/env bash

# The Pixi colcon-shell task uses this file as an interactive Bash startup file.
if [ "${COLCON_SETUP_SOURCE_BASHRC:-0}" = "1" ]; then
  unset COLCON_SETUP_SOURCE_BASHRC

  if [ -f "${HOME}/.bashrc" ]; then
    source "${HOME}/.bashrc"
  fi
fi

if [ -f "${CONDA_PREFIX}/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ]; then
  source "${CONDA_PREFIX}/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
fi

if [ -f "${CONDA_PREFIX}/share/colcon_cd/function/colcon_cd.sh" ]; then
  source "${CONDA_PREFIX}/share/colcon_cd/function/colcon_cd.sh"
fi
