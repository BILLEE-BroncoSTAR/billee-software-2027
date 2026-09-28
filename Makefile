# Machine-specific setup for the BILLEE workspace.
#
#   make setup mac             Apple-Silicon Mac (Docker Desktop) -> mac-cpu image
#   make setup linux-aarch64   ARM64 Linux, no NVIDIA GPU -> native Pixi linux-aarch64 env
#   make setup l4t             NVIDIA Jetson rover (JetPack / L4T) -> rover image + l4t env
#   make setup x86             x86-64 Linux + NVIDIA GPU (ground station) -> NVIDIA driver, Docker,
#                              NVIDIA Container Toolkit (tooling/install-x86-host), desktop image + default env
#
# After setup, `make build` rebuilds and `make shell` opens a ROS 2 shell for the
# recorded platform - natively, or through tooling/desktop-ros2 when it lives in a container.
#
# `make setup-<platform>` is equivalent. Run from the repo root on the host. Inside a
# devcontainer that mounts the whole repo (e.g. .devcontainer/l4t), the same command
# skips the host steps and only installs + builds the Pixi environment.
#
# Options:
#   SKIP_APT=1   don't install host packages (apt packages; for x86 also the NVIDIA
#                driver, Docker and the NVIDIA Container Toolkit)

SHELL := /bin/bash
.SHELLFLAGS := -euo pipefail -c
.DEFAULT_GOAL := help

PLATFORMS := mac linux-aarch64 l4t x86
PLATFORM := $(firstword $(filter $(PLATFORMS),$(MAKECMDGOALS)))

REPO_ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
WS := $(REPO_ROOT)/ros2_ws

# Keep in sync with PIXI_VERSION in docker/Dockerfile.*
PIXI_VERSION := 0.76.1
PIXI := $(or $(shell command -v pixi 2>/dev/null),$(HOME)/.pixi/bin/pixi)

HOST_OS := $(shell uname -s)
HOST_ARCH := $(shell uname -m)
IN_CONTAINER := $(shell [ -f /.dockerenv ] && echo 1)
SKIP_APT ?=

# `make setup` records the platform here; tooling/billee-env.sh reads it so the
# shell config, sim-up and desktop-ros2 all use the matching Pixi env / image.
PLATFORM_FILE := $(WS)/.billee-platform
CONTAINER := $(REPO_ROOT)/tooling/desktop-ros2
# Host packages for the virtual/real CAN bus (tooling/can-up, candump/cansend).
APT_PACKAGES := can-utils kmod

# First l4t build is memory-limited on the Orin Nano: 2 workers, sequential (README).
L4T_FIRST_BUILD := colcon build --symlink-install --parallel-workers 2 \
	--executor sequential --event-handlers console_direct+ --base-paths src \
	--cmake-args -DCMAKE_BUILD_TYPE=Release

.PHONY: help print-banner setup setup-usage $(PLATFORMS) setup-mac setup-linux-aarch64 \
	setup-l4t setup-x86 build shell sim apt-deps pixi

help: ## Show available commands
	@$(MAKE) --no-print-directory print-banner
	@echo "Available commands:"
	@grep -E '^[A-Za-z0-9_.-]+:.*##' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*##"} {printf "  %-22s %s\n", $$1, $$2}'
	@echo
	@echo "Usage: make setup <$(subst $() ,|,$(PLATFORMS))>   (same as make setup-<platform>)"
	@echo "Options: SKIP_APT=1 skips apt-installing host packages ($(APT_PACKAGES))"
	@echo

print-banner: ## Print the project splash screen
	@echo ""
	@echo "██████╗  ██╗██╗     ██╗     ███████╗███████╗"
	@echo "██╔══██╗ ██║██║     ██║     ██╔════╝██╔════╝"
	@echo "██████╔╝ ██║██║     ██║     █████╗  █████╗  "
	@echo "██╔══██╗ ██║██║     ██║     ██╔══╝  ██╔══╝  "
	@echo "██████╔╝ ██║███████╗███████╗███████╗███████╗"
	@echo "╚═════╝  ╚═╝╚══════╝╚══════╝╚══════╝╚══════╝"
	@echo ""
	@echo "        Rover Software Stack"
	@echo "        Powered by ROS2 Humble"
	@echo ""

setup: $(if $(PLATFORM),setup-$(PLATFORM),setup-usage) ## Set up this machine: make setup <mac|linux-aarch64|l4t|x86>

setup-usage:
	@echo "usage: make setup <$(subst $() ,|,$(PLATFORMS))>" >&2; exit 2

# Platform names are arguments to `setup`; on their own they do nothing.
$(PLATFORMS):
	@$(if $(filter setup,$(MAKECMDGOALS)),:,echo "did you mean 'make setup $@'?" >&2; exit 2)

# Write platform $(1) to ros2_ws/.billee-platform.
define record_platform
	@echo $(1) > "$(PLATFORM_FILE)"
	@echo "Recorded platform '$(1)' in ros2_ws/.billee-platform"
endef

# Shell snippet: if ros2_ws/build was configured with a different Pixi env than $(1)
# (e.g. after switching mac-cpu -> linux-aarch64), remove build/ install/ log/ -
# CMake caches and installed scripts embed the env path, so they can't be reused.
define clean_stale_build
cd "$(WS)" && if [ -d build ] && grep -rqs --include=CMakeCache.txt ".pixi/envs/" build \
  && ! grep -rqs --include=CMakeCache.txt ".pixi/envs/$(1)/" build; then \
  echo "ros2_ws/build was made with another Pixi environment - removing build/ install/ log/"; \
  rm -rf build install log; fi
endef

# pixi install + colcon build of environment $(1) in ros2_ws.
define pixi_build
	cd "$(WS)" && "$(PIXI)" install --environment $(1)
	@$(call clean_stale_build,$(1))
	cd "$(WS)" && "$(PIXI)" run --environment $(1) build
endef

define require_linux_aarch64
	@[ "$(HOST_OS)" = Linux ] && [ "$(HOST_ARCH)" = aarch64 ] || { \
	  echo "$@ needs an ARM64 Linux machine (this is $(HOST_OS)/$(HOST_ARCH))." >&2; exit 1; }
endef

define require_nvidia_runtime
	@docker info --format '{{json .Runtimes}}' | grep -q nvidia || { \
	  echo "Docker has no 'nvidia' runtime - install the NVIDIA Container Toolkit (see README.md)." >&2; exit 1; }
endef

# Run `pixi $(1)` in ros2_ws for the recorded platform: directly when natively
# installed (linux-aarch64) or already inside a container, else in its container.
define in_platform_env
	@source "$(REPO_ROOT)/tooling/billee-env.sh"; \
	echo "Platform $$BILLEE_PLATFORM, Pixi environment $$BILLEE_PIXI_ENV"; \
	if [ "$(IN_CONTAINER)" = 1 ] || [ "$$BILLEE_PLATFORM" = linux-aarch64 ]; then \
	  cd "$$BILLEE_WS" && pixi $(1); \
	else \
	  "$(CONTAINER)" run pixi $(1); \
	fi
endef

define require_docker
	@command -v docker >/dev/null || { echo "Docker not found - install it first." >&2; exit 1; }
	@docker info >/dev/null 2>&1 || { \
	  echo "Can't reach the Docker daemon. Is it running, and are you in the 'docker' group?" >&2; exit 1; }
endef

# ---------------------------------------------------------------------------- mac

setup-mac: ## Apple-Silicon Mac: build the mac-cpu image, then open the Mac devcontainer
	$(call record_platform,mac)
ifeq ($(IN_CONTAINER),1)
	$(call pixi_build,mac-cpu)
else
	@[ "$(HOST_ARCH)" = arm64 ] || [ "$(HOST_ARCH)" = aarch64 ] || { \
	  echo "The mac-cpu environment is linux-aarch64 only; this machine is $(HOST_ARCH)." >&2; exit 1; }
	$(require_docker)
	BILLEE_PLATFORM=mac "$(CONTAINER)" build
	@echo
	@echo "Built billee-mac-cpu:latest (Pixi mac-cpu env + colcon build baked in)."
	@echo
	@echo "Drive the simulated rover in Foxglove:"
	@echo "  1. Start the sim (headless Gazebo + Foxglove bridge on port 8765):"
	@echo "       make sim"
	@echo "     or in VS Code (Dev Containers: Reopen in Container -> desktop-roshumble-mac-cpu),"
	@echo "     in a container terminal:  ../tooling/sim-up"
	@echo "  2. Open Foxglove Studio on the Mac -> Open connection -> Foxglove WebSocket"
	@echo "     -> ws://localhost:8765"
	@echo "  3. Layouts -> Import from file -> ros2_ws/src/chassis_bringup/foxglove/drivetrain.json"
	@echo "  4. Drive: add a Teleop panel publishing to /diff_drive_controller/cmd_vel_unstamped,"
	@echo "     or in a second terminal: make shell, then"
	@echo "       ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped"
	@echo "  Full walkthrough: README.md -> Setup -> Apple-Silicon Mac"
endif
	@$(MAKE) --no-print-directory print-banner

# ------------------------------------------------------------------ linux-aarch64

setup-linux-aarch64: ## ARM64 Linux, no NVIDIA GPU: install Pixi + build the linux-aarch64 env natively
	$(require_linux_aarch64)
	$(call record_platform,linux-aarch64)
ifneq ($(IN_CONTAINER),1)
	@$(MAKE) --no-print-directory apt-deps pixi APT_PACKAGES="$(APT_PACKAGES) xvfb"
endif
	$(call pixi_build,linux-aarch64)
	@echo
	@echo "Built the linux-aarch64 environment natively in ros2_ws/.pixi."
	@echo "Next:   make sim    (Gazebo + RViz window + Foxglove on :8765)"
	@echo "        make shell  (ROS 2 shell for everything else)"
	@echo "        ROS 2 terminal without make: cd ros2_ws && pixi shell -e linux-aarch64"
	@echo "Python: set the VS Code interpreter to ros2_ws/.pixi/envs/linux-aarch64/bin/python3"
	@$(MAKE) --no-print-directory print-banner

# ---------------------------------------------------------------------------- l4t

setup-l4t: ## NVIDIA Jetson rover: build the rover image + l4t env inside it
	$(require_linux_aarch64)
	$(call record_platform,l4t)
ifeq ($(IN_CONTAINER),1)
	cd "$(WS)" && "$(PIXI)" install --environment l4t
	cd "$(WS)" && if [ -d install ]; then \
	  "$(PIXI)" run --environment l4t build; \
	else \
	  "$(PIXI)" run --environment l4t $(L4T_FIRST_BUILD); \
	fi
else
	@$(MAKE) --no-print-directory apt-deps
	$(require_docker)
	$(require_nvidia_runtime)
	BILLEE_PLATFORM=l4t "$(CONTAINER)" build
	@# The container mounts the repo at the devcontainer's path, so this runs the
	@# in-container branch above (and the env is reusable from VS Code).
	BILLEE_PLATFORM=l4t "$(CONTAINER)" run make -C .. setup l4t
	@echo
	@echo "Built rover-ros2:latest and the l4t environment."
	@echo "Next: make shell   (or VS Code -> rover-roshumble_l4t-aarch64)"
endif
	@$(MAKE) --no-print-directory print-banner

# ---------------------------------------------------------------------------- x86

setup-x86: ## x86-64 + NVIDIA ground station: install driver/Docker/NVIDIA toolkit, build image + env
	@[ "$(HOST_OS)" = Linux ] && [ "$(HOST_ARCH)" = x86_64 ] || { \
	  echo "$@ needs an x86-64 Linux machine (this is $(HOST_OS)/$(HOST_ARCH))." >&2; exit 1; }
	$(call record_platform,x86)
ifeq ($(IN_CONTAINER),1)
	$(call pixi_build,default)
else
ifeq ($(SKIP_APT),)
	@# NVIDIA driver, Docker Engine, NVIDIA Container Toolkit. Exits 3 when a reboot or
	@# re-login is needed; re-running make setup x86 then continues from there.
	"$(REPO_ROOT)/tooling/install-x86-host"
endif
	$(require_docker)
	$(require_nvidia_runtime)
	BILLEE_PLATFORM=x86 "$(CONTAINER)" build
	BILLEE_PLATFORM=x86 "$(CONTAINER)" run make -C .. setup x86
	@echo
	@echo "Built billee-desktop:latest and the default environment."
	@echo "Next: make shell   (or VS Code -> desktop-roshumble_dev-x862)"
endif
	@$(MAKE) --no-print-directory print-banner

# ------------------------------------------------------------------- day to day

build: ## Rebuild ros2_ws for this machine (run make setup first)
	@source "$(REPO_ROOT)/tooling/billee-env.sh"; \
	if [ "$(IN_CONTAINER)" = 1 ] || [ "$$BILLEE_PLATFORM" = linux-aarch64 ]; then \
	  $(call clean_stale_build,$$BILLEE_PIXI_ENV); \
	fi
	$(call in_platform_env,run --environment "$$BILLEE_PIXI_ENV" build)

shell: ## Open a shell with ROS 2 + the workspace sourced, for this machine
	$(call in_platform_env,shell --environment "$$BILLEE_PIXI_ENV")

# tooling/sim-up picks the viewer: Mac -> headless Gazebo + Foxglove bridge on :8765
# (published to the Mac), Linux with a display -> RViz (+ Foxglove). Every container
# has tooling/ at ../tooling relative to its ros2_ws working directory.
sim: ## Run the Gazebo sim with this machine's viewer (Mac: Foxglove, Linux: RViz)
	@source "$(REPO_ROOT)/tooling/billee-env.sh"; \
	if [ "$(IN_CONTAINER)" = 1 ] || [ "$$BILLEE_PLATFORM" = linux-aarch64 ]; then \
	  "$(REPO_ROOT)/tooling/sim-up"; \
	else \
	  "$(CONTAINER)" run ../tooling/sim-up; \
	fi

# --------------------------------------------------------------------- helpers

apt-deps:
ifeq ($(SKIP_APT),)
	@if command -v apt-get >/dev/null && ! dpkg -s $(APT_PACKAGES) >/dev/null 2>&1; then \
	  echo "Installing host packages: $(APT_PACKAGES)"; \
	  sudo apt-get update && sudo apt-get install -y --no-install-recommends $(APT_PACKAGES); \
	fi
endif

pixi:
	@if [ ! -x "$(PIXI)" ]; then \
	  echo "Installing pixi $(PIXI_VERSION) to ~/.pixi"; \
	  curl -fsSL https://pixi.sh/install.sh | PIXI_VERSION=v$(PIXI_VERSION) bash; \
	fi
