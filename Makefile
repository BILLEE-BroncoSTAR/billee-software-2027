# billee-software-2027 — the BILLEE dashboard, from the ground station
#
#   make                 list these commands (same as make help)
#   make setup-dash      set up the dashboard: asks for the Jetson's IP, SSH key to the rover, checks
#   make start           start the dashboard (http://localhost:7780) and open the RCM GDS
#   make test-dash       run the dashboard's tests
#
#   make start ARGS="--no-browser"   pass options to the dashboard (see ARGS="--help")
#
# Run these on the ground station HOST, not in a dev container: the dashboard starts the
# ground station's dev container itself and drives the rover over SSH. The dashboard is the
# BILLEE-dashboard-tool/ submodule (github.com/LucaLanzi/BILLEE-dashboard-tool).

DASH_DIR := BILLEE-dashboard-tool
ARGS     ?=

.DEFAULT_GOAL := help
.PHONY: help banner setup-dash start test-dash dash-submodule

help: ## list these commands
	@$(MAKE) --no-print-directory banner
	@echo "Commands:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
	  | awk 'BEGIN {FS = ":.*?## "} {printf "  \033[36mmake %-12s\033[0m %s\n", $$1, $$2}'
	@echo
	@echo '  make start ARGS="--no-browser"   pass options to the dashboard (see ARGS="--help")'

setup-dash: dash-submodule ## set up the dashboard: asks for the Jetson's IP, SSH key to the rover, checks
	@bash $(DASH_DIR)/scripts/setup.sh
	@$(MAKE) --no-print-directory banner

start: dash-submodule ## start the dashboard (http://localhost:7780) and open the RCM GDS
	@exec $(DASH_DIR)/billee-dash $(ARGS)

test-dash: dash-submodule ## run the dashboard's tests
	@cd $(DASH_DIR) && python3 -m unittest discover -s tests

# Fetches the dashboard submodule if this clone doesn't have it yet.
dash-submodule:
	@[ -f $(DASH_DIR)/billee-dash ] || git submodule update --init $(DASH_DIR)

banner:
	@echo ""
	@echo "	   ██████╗ ██╗██╗     ██╗     ███████╗███████╗"
	@echo "	   ██╔══██╗██║██║     ██║     ██╔════╝██╔════╝"
	@echo "	   ██████╔╝██║██║     ██║     █████╗  █████╗"
	@echo "	   ██╔══██╗██║██║     ██║     ██╔══╝  ██╔══╝"
	@echo "	   ██████╔╝██║███████╗███████╗███████╗███████╗"
	@echo "	   ╚═════╝ ╚═╝╚══════╝╚══════╝╚══════╝╚══════╝"
	@echo ""
	@echo "          Rover Control Module  ·  Science Control Module"
	@echo "     Powered by ROS2 Humble and F\` Flight Software (NASA/JPL)"
	@echo ""
