# billee-software-2027 — ground-station shortcuts for the BILLEE dashboard
#
#   make              same as `make help`
#   make setup-dash   install and configure the dashboard on the ground station (x86 host)
#   make start        start the dashboard: http://localhost:7780 + the RCM GDS
#   make start ARGS=--no-browser    pass options to the dashboard (./BILLEE-dashboard-tool/billee-dash --help)
#   make test-dash    run the dashboard's tests
#
# Run on the ground station HOST, not in a dev container: the dashboard starts the ground
# station's x86 dev container itself and drives the rover's l4t container over SSH.
# The dashboard is the `BILLEE-dashboard-tool/` submodule (github.com/LucaLanzi/BILLEE-dashboard-tool).

DASH_DIR := BILLEE-dashboard-tool
ARGS     ?=

.DEFAULT_GOAL := help
.PHONY: help banner setup-dash start test-dash dash-submodule

help: ## Show this list of targets
	@$(MAKE) --no-print-directory banner
	@echo "Targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
	  | awk 'BEGIN {FS = ":.*?## "} {printf "  \033[36m%-11s\033[0m %s\n", $$1, $$2}'
	@echo
	@echo "  make start ARGS=--no-browser   pass options to the dashboard"

dash-submodule:
	@[ -f $(DASH_DIR)/billee-dash ] || git submodule update --init $(DASH_DIR)

setup-dash: dash-submodule ## Install the dashboard's dependencies, SSH key to the rover, config
	@bash $(DASH_DIR)/scripts/setup.sh
	@$(MAKE) --no-print-directory banner

start: dash-submodule ## Start the dashboard (opens it and the RCM GDS in the browser)
	@exec $(DASH_DIR)/billee-dash $(ARGS)

test-dash: dash-submodule ## Run the dashboard's tests
	@cd $(DASH_DIR) && python3 -m unittest discover -s tests

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
