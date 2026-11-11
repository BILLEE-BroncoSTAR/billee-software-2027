cmake_minimum_required(VERSION 3.8)
project(arm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/arm_slow_jog.py
  scripts/pxx_probe.py
  scripts/roboclaw_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
