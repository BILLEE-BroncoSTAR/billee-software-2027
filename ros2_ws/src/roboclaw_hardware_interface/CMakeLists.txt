cmake_minimum_required(VERSION 3.8)
project(roboclaw_hardware_interface)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(roboclaw_serial REQUIRED)

pluginlib_export_plugin_description_file(hardware_interface roboclaw_hardware_interface.xml)

# Include directories
include_directories(
  include
)

# Add roboclaw hardware interface shared library
add_library(${PROJECT_NAME} SHARED
  src/roboclaw_hardware_interface.cpp
  src/limit_switch.cpp
  src/roboclaw_unit.cpp
  src/motor_joint.cpp
)

# JetsonGPIO is vendored at the repository root rather than in ros2_ws/src:
# it is a CMake library, not an ament package.  Do not install its udev rules
# during a colcon install; deployment performs that one privileged setup step.
set(BUILD_EXAMPLES OFF CACHE BOOL "Build JetsonGPIO examples" FORCE)
set(JETSON_GPIO_POST_INSTALL OFF CACHE BOOL "Install JetsonGPIO udev permissions" FORCE)
# Do not let a colcon install write into /lib/udev. Keep the rule with this
# package; deployment can copy it to the system deliberately on a Jetson.
set(UDEVRULES_INSTALL_DIR share/JetsonGPIO/udev CACHE STRING "JetsonGPIO udev rule install dir" FORCE)
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/JetsonGPIO
  ${CMAKE_CURRENT_BINARY_DIR}/jetson_gpio
  EXCLUDE_FROM_ALL
)
# JetsonGPIO's GPIOEvent private header uses uint64_t without including
# <cstdint>. Keep the vendored submodule pristine while making that upstream
# omission explicit for GCC and Clang builds.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(JetsonGPIO PRIVATE -include cstdint)
endif()

target_link_libraries(${PROJECT_NAME}
  roboclaw_serial
  JetsonGPIO::JetsonGPIO
)

# Add library dependencies
ament_target_dependencies(${PROJECT_NAME}
  pluginlib hardware_interface
  roboclaw_serial
)

# Install the shared library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the plugin description xml file
install(FILES
  roboclaw_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install and export the headers so other packages (pxx uses LimitSwitch)
# can include them and link this library.
install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
