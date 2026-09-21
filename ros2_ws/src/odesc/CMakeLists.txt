cmake_minimum_required(VERSION 3.8)
project(odesc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# The hardware interface talks raw Linux SocketCAN. The Gazebo backend
# (use_sim:=true) does not depend on this Linux-only package.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  find_package(hardware_interface REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)

  add_library(odesc SHARED
    src/mock_drive_backend.cpp
    src/odesc.cpp
    src/socketcan_drive_backend.cpp
  )
  target_compile_features(odesc PUBLIC cxx_std_17)
  target_include_directories(odesc PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(odesc
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )

  # Export the plugin so ros2_control's pluginlib loader finds OdescSystemHardware.
  pluginlib_export_plugin_description_file(hardware_interface odesc_plugin.xml)

  install(TARGETS odesc
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(DIRECTORY include/
    DESTINATION include
  )
  install(FILES odesc_plugin.xml
    DESTINATION share/${PROJECT_NAME}
  )
  install(PROGRAMS scripts/odesc_vcan_emulator.py
    DESTINATION lib/${PROJECT_NAME}
  )

  ament_export_include_directories(include)
  ament_export_libraries(odesc)
  ament_export_dependencies(
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )
else()
  message(WARNING
    "odesc: SocketCAN hardware interface is Linux-only. "
    "The Gazebo sim backend (use_sim:=true) is unaffected.")
endif()

# node_map.yaml is the canonical node-id<->joint map; useful on every platform.
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
