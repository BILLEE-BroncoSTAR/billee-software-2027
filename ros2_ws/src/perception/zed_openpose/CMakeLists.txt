cmake_minimum_required(VERSION 3.10)
project(zed_openpose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# The 3D lift (skeleton_3d.hpp) has no ZED/OpenPose dependency: test it on every machine.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_skeleton_3d test/test_skeleton_3d.cpp)
  target_include_directories(test_skeleton_3d PRIVATE include)
  target_compile_features(test_skeleton_3d PRIVATE cxx_std_17)
endif()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# The node runs only on the rover: docker/Dockerfile.l4t-humble installs the ZED SDK
# (/usr/local/zed) and CMU OpenPose built with CUDA (/opt/openpose). On the ground station and
# the Mac this package builds nothing else.
find_path(OPENPOSE_INCLUDE_DIR openpose/headers.hpp PATHS /opt/openpose/include NO_DEFAULT_PATH)
find_library(OPENPOSE_LIB openpose PATHS /opt/openpose/lib NO_DEFAULT_PATH)
find_library(CAFFE_LIB caffe PATHS /opt/openpose/lib NO_DEFAULT_PATH)
if(NOT EXISTS /usr/local/zed/zed-config.cmake OR NOT OPENPOSE_INCLUDE_DIR OR NOT OPENPOSE_LIB
    OR NOT CAFFE_LIB)
  message(WARNING "zed_openpose: no ZED SDK (/usr/local/zed) or OpenPose (/opt/openpose); "
    "skipping the node. Expected everywhere except the Jetson (l4t) image.")
  ament_package()
  return()
endif()

find_package(ZED REQUIRED PATHS /usr/local/zed NO_DEFAULT_PATH)
find_package(CUDA REQUIRED)
# The image's system OpenCV, which OpenPose was built against - never the pixi env's own OpenCV.
# In the l4t image that is Ubuntu's 4.5.4 from the Isaac ROS base, in /usr/lib/<arch>/cmake/opencv4.
# That directory is named explicitly: pixi's conda compiler reports no multiarch directory, so a
# plain /usr prefix only searches /usr/lib/cmake/opencv4 (where NVIDIA's Jetson-repo build goes).
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs
  PATHS /usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/cmake/opencv4 /usr NO_DEFAULT_PATH)
find_package(rclcpp REQUIRED)
find_package(people_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(zed_openpose_node src/zed_openpose_node.cpp)
target_compile_features(zed_openpose_node PUBLIC cxx_std_17)
target_include_directories(zed_openpose_node PRIVATE
  include ${OPENPOSE_INCLUDE_DIR} ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(zed_openpose_node
  rclcpp people_msgs geometry_msgs sensor_msgs visualization_msgs tf2 tf2_ros)
# OpenPose links OpenCV's highgui, which drags in the system GTK + GLib, while the pixi env has
# its own, newer GLib: mixing the two aborts at load ("libgio-2.0.so.0: undefined symbol
# g_variant_builder_..."). Link the env's GLib first and keep it in NEEDED, so it loads before
# anything else asks for GLib and the whole process shares that one (backward-compatible) copy.
find_library(GLIB_LIB glib-2.0 PATHS $ENV{CONDA_PREFIX}/lib NO_DEFAULT_PATH REQUIRED)
find_library(GOBJECT_LIB gobject-2.0 PATHS $ENV{CONDA_PREFIX}/lib NO_DEFAULT_PATH REQUIRED)
find_library(GMODULE_LIB gmodule-2.0 PATHS $ENV{CONDA_PREFIX}/lib NO_DEFAULT_PATH REQUIRED)
find_library(GIO_LIB gio-2.0 PATHS $ENV{CONDA_PREFIX}/lib NO_DEFAULT_PATH REQUIRED)
target_link_libraries(zed_openpose_node
  -Wl,--no-as-needed ${GLIB_LIB} ${GOBJECT_LIB} ${GMODULE_LIB} ${GIO_LIB} -Wl,--as-needed
  ${OPENPOSE_LIB} ${CAFFE_LIB} ${ZED_LIBRARIES} ${OpenCV_LIBS} ${CUDA_CUDART_LIBRARY})
# pixi's conda linker uses an older glibc sysroot than the system libraries linked above; let
# their symbols resolve at run time (the x86 build task in pixi.toml does the same).
target_link_options(zed_openpose_node PRIVATE "-Wl,--allow-shlib-undefined")

install(TARGETS zed_openpose_node DESTINATION lib/${PROJECT_NAME})

ament_package()
