#!/bin/sh
# Clone, patch and build CMU OpenPose (CUDA + cuDNN) into /opt/openpose, for zed_openpose.
# Run by docker/Dockerfile.l4t-humble; also runnable by hand inside that image to iterate:
#
#   sh docker/build_openpose.sh /path/to/src   # re-running reuses the clone and the build dir
#
# OPENPOSE_JOBS (env) = parallel compile jobs, default all cores.
set -eu

SRC=${1:-/tmp/openpose}
OPENPOSE_COMMIT=5c5d96523ef917bd30301245fdc8343937cae48d

if [ ! -d "$SRC/.git" ]; then
  git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git "$SRC"
  cd "$SRC"
  git checkout "$OPENPOSE_COMMIT"
  git submodule update --init 3rdparty/caffe
  # OpenPose's CMake later checks Caffe out at 1807aad (its CUDA 11 / cuDNN 8 / Ampere commit),
  # keeping these working-tree edits. Patches for current CUDA / protobuf (needed on CUDA 12.2 /
  # JetPack 6, which the image targets, and on CUDA 13 / JetPack 7):
  #  - C++17 everywhere: CUDA 12's Thrust needs C++14 or newer, CUDA 13's needs C++17
  #  - cudaDeviceProp fields deprecated in CUDA 12 and removed in 13 are dropped from Caffe's
  #    device report
  #  - protobuf's two-argument SetTotalBytesLimit -> the one-argument form
  sed -i -e 's/-std=c++11/-std=c++17/' -e 's/-std=c++14/-std=c++17/' CMakeLists.txt
  sed -i 's/-std=c++11/-std=c++17/' 3rdparty/caffe/CMakeLists.txt
  sed -i -e 's/prop\.clockRate/0/' -e 's/prop\.deviceOverlap/false/' \
    -e 's/prop\.kernelExecTimeoutEnabled/false/' 3rdparty/caffe/src/caffe/common.cpp
  sed -i 's/SetTotalBytesLimit(kProtoReadBytesLimit, 536870912)/SetTotalBytesLimit(kProtoReadBytesLimit)/' \
    3rdparty/caffe/src/caffe/util/io.cpp
fi

# CUDA 13 moved Thrust/CUB under include/cccl: nvcc finds them, but Caffe's .cpp files that
# include <thrust/...> are compiled by plain g++. (CUDA 12 has them in include/: no-op there.)
export CPATH="/usr/local/cuda/include/cccl${CPATH:+:$CPATH}"

# The Isaac ROS base installs protobuf 26 (+ abseil) into /usr/local, which shadows Ubuntu's
# protobuf 3.12 that this Caffe needs: OpenPose's and Caffe's FindProtobuf would pick
# /usr/local's library and protoc, and g++ searches /usr/local/include before /usr/include.
# /usr first on CMAKE_PREFIX_PATH makes CMake find Ubuntu's library and protoc; a directory with
# only Ubuntu's google/ headers on CPATH is searched before every system include directory.
# Nothing is recorded in the installed libraries. No-op on bases without /usr/local's protobuf.
if [ -e /usr/local/include/google/protobuf ]; then
  mkdir -p "$SRC/protobuf-system-include"
  ln -sfn /usr/include/google "$SRC/protobuf-system-include/google"
  export CPATH="$SRC/protobuf-system-include:$CPATH"
  export CMAKE_PREFIX_PATH="/usr${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
fi

# The Isaac ROS base also builds Boost 1.80 into /usr: its headers replace Ubuntu's 1.74 ones in
# /usr/include/boost, but CMake finds Ubuntu's 1.74 config in /usr/lib/<arch>/cmake first, so
# Caffe would compile against 1.80 and link 1.74 (undefined boost::filesystem::path::*_v3).
# Boost_DIR (read from the environment by every find_package(Boost), Caffe's included) selects
# the config of the Boost whose headers are installed. No-op where that is Ubuntu's own.
boost=$(sed -n 's/^#define BOOST_LIB_VERSION "\([0-9]*\)_\([0-9]*\)"/\1.\2/p' \
  /usr/include/boost/version.hpp)
if [ -f "/usr/lib/cmake/Boost-$boost.0/BoostConfig.cmake" ]; then
  export Boost_DIR="/usr/lib/cmake/Boost-$boost.0"
fi

mkdir -p "$SRC/build"
cd "$SRC/build"
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/openpose \
  -DGPU_MODE=CUDA -DUSE_CUDNN=ON \
  -DCUDA_ARCH=Manual -DCUDA_ARCH_BIN=87 -DCUDA_ARCH_PTX=87 \
  -DBUILD_EXAMPLES=OFF -DBUILD_PYTHON=OFF -DBUILD_DOCS=OFF -DBUILD_UNITY_SUPPORT=OFF \
  -DDOWNLOAD_BODY_25_MODEL=OFF -DDOWNLOAD_BODY_COCO_MODEL=OFF -DDOWNLOAD_BODY_MPI_MODEL=OFF \
  -DDOWNLOAD_FACE_MODEL=OFF -DDOWNLOAD_HAND_MODEL=OFF
make -j"${OPENPOSE_JOBS:-$(nproc)}"
make install
