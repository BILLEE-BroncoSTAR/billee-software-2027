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
