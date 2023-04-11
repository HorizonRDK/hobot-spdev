#!/bin/bash
###
 # COPYRIGHT NOTICE
 # Copyright 2023 Horizon Robotics, Inc.
 # All rights reserved.
 # @Date: 2023-03-05 11:20:19
 # @LastEditTime: 2023-04-11 15:35:36
### 

usage() {
  echo "usage: bash make.sh [x3] [debug|release]"
  exit 1
}

[ -z "${SYSROOT_DIR}" ] &&  export SYSROOT_DIR="$(realpath ../../deploy)/rootfs"
[ -z "${BUILD_OUTPUT_PATH}" ] && export BUILD_OUTPUT_PATH="$(pwd)"

#set -x
SCRIPTS_DIR=$(cd $(dirname $0) && pwd)
ALL_PROJECT_DIR=$PWD
BUILD_DIR=$BUILD_OUTPUT_PATH/build/
OUTPUT_DIR=$BUILD_OUTPUT_PATH/output/

ARCH="arm64"
MODE="release"
BIT="64bit"

function build_clean() {
  rm -rf ${BUILD_DIR} ${OUTPUT_DIR}
}

if [ $# -eq 1 ]; then
  HOBOT_COMPILE_MODE=${1}
  if [ x"${HOBOT_COMPILE_MODE}" == x"clean" ]; then
    build_clean
  fi
  exit
fi

#check params
# 1.check compile arch
if [ $# -ge 1 ]; then
  ARCH=${1}
  if [ x"${ARCH}" != x"arm64" ]; then
    echo "error!!! compile architecture:${ARCH} is not supported"
    usage
  fi
else
  echo "use default ARCH:${ARCH}"
fi

# 2.check compile mode
if [ $# -ge 2 ]; then
  MODE=${2}
  if [ ${MODE} == "release" -o ${MODE} == "debug" ]; then
    echo ""
  else
    echo "error!!! compile mode:${MODE} is not supported."
    usage
  fi
else
  echo "use default MODE:${MODE}"
fi

# 3.check compile bit
if [ $# -ge 3 ]; then
  BIT=${2}
  if [ ${BIT} == "32bit" -o ${BIT} == "64bit" ]; then
    echo ""
  else
    echo "error!!! compile bit:${BIT} is not supported."
    usage
  fi
else
  echo "use default BIT:${BIT}"
fi

function cmake_build() {
  mkdir -p ${BUILD_DIR}
  mkdir -p ${OUTPUT_DIR}
  cd ${BUILD_DIR}
  cmake ${ALL_PROJECT_DIR} $*
  echo "##############################################"
  echo $1
  make -j${N} VERBOSE=1
  if [ $? -ne 0 ]; then
    echo "failed to build"
    exit 1
  fi
  # make copy
  make install

  # make whl
  cd ${BUILD_DIR}
  cp -arf ${ALL_PROJECT_DIR}/python ${BUILD_DIR}
  cp -arf ${BUILD_DIR}/src/libhbspdev.so ${BUILD_DIR}/python/hobot_vio/
  cp -arf ${BUILD_DIR}/src/libsrcampy.so ${BUILD_DIR}/python/hobot_vio/
  cd ${BUILD_DIR}/python
  python3 setup.py bdist_wheel

  cp ${BUILD_DIR}/python/dist/*.whl ${OUTPUT_DIR}

  mkdir -p ${OUTPUT_DIR}/include
  cp ${SCRIPTS_DIR}/src/clang/*.h ${OUTPUT_DIR}/include

  cd ${ALL_PROJECT_DIR}
}

build_clean
cmake_build
