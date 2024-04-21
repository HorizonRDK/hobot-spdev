#!/bin/bash
###
 # COPYRIGHT NOTICE
 # Copyright 2024 D-Robotics, Inc.
 # All rights reserved.
 # @Date: 2023-03-05 11:20:19
 # @LastEditTime: 2023-04-20 09:30:35
###
set -e  # Exit immediately if a command exits with a non-zero status
set -u  # Treat unset variables as an error
set -o pipefail  # Causes a pipeline to produce a failure return code if any command errors

usage() {
	echo "usage: bash build.sh [ARCH] [MODE] [BIT]"
	echo "  ARCH: compile architecture (e.g., arm64)"
	echo "  MODE: compile mode (e.g., debug or release)"
	echo "  BIT: compile bit (e.g., 32bit or 64bit)"
	exit 1
}

# Default values
ARCH="arm64"
MODE="release"
BIT="64bit"

# Set default output paths
SYSROOT_DIR="${SYSROOT_DIR:-$(realpath ../../deploy)/rootfs}"
export SYSROOT_DIR
BUILD_OUTPUT_PATH="${BUILD_OUTPUT_PATH:-$(pwd)}"
export BUILD_OUTPUT_PATH

# Constants
SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ALL_PROJECT_DIR="$PWD"
BUILD_DIR="$BUILD_OUTPUT_PATH/build/"
OUTPUT_DIR="$BUILD_OUTPUT_PATH/output/"

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
# Clean build and output directories
function build_clean() {
	rm -rf "${BUILD_DIR}" "${OUTPUT_DIR}"
}

# Parse command line arguments
if [ $# -eq 1 ] && [ "${1}" == "clean" ]; then
	build_clean
	exit
fi

# Check and assign command line arguments
if [ $# -ge 1 ]; then
	ARCH="${1}"
fi

if [ $# -ge 2 ]; then
	MODE="${2}"
fi

if [ $# -ge 3 ]; then
	BIT="${3}"
fi

# Validate ARCH
if [ "${ARCH}" != "arm64" ]; then
	echo "error!!! compile architecture: ${ARCH} is not supported"
	usage
fi

# Validate MODE
if [ "${MODE}" != "release" ] && [ "${MODE}" != "debug" ]; then
	echo "error!!! compile mode: ${MODE} is not supported."
	usage
fi

# Validate BIT
if [ "${BIT}" != "32bit" ] && [ "${BIT}" != "64bit" ]; then
	echo "error!!! compile bit: ${BIT} is not supported."
	usage
fi

function cmake_build() {
	mkdir -p "${BUILD_DIR}"
	mkdir -p "${OUTPUT_DIR}"
	cd "${BUILD_DIR}"
	cmake "${ALL_PROJECT_DIR}" -DCMAKE_BUILD_TYPE="${MODE}"
	make
	make install

	# make hobot_vio whl
	cd "${BUILD_DIR}"
	mkdir -p "${BUILD_DIR}"/python_hobot_vio
	cp -arf "${ALL_PROJECT_DIR}/python/hobot_vio" "${BUILD_DIR}"/python_hobot_vio
	cp -arf "${ALL_PROJECT_DIR}/python/setup_hobot_vio.py" "${BUILD_DIR}"/python_hobot_vio
	cp -arf "${BUILD_DIR}/src/libhbspdev.so" "${BUILD_DIR}/python_hobot_vio/hobot_vio/"
	cp -arf "${BUILD_DIR}/src/libsrcampy.so" "${BUILD_DIR}/python_hobot_vio/hobot_vio/"
	cd "${BUILD_DIR}/python_hobot_vio"
	python3 setup_hobot_vio.py bdist_wheel

	cp "${BUILD_DIR}/python_hobot_vio/dist/"*.whl "${OUTPUT_DIR}"


	# make hobot_vio whl
	mkdir -p "${BUILD_DIR}"/python_hobot_dnn
	cp -arf "${ALL_PROJECT_DIR}/python/hobot_dnn" "${BUILD_DIR}"/python_hobot_dnn
	cp -arf "${ALL_PROJECT_DIR}/python/setup_hobot_dnn.py" "${BUILD_DIR}"/python_hobot_dnn
	cp -arf "${BUILD_DIR}/src/libdnnpy.so" "${BUILD_DIR}/python_hobot_dnn/hobot_dnn/"
	cd "${BUILD_DIR}/python_hobot_dnn"
	python3 setup_hobot_dnn.py bdist_wheel
	cp "${BUILD_DIR}/python_hobot_dnn/dist/"*.whl "${OUTPUT_DIR}"

	mkdir -p "${OUTPUT_DIR}/include"
	cp "${SCRIPTS_DIR}/src/clang/"*.h "${OUTPUT_DIR}/include"
}

# Main execution
build_clean
cmake_build

