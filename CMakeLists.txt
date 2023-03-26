cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_COMPILER $ENV{CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER $ENV{CROSS_COMPILE}g++)

project(hobot_spdev)

option(RELEASE_LIB "build version of release" ON)
message("config types: ${CMAKE_CONFIGURATION_TYPES}")

if (${RELEASE_LIB})
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "BUILD TYPE:Release")
else ()
  set(CMAKE_BUILD_TYPE Debug)
  set(HOBOT_BUILD_DEBUG "TRUE")
  message(STATUS "BUILD TYPE:Debug")
endif ()

list(APPEND CMAKE_C_FLAGS "-Wall -Wno-unknown-pragmas -fPIC -fexceptions ")
list(APPEND CMAKE_C_FLAGS "--sysroot=$ENV{SYSROOT_DIR} ")
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  list(APPEND CMAKE_C_FLAGS "-march=armv8-a -mcpu=cortex-a53 -O3 ")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC ")
else ()
  list(APPEND CMAKE_C_FLAGS "-march=armv8-a -mcpu=cortex-a53 -g -O0 -fPIC ")
endif()

list(APPEND CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++14 -fno-permissive ")
string(REGEX REPLACE ";" " " CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
string(REGEX REPLACE ";" " " CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
set(MY_OUTPUT_ROOT $ENV{BUILD_OUTPUT_PATH}/output)
set(MY_BUILD_ROOT $ENV{BUILD_OUTPUT_PATH}/build)

include_directories(
    $ENV{SRC_DIR}/include
    $ENV{SRC_DIR}/include/vio
    $ENV{SRC_DIR}/include/hbmedia
    $ENV{SRC_DIR}/include/libmm
    $ENV{SRC_DIR}/include/cam
    $ENV{SRC_DIR}/include/iar
    $ENV{SRC_DIR}/include/linux
    $ENV{SRC_DIR}/include/isp
    $ENV{SRC_DIR}/include/hbusb
    $ENV{SRC_DIR}/hobot-dnn/usr/include
    $ENV{SYSROOT_DIR}/usr/include/aarch64-linux-gnu
)

link_directories(
  $ENV{SRC_DIR}/lib
  $ENV{SRC_DIR}/lib/hbbpu
  $ENV{SRC_DIR}/lib/hbmedia
  $ENV{SRC_DIR}/hobot-dnn/usr/lib
  $ENV{SYSROOT_DIR}/usr/lib/aarch64-linux-gnu
  $ENV{SYSROOT_DIR}/usr/lib/python3.8/config-3.8-aarch64-linux-gnu/
)

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
set(CMAKE_INSTALL_RPATH "${X3_EXTERNAL_LIB}:${X3_HB_MEDIA_LIB}:${X3_FFMPEG_LIB}:${X3_PYTHON_LIB}")

set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/${PROJECT_NAME})

install(DIRECTORY ${MY_BUILD_ROOT}/src/libhbspdev.so
	DESTINATION ${MY_OUTPUT_ROOT}/)

add_subdirectory(src)
