cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_COMPILER $ENV{CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER $ENV{CROSS_COMPILE}g++)

project(hobot_spdev)

set(SOC "X3")
add_definitions(-D${SOC}="${SOC}")

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
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cameras/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vpp_swap/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vio
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hbmedia
    ${CMAKE_CURRENT_SOURCE_DIR}/include/libmm
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cam
    ${CMAKE_CURRENT_SOURCE_DIR}/include/iar
    ${CMAKE_CURRENT_SOURCE_DIR}/include/linux
    ${CMAKE_CURRENT_SOURCE_DIR}/include/isp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hbusb
    ${CMAKE_CURRENT_SOURCE_DIR}/hobot-dnn/usr/include
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/hbbpu
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/hbmedia
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64-linux-gnu
  ${CMAKE_CURRENT_SOURCE_DIR}/hobot-dnn/usr/lib/hbbpu
)

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${X3_EXTERNAL_LIB}:${X3_HB_MEDIA_LIB}:${X3_FFMPEG_LIB}:${X3_PYTHON_LIB}")

set(OUTPUT_ROOT ${CMAKE_SOURCE_DIR}/output/${PROJECT_NAME})

# link library
set(BASE_LIBRARIES pthread dl rt)

if ("${SOC}" STREQUAL "X3")
    set(HB_SDK_LIB hbmedia multimedia avutil avformat vio gdcbin isp isp_algo iar cam tinyalsa alog python3.8)
endif()

# add spdev include path
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vp_swap/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/python/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/python/include/python3.6m
    ${CMAKE_CURRENT_SOURCE_DIR}/src/clang
)


install(DIRECTORY ${MY_BUILD_ROOT}/src/libhbspdev.so
	DESTINATION ${MY_OUTPUT_ROOT}/)

add_subdirectory(src)
