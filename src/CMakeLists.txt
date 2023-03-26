cmake_minimum_required(VERSION 2.8)

# projetc name
project(hobot_spdev)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/include
    ${CMAKE_CURRENT_SOURCE_DIR}/x3_sdk_swap/include
    ${CMAKE_CURRENT_SOURCE_DIR}/sdk_interface
    ${CMAKE_CURRENT_SOURCE_DIR}/python
    ${CMAKE_CURRENT_SOURCE_DIR}/python/include
    ${CMAKE_CURRENT_SOURCE_DIR}/python/include/python3.6m
    )


# 添加头文件路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/include
    ${CMAKE_CURRENT_SOURCE_DIR}/x3_sdk_swap/include
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_utils/include
    ${CMAKE_CURRENT_SOURCE_DIR}/sdk_interface
    ${CMAKE_CURRENT_SOURCE_DIR}/python
    ${CMAKE_CURRENT_SOURCE_DIR}/python/include
    ${CMAKE_CURRENT_SOURCE_DIR}/python/include/python3.6m
    )

# make libhbspdev.so
set(HBSPDEV_NAME hbspdev)
file(GLOB HBSPDEV_INC
    "utils/include/*.h"
    "x3_sdk_swap/include/*.h"
    "sdk_interface/*.h"
    )

# 源文件路径信息
file(GLOB HBSPDEV_SRC
    "sdk_interface/*.cpp"
    "x3_sdk_swap/src/*.c"
    "utils/src/*.c"
    )

set(SOURCE_FILES ${HBSPDEV_SRC} ${HBSPDEV_INC})
add_library(${HBSPDEV_NAME} SHARED ${SOURCE_FILES})

set(BASE_LIBRARIES pthread dl rt)
set(HB_SDK_LIB multimedia avutil avformat vio gdcbin isp isp_algo iar cam alog python3.8)
target_link_libraries(${HBSPDEV_NAME} ${BASE_LIBRARIES} ${HB_SDK_LIB})

# make libsppydev.so
set(SPPYDEV_NAME sppydev)
file(GLOB SPPYDEV_INC
    "python/include/python3.6m/*.h"
    "python/*.h"
    )

# 源文件路径信息
file(GLOB SPPYDEV_SRC
    "python/*.cpp"
    )

set(SOURCE_FILES ${SPPYDEV_SRC} ${SPPYDEV_INC})
add_library(${SPPYDEV_NAME} SHARED ${SOURCE_FILES})

set(BASE_LIBRARIES pthread dl rt)
set(HB_SDK_LIB ${HBSPDEV_NAME} multimedia avutil avformat vio gdcbin isp isp_algo iar cam alog python3.8)
target_link_libraries(${SPPYDEV_NAME} ${BASE_LIBRARIES} ${HB_SDK_LIB})

# make libspcdev.so
set(SPCDEV_NAME spcdev)
file(GLOB SPCDEV_INC
    "clang/*.h"
    )

# 源文件路径信息
file(GLOB SPCDEV_SRC
    "clang/*.cpp"
    )

set(SOURCE_FILES ${SPCDEV_SRC} ${SPCDEV_INC})
add_library(${SPCDEV_NAME} SHARED ${SOURCE_FILES})

set(BASE_LIBRARIES pthread dl rt)
set(HB_SDK_LIB ${HBSPDEV_NAME} multimedia avutil avformat vio gdcbin isp isp_algo iar cam alog python3.8)
target_link_libraries(${SPCDEV_NAME} ${BASE_LIBRARIES} ${HB_SDK_LIB})

install(TARGETS ${HBSPDEV_NAME} DESTINATION ${MY_OUTPUT_ROOT})
install(TARGETS ${SPPYDEV_NAME} DESTINATION ${MY_OUTPUT_ROOT})
install(TARGETS ${SPCDEV_NAME} DESTINATION ${MY_OUTPUT_ROOT})
