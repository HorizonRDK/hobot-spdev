cmake_minimum_required(VERSION 2.8)

# projetc name
project(hobot_spdev)

# make libhbspdev.so
set(HBSPDEV_NAME hbspdev)
file(GLOB HBSPDEV_INC
    "utils/include/*.h"
    "cameras/include/*.h"
    "vpp_swap/include/*.h"
    )

file(GLOB HBSPDEV_SRC
    "vpp_swap/src/*.cpp"
    "vpp_swap/src/*.c"
    "utils/src/*.c"
    "cameras/src/*.c"
    )

set(SOURCE_FILES ${HBSPDEV_SRC} ${HBSPDEV_INC})
add_library(${HBSPDEV_NAME} SHARED ${SOURCE_FILES})

target_link_libraries(${HBSPDEV_NAME} ${BASE_LIBRARIES} ${HB_SDK_LIB})

# make libsrcampy.so
set(SRCAMPY_NAME srcampy)
file(GLOB SRCAMPY_INC
    "python/include/python3.6m/*.h"
    "python/*.h"
    )

# 源文件路径信息
file(GLOB SRCAMPY_SRC
    "python/*.cpp"
    )

set(SOURCE_FILES ${SRCAMPY_SRC} ${SRCAMPY_INC})
add_library(${SRCAMPY_NAME} SHARED ${SOURCE_FILES})

target_link_libraries(${SRCAMPY_NAME} ${BASE_LIBRARIES} ${HB_SDK_LIB} ${HBSPDEV_NAME})

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

target_link_libraries(${SPCDEV_NAME} ${BASE_LIBRARIES} ${HB_SDK_LIB} dnn ${HBSPDEV_NAME})

install(TARGETS ${HBSPDEV_NAME} DESTINATION ${MY_OUTPUT_ROOT})
install(TARGETS ${SRCAMPY_NAME} DESTINATION ${MY_OUTPUT_ROOT})
install(TARGETS ${SPCDEV_NAME} DESTINATION ${MY_OUTPUT_ROOT})
