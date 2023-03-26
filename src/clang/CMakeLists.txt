cmake_minimum_required(VERSION 2.8)

# projetc name
project(hobot_spdev)

# make libhbspdev.so
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/include
    ${CMAKE_CURRENT_SOURCE_DIR}/x3_sdk_swap/include
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_utils/include
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

# 头文件路径信息
file(GLOB PROJECT_INC
    "utils/include/*.h"
    "x3_sdk_swap/include/*.h"
    "python/include/python3.6m/*.h"
    "python/*.h"
    "sensor_utils/include/*.h"
    "sdk_interface/*.h"
    )

# 源文件路径信息
file(GLOB PROJECT_SRC
    "sdk_interface/*.cpp"
    "python/*.cpp"
    "x3_sdk_swap/src/*.c"
    "utils/src/*.c"
    "sensor_utils/src/*.c"
    )

set(SOURCE_FILES ${PROJECT_SRC} ${PROJECT_INC})
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

set(BASE_LIBRARIES pthread dl rt)
set(X3_SDK_LIB multimedia avutil avformat vio gdcbin isp isp_algo iar cam tinyalsa alog python3.8)
target_link_libraries(${PROJECT_NAME} ${BASE_LIBRARIES} ${X3_SDK_LIB})

install(TARGETS ${PROJECT_NAME} DESTINATION ${MY_OUTPUT_ROOT})
