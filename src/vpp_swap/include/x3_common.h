/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-03-05 15:28:19
 * @LastEditTime: 2023-04-11 14:51:15
 ***************************************************************************/
#ifndef X3_COMMONH_
#define X3_COMMONH_

#include <stdint.h>

#define AUTO_GUARD_MTX_LOCK(mtxlock) \
    std::lock_guard<std::mutex> __guard_lock__##mtxlock(mtxlock)

#define AUTO_GUARD_RECURSIVE_LOCK(mtxlock) \
    std::lock_guard<std::recursive_mutex> __guard_lock__##mtxlock(mtxlock)

#define AUTO_UNIQUE_MTX_LOCK(mtxlock) \
    std::unique_lock<std::mutex> __unique_lock__##mtxlock(mtxlock)

#define AUTO_UNIQUE_RECURSIVE_MTX_LOCK(mtxlock) \
    std::unique_lock<std::recursive_mutex> __unique_lock__##mtxlock(mtxlock)

#define SDK_MAX_PATH_LENGTH 128
#define SDK_MAX_FILES_NUM 128

#define SDK_MAX_FIELD_LENGTH 20
#define SDK_MAX_CMD_LENGTH 128
#define SDK_MAX_RESULT_LENGTH 1024

#define SDK_GET_FRAME_TIMEOUT 3000

typedef enum {
    VPP_CAMERA = 0,
    VPP_ENCODE,
    VPP_DECODE,
    VPP_DISPLAY
} Sdk_Object_e;

typedef struct {
    int32_t width;
    int32_t height;
    int32_t stride;

    int64_t image_id;
    int64_t lost_image_num;
    int64_t exp_time;
    int64_t image_timestamp;
    int32_t plane_count;
    
    uint8_t *data[2];
    uint64_t pdata[2];
    uint32_t data_size[2];
    void *frame_info;
} ImageFrame;

#endif // X3_COMMONH_