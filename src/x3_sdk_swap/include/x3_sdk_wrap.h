/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-03-05 15:28:19
 * @LastEditTime: 2023-03-05 16:31:05
 ***************************************************************************/
/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-01-30 11:27:41
 * @LastEditTime: 2023-03-05 14:15:22
 ***************************************************************************/
#ifndef X3_SDK_WRAP_H_
#define X3_SDK_WRAP_H_

#include <stddef.h>

// SDK 提供的接口都经过这里封装好向上提供
#include "vio/hb_vio_interface.h"

#define X3_MAX_DEV 4

enum {
    TYPE_AI_CAM,
    TYPE_TOF_CAM,
    TYPE_LASER_CAM,
    TYPE_OV10635_YUV422_CAM
};

// 以下两个结构体用来描述模块使用的内存buff信息
typedef struct vp_param {
    uint64_t mmz_paddr[32];
    char *mmz_vaddr[32];
    int mmz_cnt;
    int mmz_size;
} vp_param_t;

typedef struct av_param {
    int count;
    int videoIndex;
    int bufSize;
    int firstPacket;
} av_param_t;

typedef struct x3_vin_info {
    int m_cam_index;
    char m_cam_cfg[256];
    char m_vin_cfg[256];
} x3_vin_info_t;

typedef struct {
    int source_width;
    int source_height;
    int source_stride_y;
    int source_stride_uv;

    int chn_en; // bit0-4: ds0-ds4  bit5:us
    struct {
        int upscale_roi_en;
        int us_roi_start_x;
        int us_roi_start_y;
        int us_roi_width;
        int us_roi_height;
        int upscale_us_en;
        int us_tag_width;
        int us_tag_height;
        int us_buf_num;
    } ipu_us_config;
    struct {
        int ds_roi_en;
        int ds_roi_start_x;
        int ds_roi_start_y;
        int ds_roi_width;
        int ds_roi_height;
        int downscale_ds_en;
        int ds_tag_width;
        int ds_tag_height;
        int ds_buf_num;
    } ipu_ds_config[5];
} x3_ipu_info_t;

typedef struct {
    int m_enable;
    int m_vin_enable;         // 使能标志
    x3_vin_info_t m_vin_info; // 包括 sensor、 mipi、isp、 ldc、dis的配置
    x3_ipu_info_t m_ipu_info;
} x3_modules_info_t;

typedef struct {
    int64_t image_id;
    int64_t lost_image_num;
    int64_t exp_time;
    int64_t image_timestamp;
    int32_t plane_count;
    uint8_t *data[2];
    uint32_t data_size[2];
    void *frame_info;
} ImageFrame;

typedef enum {
    SrPy_Camera,
    SrPy_Encode,
    SrPy_Decode,
    SrPy_Display
} SrPy_Object_e;

#ifdef __cplusplus
extern "C" {
#endif

int vin_param_init(const int video_index, x3_vin_info_t* vin_info);
int x3_reconfig_ipu(int chn_num, int *width, int *height, x3_vin_info_t *vin_info, x3_ipu_info_t *ipu_info);

void print_debug_infos(void);

void x3_normal_buf_info_print(hb_vio_buffer_t *buf);
int x3_dump_nv12(char *filename, char *srcBuf, char *srcBuf1,
                 unsigned int size, unsigned int size1);
int x3_dump_vio_buf_to_nv12(char *filename, hb_vio_buffer_t *vio_buf);
int x3_save_jpeg(char *filename, char *srcBuf, unsigned int size);
int x3_dumpToFile(char *filename, char *srcBuf, unsigned int size);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // X3_SDK_WRAP_H_
