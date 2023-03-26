/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-02-23 13:47:14
 * @LastEditTime: 2023-03-05 15:55:29
 ***************************************************************************/
#include <stdio.h>
#include <sys/stat.h>

#include "utils_log.h"
#include "logging.h"
#include "x3_vio_vin.h"
#include "x3_vio_vps.h"
#include "x3_sdk_wrap.h"

#include "vio/hb_vio_interface.h"
#include "cam/hb_cam_interface.h"

/* 绑定关系：
 * mipi(vc) -> dev(sif) -> isp(pipeline)
 */
int x3_vin_init(x3_vin_info_t *vin_info)
{
    int ret = 0;

    ret = hb_vio_init(vin_info->m_vin_cfg);
    if (ret < 0)
    {
        LOGD_print("hb_vio_init fail\n");
        return -1;
    }
    ret = hb_cam_init(0, vin_info->m_cam_cfg);
    if (ret < 0)
    {
        LOGD_print("hb_cam_init fail\n");
        hb_vio_deinit();
        return -1;
    }
    return ret;
}

int x3_vin_start(x3_vin_info_t *vin_info)
{
    int ret = 0;

    ret = hb_vio_start_pipeline(0);
    if (ret < 0)
    {
        printf("vio start fail, do cam&vio&display deinit.\n");
        return -1;
    }
    ret = hb_cam_start(0);
    if (ret < 0)
    {
        printf("cam start fail, do cam&vio&display deinit.\n");
    }
    LOGD_print("ok!\n");
    return ret;
}

void x3_vin_stop(x3_vin_info_t *vin_info)
{
    hb_cam_stop(0);
    hb_vio_stop_pipeline(0);
    LOGD_print("ok!\n");
}

void x3_vin_deinit(x3_vin_info_t *vin_info)
{
    hb_cam_deinit(0);
    hb_vio_deinit();
    LOGD_print("ok!");
}

int time_cost_ms(struct timeval *start, struct timeval *end)
{
    int time_ms = -1;
    time_ms = ((end->tv_sec * 1000 + end->tv_usec / 1000) -
               (start->tv_sec * 1000 + start->tv_usec / 1000));
    printf("time cost %d ms \n", time_ms);
    return time_ms;
}

int x3_vin_sif_get_data(const int pipe_id, hb_vio_buffer_t *sif_raw, int timeout)
{
    int ret = 0, size = 0;

    ret = hb_vio_get_data(pipe_id, HB_VIO_SIF_RAW_DATA, sif_raw);

    if (ret) {
        LOGE_print("hb_vio_get_data get raw error, ret=%d\n", ret);
    } else {
        /*x3_normal_buf_info_print(sif_raw);*/

        size = sif_raw->img_addr.stride_size * sif_raw->img_addr.height;
        if (size == 0) {// 有可能是因为模式不是offline的，所以获取不到raw图
            hb_vio_free_sifbuf(pipe_id, sif_raw);
            return -1;
        }
    }
    return ret;
}

int x3_vin_sif_release_data(const int pipe_id, hb_vio_buffer_t *sif_raw)
{
    int ret = 0;
    ret = hb_vio_free_sifbuf(pipe_id, sif_raw);
    if (ret) {
        LOGE_print("hb_vio_free_sifbuf free raw error, ret=%d\n", ret);
        return -1;
    }
    return ret;
}

int x3_vin_isp_get_data(const int pipe_id, hb_vio_buffer_t *isp_yuv, int timeout)
{
   int ret = 0, size = 0;

    ret = hb_vio_get_data(pipe_id, HB_VIO_ISP_YUV_DATA, isp_yuv);
    if (ret) {
        LOGE_print("hb_vio_get_data get yuv error, ret=%d\n", ret);
    } else {
        /*x3_normal_buf_info_print(isp_yuv);*/

        size = isp_yuv->img_addr.stride_size * isp_yuv->img_addr.height;
        if (size == 0) { // 有可能是因为模式不是offline的，所以获取不到yuv图
            hb_vio_free_ispbuf(pipe_id, isp_yuv);
            return -1;
        }
    }
    return ret;
}

int x3_vin_isp_release_data(const int pipe_id, hb_vio_buffer_t *isp_yuv)
{
    int ret = 0;
    ret = hb_vio_free_ispbuf(pipe_id, isp_yuv);
    if (ret) {
        LOGE_print("hb_vio_free_ispbuf free yuv error, ret=%d\n", ret);
        return -1;
    }
    return ret;
}

static int save_jpeg(char *filename, char *srcBuf, unsigned int size)
{
    FILE *fd = NULL;

    fd = fopen(filename, "w+");

    if (fd == NULL)
    {
        printf("open(%s) fail", filename);
        return -1;
    }

    fflush(stdout);

    fwrite(srcBuf, 1, size, fd);

    fflush(fd);

    if (fd)
        fclose(fd);

    printf("DEBUG:save jpeg(%s, size(%d) is successed!!\n", filename, size);

    return 0;
}
