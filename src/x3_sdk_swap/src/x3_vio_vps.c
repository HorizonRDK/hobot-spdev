/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-02-23 14:01:59
 * @LastEditTime: 2023-03-05 15:57:48
 ***************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "utils_log.h"
#include "logging.h"
#include "x3_vio_vin.h"
#include "x3_vio_vps.h"
#include "x3_sdk_wrap.h"

#include "vio/hb_vio_interface.h"

int x3_ipu_chn_to_type(int chn)
{
    switch (chn)
    {
    case 0:
        return HB_VIO_IPU_DS0_DATA;
    case 1:
        return HB_VIO_IPU_DS1_DATA;
    case 2:
        return HB_VIO_IPU_DS2_DATA;
    case 3:
        return HB_VIO_IPU_DS3_DATA;
    case 4:
        return HB_VIO_IPU_DS4_DATA;
    case 5:
        return HB_VIO_IPU_US_DATA;
    default:
        return HB_VIO_IPU_DS2_DATA;
    }
}

int x3_vps_select_chn(int *chn_en, int src_width, int src_height, int dst_width, int dst_height)
{
    if (((dst_width == src_width) || (dst_height == src_height)) &&
        (!(*chn_en & 1 << HB_VIO_IPU_DS2_DATA)) &&
        (dst_width <= 4096 && dst_height <= 4096))
    {
        return HB_VIO_IPU_DS2_DATA;
    }
    if ((dst_width <= src_width) || (dst_height <= src_height))
    {
        if ((dst_width <= 1920 && dst_height <= 1080) &&
            (!(*chn_en & 1 << HB_VIO_IPU_DS1_DATA)))
        {
            return HB_VIO_IPU_DS1_DATA;
        }
        else if ((dst_width <= 1920 && dst_height <= 1080) &&
                 (!(*chn_en & 1 << HB_VIO_IPU_DS3_DATA)))
        {
            return HB_VIO_IPU_DS3_DATA;
        }
        else if ((dst_width <= 1280 && dst_height <= 720) &&
                 (!(*chn_en & 1 << HB_VIO_IPU_DS0_DATA)))
        {
            return HB_VIO_IPU_DS0_DATA;
        }
        else if ((dst_width <= 1280 && dst_height <= 720) &&
                 (!(*chn_en & 1 << HB_VIO_IPU_DS4_DATA)))
        {
            return HB_VIO_IPU_DS4_DATA;
        }
    }
    if (((dst_width >= src_width) || (dst_height > src_height)) &&
        (!(*chn_en & 1 << HB_VIO_IPU_US_DATA)) &&
        (dst_width <= 4096 && dst_height <= 4096))
    {
        return HB_VIO_IPU_US_DATA;
    }

    return -1;
}

int x3_vps_start(uint32_t pipeline_id)
{
    int ret = 0;
    return ret;
}

void x3_vps_stop(int pipeline_id)
{
    LOGD_print("ok!\n");
}

void x3_vps_deinit(int pipeline_id)
{
    LOGD_print("ok!");
}

int x3_vps_input(uint32_t pipeline_id, hb_vio_buffer_t *buffer)
{
    int ret = 0;
    return ret;
}

int x3_vps_get_output(uint32_t pipeline_id, int channel, hb_vio_buffer_t *buffer,
                      const int timeout)
{
    int ret = 0, size = 0;

    ret = hb_vio_get_data(pipeline_id, x3_ipu_chn_to_type(channel), buffer);
    if (ret)
    {
        LOGE_print("hb_vio_get_data get ipu yuv error, ret=%d\n", ret);
    }
    else
    {
        /*x3_normal_buf_info_print(isp_yuv);*/

        size = buffer->img_addr.stride_size * buffer->img_addr.height;
        if (size == 0)
        { // 有可能是因为模式不是offline的，所以获取不到yuv图
            hb_vio_free_ipubuf(pipeline_id, buffer);
            return -1;
        }
    }
    return ret;
}

int x3_vps_get_output_cond(uint32_t pipeline_id, int channel,
                           hb_vio_buffer_t *buffer, const int timeout)
{
    int ret = 0, size = 0;

    ret = hb_vio_get_data_conditional(pipeline_id, x3_ipu_chn_to_type(channel), buffer, timeout);
    if (ret)
    {
        LOGE_print("hb_vio_get_data_conditional get ipu yuv error, ret=%d\n", ret);
    }
    else
    {
        /*x3_normal_buf_info_print(isp_yuv);*/

        size = buffer->img_addr.stride_size * buffer->img_addr.height;
        if (size == 0)
        { // 有可能是因为模式不是offline的，所以获取不到yuv图
            hb_vio_free_ipubuf(pipeline_id, buffer);
            return -1;
        }
    }
    return ret;
}

int x3_vps_output_release(uint32_t pipeline_id, int channel, hb_vio_buffer_t *buffer)
{
    int ret = 0;
    ret = hb_vio_free_ipubuf(pipeline_id, buffer);
    if (ret)
    {
        LOGE_print("hb_vio_free_ipubuf free yuv error, ret=%d\n", ret);
        return -1;
    }
    return ret;
}
