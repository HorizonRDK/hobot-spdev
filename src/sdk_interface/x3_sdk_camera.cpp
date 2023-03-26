/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-02-20 17:14:01
 * @LastEditTime: 2023-03-05 16:37:47
 ***************************************************************************/
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>

#include "x3_vio_vin.h"
#include "x3_vio_vps.h"
#include "x3_sdk_wrap.h"

#include "utils_log.h"
#include "x3_sdk_wrap.h"
#include "x3_sdk_camera.h"

#define EN_PRINT_INFO 0

namespace srpy_cam
{

    int SrPyCamera::OpenCamera(const int pipe_id, const int video_index, int fps,
                               int chn_num, int *width, int *height)
    {
        int ret = 0;

        memset(&m_x3_modules_info, 0, sizeof(x3_modules_info_t));

        m_x3_modules_info.m_enable = 1;
        m_x3_modules_info.m_vin_enable = 1;

        ret = vin_param_init(video_index, &m_x3_modules_info.m_vin_info);
        if (ret)
        {
            printf("vin_param_init failed, %d\n", ret);
            return -1;
        }

        ret = x3_reconfig_ipu(chn_num, width, height, &m_x3_modules_info.m_vin_info, &m_x3_modules_info.m_ipu_info);
        if (ret)
        {
            printf("x3_reconfig_ipu failed, %d\n", ret);
            return -1;
        }

        ret = x3_vin_init(&m_x3_modules_info.m_vin_info);
        if (ret)
        {
            printf("x3_vin_init failed, %d\n", ret);
            return -1;
        }
        ret = x3_vin_start(&m_x3_modules_info.m_vin_info);
        if (ret)
        {
            x3_vin_deinit(&m_x3_modules_info.m_vin_info);
            return -1;
        }

        /* 打印个模块配置信息 */
        print_debug_infos();
        m_pipe_id = pipe_id;
        last_frame_id = 0;
        return 0;
    }

    int SrPyCamera::OpenVPS(const int pipe_id, int chn_num, int proc_mode,
                            int src_width, int src_height, int *dst_width, int *dst_height,
                            int *crop_x, int *crop_y, int *crop_width, int *crop_height, int *rotate)
    {
        int ret = 0;

        return 0;
    }

    int SrPyCamera::CloseCamera(void)
    {
        x3_vin_stop(&m_x3_modules_info.m_vin_info);
        x3_vin_deinit(&m_x3_modules_info.m_vin_info);
        return 0;
    }

    int SrPyCamera::setExposureGain(int exp_val, int gain_val) { return -1; }

#define SIF_EXCTRL_MAGIC 0x95
#define IOCTL_SIF_EXCTRL_GET_VER _IOW(SIF_EXCTRL_MAGIC, 80, int)
#define IOCTL_SIF_EXCTRL_RST_SYNC _IOW(SIF_EXCTRL_MAGIC, 81, int)
#define IOCTL_SIF_EXCTRL_GET_FRAMEID _IOW(SIF_EXCTRL_MAGIC, 82, int)

    int SrPyCamera::ResetSync(void)
    {
        return -1;
    }

    static int GetSifRawData(const int pipe_id, ImageFrame *image_frame, const int timeout)
    {
        int ret = 0;
        hb_vio_buffer_t *sif_img = nullptr;
        int data_size = 0;

        sif_img = new hb_vio_buffer_t();

        ret = x3_vin_sif_get_data(pipe_id, sif_img, timeout);
        if (ret)
        {
            delete sif_img;
            sif_img = nullptr;
            return -1;
        }

        if (sif_img->img_info.planeCount == 1)
        { // raw的 planeCount是1
            data_size = sif_img->img_info.size[0];
            if (data_size == 0)
            {
                delete sif_img;
                sif_img = nullptr;
                return -1;
            }
            image_frame->data[0] = (uint8_t *)sif_img->img_addr.addr[0];
            image_frame->data_size[0] = data_size;
            image_frame->plane_count = sif_img->img_info.planeCount;
            image_frame->frame_info = static_cast<void *>(sif_img);

            image_frame->image_id = sif_img->img_info.frame_id & 0xFFFF; // 低16位是帧id
            image_frame->image_timestamp = sif_img->img_info.tv.tv_sec * 1000 + sif_img->img_info.tv.tv_usec / 1000;
            image_frame->exp_time = sif_img->img_info.frame_id >> 29; // 高3位是左右激光管状态;
#if EN_PRINT_INFO
            printf("pipe:%d dump normal sif frame id(%ld),plane(%d)size(%d)\n",
                   pipe_id, image_frame->image_id, sif_img->img_info.planeCount,
                   size);
#endif
        }
        else if (sif_img->img_info.planeCount == 2)
        { // yuv的 planeCount是2
            data_size = sif_img->img_info.size[0] + sif_img->img_info.size[1];
            if (data_size == 0)
            {
                delete sif_img;
                sif_img = nullptr;
                return -1;
            }
            // y 和 uv 分量存放在连续地址上，可以只返回一个addr
            image_frame->data[0] = (uint8_t *)sif_img->img_addr.addr[0];
            image_frame->data[1] = (uint8_t *)sif_img->img_addr.addr[1];
            image_frame->data_size[0] = sif_img->img_info.size[0];
            image_frame->data_size[1] = sif_img->img_info.size[1];
            image_frame->plane_count = sif_img->img_info.planeCount;
            image_frame->frame_info = static_cast<void *>(sif_img);

            image_frame->image_id = sif_img->img_info.frame_id & 0xFFFF; // 低16位是帧id
            image_frame->image_timestamp = sif_img->img_info.tv.tv_sec * 1000 + sif_img->img_info.tv.tv_usec / 1000;
            image_frame->exp_time = sif_img->img_info.frame_id >> 29; // 高3位是左右激光管状态;
#if EN_PRINT_INFO
            printf("pipe:%d dump normal sif frame id(%ld),plane(%d)size(%d)\n",
                   pipe_id, image_frame->image_id, sif_img->img_info.planeCount,
                   size);
#endif
        }
        else
        {
            printf("pipe:%d raw buf planeCount wrong !!!\n", pipe_id);
            delete sif_img;
            sif_img = nullptr;
            return -1;
        }

        return 0;
    }

    static int ReleaseSifRawData(const int pipe_id, ImageFrame *image_frame)
    {

        // 释放 data buffer
        x3_vin_sif_release_data(pipe_id, (hb_vio_buffer_t *)image_frame->frame_info);

        // 释放结构体本身内存
        delete ((hb_vio_buffer_t *)image_frame->frame_info);
        image_frame->frame_info = nullptr;

        return 0;
    }

    static int GetISPYuvData(const int pipe_id, ImageFrame *image_frame, const int timeout)
    {
        int ret = 0;
        hb_vio_buffer_t *isp_yuv = nullptr;
        uint32_t data_size = 0;

        isp_yuv = new hb_vio_buffer_t();

        ret = x3_vin_isp_get_data(pipe_id, isp_yuv, timeout);
        if (ret)
        {
            delete isp_yuv;
            isp_yuv = nullptr;
            return -1;
        }

        if (isp_yuv->img_info.planeCount == 2)
        { // yuv的 planeCount是2
            data_size = isp_yuv->img_info.size[0] + isp_yuv->img_info.size[1];
            if (data_size == 0)
            {
                delete isp_yuv;
                isp_yuv = nullptr;
                return -1;
            }
            // y 和 uv 分量存放在连续地址上，可以只返回一个addr
            image_frame->data[0] = (uint8_t *)isp_yuv->img_addr.addr[0];
            image_frame->data[1] = (uint8_t *)isp_yuv->img_addr.addr[1];
            image_frame->data_size[0] = isp_yuv->img_info.size[0];
            image_frame->data_size[1] = isp_yuv->img_info.size[1];
            image_frame->plane_count = isp_yuv->img_info.planeCount;
            image_frame->frame_info = static_cast<void *>(isp_yuv);

            image_frame->image_id = isp_yuv->img_info.frame_id & 0xFFFF; // 低16位是帧id
            image_frame->image_timestamp = isp_yuv->img_info.tv.tv_sec * 1000 + isp_yuv->img_info.tv.tv_usec / 1000;
            image_frame->exp_time = isp_yuv->img_info.frame_id >> 29; // 高3位是左右激光管状态;
#if EN_PRINT_INFO
            printf("pipe:%d dump normal raw frame id(%ld),plane(%d)size(%d)\n",
                   pipe_id, image_frame->image_id, isp_yuv->img_info.planeCount,
                   size);
#endif
        }
        else
        {
            printf("pipe:%d isp yuv buf planeCount wrong !!!\n", pipe_id);
            delete isp_yuv;
            isp_yuv = nullptr;
            return -1;
        }

        return 0;
    }

    static int ReleaseISPYuvData(const int pipe_id, ImageFrame *image_frame)
    {

        // 释放 data buffer
        x3_vin_isp_release_data(pipe_id, (hb_vio_buffer_t *)image_frame->frame_info);

        // 释放结构体本身内存
        delete ((hb_vio_buffer_t *)image_frame->frame_info);
        image_frame->frame_info = nullptr;

        return 0;
    }

    static int GetVpsChnData(const int pipe_id, int chn_id, ImageFrame *image_frame, const int timeout)
    {
        int ret = 0;
        hb_vio_buffer_t *vps_yuv = nullptr;
        uint32_t data_size = 0;

        printf("pipe_id:%d\n", pipe_id);

        vps_yuv = new hb_vio_buffer_t();

        ret = x3_vps_get_output(pipe_id, chn_id, vps_yuv, timeout);
        if (ret)
        {
            delete vps_yuv;
            vps_yuv = nullptr;
            return -1;
        }

        if (vps_yuv->img_info.planeCount == 2)
        { // yuv的 planeCount是2
            data_size = vps_yuv->img_info.size[0] + vps_yuv->img_info.size[1];
            if (data_size == 0)
            {
                delete vps_yuv;
                vps_yuv = nullptr;
                return -1;
            }
            // y 和 uv 分量存放在连续地址上，可以只返回一个addr
            image_frame->data[0] = (uint8_t *)vps_yuv->img_addr.addr[0];
            image_frame->data[1] = (uint8_t *)vps_yuv->img_addr.addr[1];
            image_frame->data_size[0] = vps_yuv->img_addr.stride_size * vps_yuv->img_addr.height;
            image_frame->data_size[1] = vps_yuv->img_addr.stride_size * vps_yuv->img_addr.height / 2;
            image_frame->plane_count = vps_yuv->img_info.planeCount;
            image_frame->frame_info = static_cast<void *>(vps_yuv);

            image_frame->image_id = vps_yuv->img_info.frame_id & 0xFFFF; // 低16位是帧id
            image_frame->image_timestamp = vps_yuv->img_info.tv.tv_sec * 1000 + vps_yuv->img_info.tv.tv_usec / 1000;
            image_frame->exp_time = vps_yuv->img_info.frame_id >> 29; // 高3位是左右激光管状态;
#if EN_PRINT_INFO
            printf("pipe:%d dump normal raw frame id(%ld),plane(%d)size(%d)\n",
                   pipe_id, image_frame->image_id, vps_yuv->img_info.planeCount,
                   size);
#endif
        }
        else
        {
            printf("pipe:%d vps yuv buf planeCount wrong !!!\n", pipe_id);
            delete vps_yuv;
            vps_yuv = nullptr;
            return -1;
        }

        return 0;
    }

    static int ReleaseVpsChnData(const int pipe_id, int chn_id, ImageFrame *image_frame)
    {
        // 释放 data buffer
        x3_vps_output_release(pipe_id, chn_id, (hb_vio_buffer_t *)image_frame->frame_info);

        // 释放结构体本身内存
        delete ((hb_vio_buffer_t *)image_frame->frame_info);
        image_frame->frame_info = nullptr;

        return 0;
    }

    // 对一路的三个数据处理模块取数据
    int SrPyCamera::GetImageFrame(ImageFrame *image_frame, DevModule module,
                                  int width, int height, const int timeout)
    {
        int ret = 0;
        int chn_id = -1;

        printf("GetImageFrame module=%d\n", module);

        switch (module)
        {
        case Dev_IPU:
            chn_id = GetChnId(SrPy_Camera, 0, width, height);
            if (chn_id == -1)
            {
                printf("Error: no vps chn can be get\n");
                return -1;
            }
            printf("m_pipe_id:%d\n", m_pipe_id);
            ret = GetVpsChnData(m_pipe_id, chn_id, image_frame, timeout);
            break;
        case Dev_ISP:
            if (m_x3_modules_info.m_vin_enable == 0)
            {
                printf("Error: vin was not enable\n");
                return -1;
            }
            ret = GetISPYuvData(m_pipe_id, image_frame, timeout);
            break;
        case Dev_SIF:
            if (m_x3_modules_info.m_vin_enable == 0)
            {
                printf("Error: vin was not enable\n");
                return -1;
            }
            ret = GetSifRawData(m_pipe_id, image_frame, timeout);
            break;
        default:
            printf("Error: module not supported!\n");
            return -1;
        }

        // 计算丢帧数和更新上次帧id
        image_frame->lost_image_num = image_frame->image_id - last_frame_id - 1;
        last_frame_id = image_frame->image_id;
        return ret;
    }

    void SrPyCamera::ReturnImageFrame(ImageFrame *image_frame, DevModule module,
                                      int width, int height)
    {
        int chn_id = -1;

        switch (module)
        {
        case Dev_IPU:
            chn_id = GetChnId(SrPy_Camera, 0, width, height);
            if (chn_id == -1)
            {
                printf("Error: no vps chn can be get\n");
                return;
            }
            ReleaseVpsChnData(m_pipe_id, chn_id, image_frame);
            break;
        case Dev_ISP:
            ReleaseISPYuvData(m_pipe_id, image_frame);
            break;
        case Dev_SIF:
            ReleaseSifRawData(m_pipe_id, image_frame);
            break;
        default:
            printf("Error: module not supported!\n");
        }
    }

    // 对一路的三个数据处理模块传入数据
    int SrPyCamera::SetImageFrame(ImageFrame *image_frame, DevModule module)
    {
        int ret = 0;
        hb_vio_buffer_t vio_buf = {0};
        int width, height;
        static int buf_cnt = 0;
        int buf_index = buf_cnt % VPS_FEEDBACK_BUF_BUM;

        width = 1920;
        height = 1080;

        vio_buf.img_addr.paddr[0] = m_vp_param.mmz_paddr[buf_index];
        vio_buf.img_addr.paddr[1] = m_vp_param.mmz_paddr[buf_index] + width * height;
        vio_buf.img_addr.addr[0] = m_vp_param.mmz_vaddr[buf_index];
        vio_buf.img_addr.addr[1] = m_vp_param.mmz_vaddr[buf_index] + width * height;
        vio_buf.img_info.planeCount = 2;
        vio_buf.img_info.img_format = 12;
        vio_buf.img_addr.width = width;
        vio_buf.img_addr.height = height;
        vio_buf.img_addr.stride_size = width;
        memcpy(vio_buf.img_addr.addr[0], image_frame->data[0], image_frame->data_size[0]);

        switch (module)
        {
        case Dev_IPU:
            ret = x3_vps_input(m_pipe_id, &vio_buf);
            break;
        case Dev_ISP:
        case Dev_SIF:
        default:
            printf("Error: module not supported!\n");
            return -1;
        }

        return ret;
    }

    int SrPyCamera::GetPipeId()
    {
        return m_pipe_id;
    }

    int SrPyCamera::GetChnId(SrPy_Object_e object, int for_bind, int width, int height)
    {
        return 2;
    }

} // namespace srpy_cam
