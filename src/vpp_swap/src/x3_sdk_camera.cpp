#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "x3_vio_venc.h"
#include "x3_vio_vin.h"
#include "x3_vio_vps.h"
#include "x3_vio_vot.h"
#include "x3_vio_bind.h"
#include "x3_sdk_wrap.h"
#include "x3_vio_rgn.h"
#include "x3_vio_vp.h"

#include "utils_log.h"
#include "x3_sdk_wrap.h"
#include "x3_preparam.h"
#include "x3_sdk_camera.h"

#define EN_PRINT_INFO 0

namespace srpy_cam
{

static int vps_select_chn(int *chn_en, int src_width, int src_height, int dst_width, int dst_height)
{
    if (((dst_width <= src_width) && (dst_height <= src_height)) &&
        (!(*chn_en & 1 << HB_VIO_IPU_DS2_DATA)) &&
        (dst_width <= 4096 && dst_height <= 4096)) {
        return HB_VIO_IPU_DS2_DATA;
    }
    if ((dst_width <= src_width) || (dst_height <= src_height)) {
        if ((dst_width <= 1920 && dst_height <= 1080) &&
            (!(*chn_en & 1 << HB_VIO_IPU_DS1_DATA))) {
            return HB_VIO_IPU_DS1_DATA;
        } else if ((dst_width <= 1920 && dst_height <= 1080) &&
                (!(*chn_en & 1 << HB_VIO_IPU_DS3_DATA))) {
            return HB_VIO_IPU_DS3_DATA;
        } else if ((dst_width <= 1280 && dst_height <= 720) &&
                (!(*chn_en & 1 << HB_VIO_IPU_DS0_DATA))) {
            return HB_VIO_IPU_DS0_DATA;
        } else if ((dst_width <= 1280 && dst_height <= 720) &&
                (!(*chn_en & 1 << HB_VIO_IPU_DS4_DATA))) {
            return HB_VIO_IPU_DS4_DATA;
        }
    }
    if (((dst_width >= src_width) && (dst_height >= src_height)) &&
            (!(*chn_en & 1 << HB_VIO_IPU_US_DATA)) &&
            (dst_width <= 4096 && dst_height <= 4096)) {
        return HB_VIO_IPU_US_DATA;
    }

    return -1;
}

int x3_cam_init_param(x3_modules_info_t *info, const int pipe_id, const int video_index, int fps,
                int chn_num,x3_sensors_parameters *parameters, int *width, int *height)
{
    int i = 0;
    int ret = 0;
    int mipi_width = 0, mipi_height = 0;
    int chn_index = 0, chn_en = 0, chn_data = -1;
    char file_name[SDK_MAX_PATH_LENGTH];
    char result[SDK_MAX_RESULT_LENGTH];
    char cmd[SDK_MAX_CMD_LENGTH] = {0};
    FILE *stream;

    memset(info, 0, sizeof(x3_modules_info_t));

    /* 使能sensor mclk, 否则i2c 通信不会成功 */
    for (i = 0; i < 2; i++)
    {
        /* 读取 /sys/class/vps/mipi_host[i]/param/snrclk_freq  的值 \
         * 如果该mipi host可以使用则会是一个大于1000的值，否则为0 \
         * 通过判断该值不为0作为可以使能该路mipi mclk的依据
         */
        memset(file_name, '\0', sizeof(file_name));
        memset(result, '\0', sizeof(result));
        sprintf(file_name, "/sys/class/vps/mipi_host%d/param/snrclk_freq", i);
        stream = fopen(file_name, "r");
        if (!stream)
        {
            continue;
        }
        ret = fread(result, sizeof(char), 32, stream);
        if (ret <= 0)
        {
            LOGE_print("read fail\n");
            fclose(stream);
            return -1;
        }
        fclose(stream);
 
        /* 如果频率不为0   就使能该路mclk */
        if (strncmp(result, "0", 1) != 0)
        {
            LOGI_print("Enable mipi host%d mclk", i);
            HB_MIPI_SetSensorClock(i, 24000000);
            HB_MIPI_EnableSensorClock(i);
        }
    }

    // 1.1 通过默认参数配置vin参数
    info->m_vin_info.dev_id = pipe_id;
    info->m_vin_info.pipe_id = pipe_id;
    info->m_vin_info.parameters = parameters;
    ret = vin_param_init(video_index, &info->m_vin_info);
    if (ret) {
        LOGE_print("vin_param_init failed, %d", ret);
        return ret;
    }
    info->m_vin_enable = 1;

    // 1.2 根据入参修改分辨率等
    if (fps != 0) {
        info->m_vin_info.mipi_attr.mipi_host_cfg.fps = fps;
    }

    // 2.1 根据vin中的分辨率配置vps
    // sensor输出的分辨率要作为vps的输入
    mipi_width = info->m_vin_info.mipi_attr.mipi_host_cfg.width;
    mipi_height = info->m_vin_info.mipi_attr.mipi_host_cfg.height;

    info->m_vps_enable = 1;
    info->m_vps_infos.m_vps_info[0].m_vps_grp_id = pipe_id;
    info->m_vps_infos.m_group_num = 1;
    // 2.2 配置group的输入
    ret |= vps_grp_param_init(&info->m_vps_infos.m_vps_info[0], pipe_id,
                                mipi_width, mipi_height);
    // 2.3 配置group每个通道的参数
    info->m_vps_infos.m_vps_info[0].m_chn_num = chn_num;
    for (int i = 0; i < chn_num; i++) {
        if((width[i] % 4 != 0) || (height[i] % 2 != 0))
        {
            LOGE_print("Width: %d must be divisible by 4, height: %d must be even!\n",width[i],height[i]);
            ret = -1;
            break;
        }
        chn_data = -1;
        if ((width[i] == 0) && (height[i] == 0)) {//如果高宽为0，那么就开一个和mipi原始高宽一致的通道
            width[i] = mipi_width;
            height[i] = mipi_height;
        }
        chn_data = vps_select_chn(&chn_en, mipi_width, mipi_height, width[i], height[i]);
        if (chn_data >= 0) {
            ret |= vps_chn_param_init(&info->m_vps_infos.m_vps_info[0].m_vps_chn_attrs[chn_index],
                    chn_data, width[i], height[i], fps);
            chn_en |= 1 << chn_data;
            chn_index++;
            printf("Setting VPS channel-%d: src_w:%d, src_h:%d; dst_w:%d, dst_h:%d;\n", chn_data,
                mipi_width, mipi_height, width[i], height[i]);
        } else {
            LOGW_print("Invalid size:%dx%d\n", width[i], height[i]);
            info->m_vps_infos.m_vps_info[0].m_chn_num--;
        }
    }

    if (ret == 0) {
        info->m_enable = 1;
    }
    return ret;
}

int x3_cam_vps_init_param(x3_modules_info_t *info, const int pipe_id, int chn_num, int proc_mode,
          int src_width, int src_height, int *dst_width, int *dst_height,
          int *crop_x, int *crop_y, int *crop_width, int *crop_height, int *rotate)
{
    int ret = 0;
    int chn_index = 0, chn_en = 0, chn_data = -1;
    int fps = 30;

    memset(info, 0, sizeof(x3_modules_info_t));

    info->m_vps_enable = 1;
    info->m_vps_infos.m_vps_info[0].m_vps_grp_id = pipe_id;
    info->m_vps_infos.m_group_num = 1;
    // 2.2 配置group的输入
    ret |= vps_grp_param_init(&info->m_vps_infos.m_vps_info[0], pipe_id,
                                src_width, src_height);
    // 2.3 配置group每个通道的参数
    info->m_vps_infos.m_vps_info[0].m_chn_num = chn_num;
    for (int i = 0; i < chn_num; i++) {
        switch (proc_mode)
        {
        case VPS_SCALE_ROTATE_CROP:
        case VPS_SCALE_CROP:
            if ((crop_width[i] == 0) || (crop_height[i] == 0)) {
                crop_width[i] = src_width;
                crop_height[i] = src_height;
            }
        case VPS_SCALE_ROTATE:
        case VPS_SCALE:
            if ((dst_width[i] == 0) && (dst_height[i] == 0)) {
                if ((crop_width[i] != 0) && (crop_height[i] != 0)) {
                    dst_width[i] = crop_width[i];
                    dst_height[i] = crop_height[i];
                } else {
                    dst_width[i] = src_width;
                    dst_height[i] = src_height;
                }
            }
            break;
        default:
            break;
        }

        chn_data = vps_select_chn(&chn_en, src_width, src_height, dst_width[i], dst_height[i]);
        if (chn_data >= 0) {
            if (proc_mode >= VPS_SCALE) {
                ret |= vps_chn_param_init(&info->m_vps_infos.m_vps_info[0].m_vps_chn_attrs[chn_index],
                    chn_data, dst_width[i], dst_height[i], fps);
            }
            if ((proc_mode == VPS_SCALE_CROP) || (proc_mode == VPS_SCALE_ROTATE_CROP)) {
                ret |= vps_chn_crop_param_init(&info->m_vps_infos.m_vps_info[0].m_vps_chn_attrs[chn_index],
                    crop_x[i], crop_y[i], crop_width[i], crop_height[i]);
            }
            if ((proc_mode == VPS_SCALE_ROTATE) || (proc_mode == VPS_SCALE_ROTATE_CROP)) {
                ret |= vps_chn_rotate_param_init(&info->m_vps_infos.m_vps_info[0].m_vps_chn_attrs[chn_index],
                    rotate[i] % ROTATION_MAX);
            }
            printf("Setting VPS channel-%d: src_w:%d, src_h:%d; dst_w:%d, dst_h:%d;\n", chn_data,
                src_width, src_height, dst_width[i], dst_height[i]);
            chn_en |= 1 << chn_data;
            chn_index++;
        } else {
            LOGW_print("Invalid size:%dx%d\n", dst_width[i], dst_height[i]);
            info->m_vps_infos.m_vps_info[0].m_chn_num--;
        }
    }

    if (ret == 0) {
        info->m_enable = 1;
    }
    return ret;
}

static int x3_cam_init(x3_modules_info_t *info)
{
    int ret = 0;
    // 1. 初始化 vin，此处调用失败，大概率是因为sensor没有接，或者没有接好，或者sensor库用的版本不配套
    if (info->m_vin_enable) {
        ret = x3_vin_init(&info->m_vin_info);
        if (ret) {
            LOGE_print("x3_vin_init failed, %d", ret);
            goto vin_err;
        }
        LOGD_print("x3_vin_init ok!\n");
    }
    // 2. 初始化 vps
    if (info->m_vps_enable) {
        ret = x3_vps_init_wrap(&info->m_vps_infos.m_vps_info[0]);
        if (ret) {
            LOGE_print("x3_vps_init failed, %d", ret);
            goto vps_err;
        }
        LOGD_print("x3_vps_init_wrap ok!\n");
    }

    // 3 vin bind vps
    if (info->m_vin_enable && info->m_vps_enable) {
        ret = x3_vin_bind_vps(info->m_vin_info.pipe_id, info->m_vps_infos.m_vps_info[0].m_vps_grp_id,
                              info->m_vin_info.vin_vps_mode);
        if (ret) {
            LOGE_print("x3_vin_bind_vps failed, %d", ret);
            goto vin_bind_err;
        }
    }

    return ret;

vin_bind_err:
    if (info->m_vps_enable) {
        x3_vps_deinit_wrap(&info->m_vps_infos.m_vps_info[0]);
    }

vps_err:
    if (info->m_vin_enable) {
        x3_vin_deinit(&info->m_vin_info);
    }
vin_err:
    return -1;
}

static int x3_cam_deinit(x3_modules_info_t *info)
{
    int ret = 0;

    if (info->m_vin_enable && info->m_vps_enable) {
        ret = x3_vin_unbind_vps(info->m_vin_info.pipe_id, info->m_vps_infos.m_vps_info[0].m_vps_grp_id,
                                info->m_vin_info.vin_vps_mode);
        if (ret) {
            LOGE_print("x3_vin_unbind_vps failed, %d", ret);
        }
    }

    if (info->m_vps_enable) {
        x3_vps_deinit_wrap(&info->m_vps_infos.m_vps_info[0]);
    }

    if (info->m_vin_enable) {
        x3_vin_deinit(&info->m_vin_info);
    }

    return ret;
}

static int x3_cam_start(x3_modules_info_t *info)
{
    int ret = 0;

    if (info->m_enable == 0)
        return 0;
    // start vps, vin

    if (info->m_vps_enable) {
        ret = x3_vps_start(info->m_vps_infos.m_vps_info[0].m_vps_grp_id);
        if (ret) {
            LOGE_print("x3_vps_start failed, %d", ret);
            return -3001;
        }
    }

    if (info->m_vin_enable) {
        ret = x3_vin_start(&info->m_vin_info);
        if (ret) {
            LOGE_print("x3_vin_start failed, %d", ret);
            return -3003;
        }
    }

    /* 打印个模块配置信息 */
    // print_debug_infos();
    return 0;
}

static int x3_cam_stop(x3_modules_info_t *info)
{
    int ret = 0;

    if (info->m_enable == 0)
        return 0;

    if (info->m_vin_enable) {
        x3_vin_stop(&info->m_vin_info);
    }
    if (info->m_vps_enable) {
        x3_vps_stop(info->m_vps_infos.m_vps_info[0].m_vps_grp_id);
    }

    return ret;
}

int x3_cam_vp_init(vp_param_t *vp_param, int width, int height)
{
    int ret = 0;

    memset(vp_param, 0, sizeof(vp_param_t));
    vp_param->mmz_cnt = VPS_FEEDBACK_BUF_BUM;
    vp_param->mmz_size = width * height * 3 / 2;

    ret = x3_vp_init();
    if (ret) {
        return -1;
    }
    ret = x3_vp_alloc(vp_param);
    if (ret) {
        return -1;
    }

    return ret;
}

int x3_cam_vp_deinit(vp_param_t *vp_param)
{
    int ret = 0;

    ret = x3_vp_free(vp_param);

    ret = x3_vp_deinit();
    if (ret) {
        return -1;
    }

    return ret;
}

int VPPCamera::OpenCamera(const int pipe_id, const int video_index, int fps,
          int chn_num,x3_sensors_parameters *parameters, int *width, int *height)
{
    int ret = 0;

    ret = x3_cam_init_param(&m_x3_modules_info, pipe_id, video_index, fps, chn_num, parameters, width, height);
    if (ret)
        return -1;
    ret = x3_cam_init(&m_x3_modules_info);
    if (ret)
        return -1;
    ret = x3_cam_start(&m_x3_modules_info);
    if (ret) {
        x3_cam_deinit(&m_x3_modules_info);
        return -1;
    }

    m_pipe_id = pipe_id;
    last_frame_id = 0;
    return 0;
}

int VPPCamera::OpenVPS(const int pipe_id, int chn_num, int proc_mode,
          int src_width, int src_height, int *dst_width, int *dst_height,
          int *crop_x, int *crop_y, int *crop_width, int *crop_height, int *rotate)
{
    int ret = 0;

    ret = x3_cam_vps_init_param(&m_x3_modules_info, pipe_id, chn_num, proc_mode, src_width, src_height,
        dst_width, dst_height, crop_x, crop_y, crop_width, crop_height, rotate);
    if (ret)
        return -1;
    ret = x3_cam_init(&m_x3_modules_info);
    if (ret)
        return -1;
    ret = x3_cam_start(&m_x3_modules_info);
    if (ret) {
        x3_cam_deinit(&m_x3_modules_info);
        return -1;
    }

    ret = x3_cam_vp_init(&m_vp_param, m_x3_modules_info.m_vps_infos.m_vps_info[0].m_vps_grp_attr.maxW,
        m_x3_modules_info.m_vps_infos.m_vps_info[0].m_vps_grp_attr.maxH);
    if (ret) {
        x3_cam_deinit(&m_x3_modules_info);
        return -1;
    }

    m_pipe_id = pipe_id;
    last_frame_id = 0;
    return 0;
}

int VPPCamera::CloseCamera(void)
{
    if (m_vp_param.mmz_cnt > 0) {
        x3_cam_vp_deinit(&m_vp_param);
    }
    x3_cam_stop(&m_x3_modules_info);
    x3_cam_deinit(&m_x3_modules_info);
    return 0;
}

int VPPCamera::setExposureGain(int exp_val, int gain_val) { return -1; }

#define SIF_EXCTRL_MAGIC             0x95
#define IOCTL_SIF_EXCTRL_GET_VER     _IOW(SIF_EXCTRL_MAGIC, 80, int)
#define IOCTL_SIF_EXCTRL_RST_SYNC    _IOW(SIF_EXCTRL_MAGIC, 81, int)
#define IOCTL_SIF_EXCTRL_GET_FRAMEID _IOW(SIF_EXCTRL_MAGIC, 82, int)

int VPPCamera::ResetSync(void)
{
    int ret = 0;
    int fd = -1;

    fd = open("/dev/sif_exctrl", O_RDWR);
    if (fd < 0) {
        perror("open");
        return -1;
    }
    /* Get version */
#if 0
    ret = ioctl(fd, IOCTL_SIF_EXCTRL_GET_VER, buf);
    if (ret == -1) {
        printf("ioctl: %s\n", strerror(errno));
        close(fd);
        exit(-1);
    }
#endif
    ret = ioctl(fd, IOCTL_SIF_EXCTRL_RST_SYNC, m_pipe_id);
    if (ret == -1) {
        printf("ioctl: %s\n", strerror(errno));
    }

    close(fd);
    return -1;
}

static int GetSifRawData(const int pipe_id, ImageFrame *image_frame, const int timeout)
{
    int ret = 0;
    hb_vio_buffer_t *sif_img = nullptr;
    int data_size = 0;

    sif_img = new hb_vio_buffer_t();

    ret = x3_vin_sif_get_data(pipe_id, sif_img, timeout);
    if (ret) {
        delete sif_img;
        sif_img = nullptr;
        return -1;
    }

    if (sif_img->img_info.planeCount == 1) { // raw的 planeCount是1
        data_size = sif_img->img_info.size[0];
        if (data_size == 0) {
            delete sif_img;
            sif_img = nullptr;
            return -1;
        }
        LOGD_print("data_size:%d,width:%d,height:%d,stride:%d\n",data_size,sif_img->img_addr.width,sif_img->img_addr.height,sif_img->img_addr.stride_size);
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
               data_size);
#endif
    } else if (sif_img->img_info.planeCount == 2) { // yuv的 planeCount是2
        data_size = sif_img->img_info.size[0] + sif_img->img_info.size[1];
        if (data_size == 0) {
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
               data_size);
#endif
    } else {
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
    if (ret) {
        delete isp_yuv;
        isp_yuv = nullptr;
        return -1;
    }

    if (isp_yuv->img_info.planeCount == 2) { // yuv的 planeCount是2
        data_size = isp_yuv->img_info.size[0] + isp_yuv->img_info.size[1];
        if (data_size == 0) {
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
        LOGD_print("data_size:%d,width:%d,height:%d,stride:%d\n",data_size,isp_yuv->img_addr.width,isp_yuv->img_addr.height,isp_yuv->img_addr.stride_size);
        image_frame->image_id = isp_yuv->img_info.frame_id & 0xFFFF; // 低16位是帧id
        image_frame->image_timestamp = isp_yuv->img_info.tv.tv_sec * 1000 + isp_yuv->img_info.tv.tv_usec / 1000;
        image_frame->exp_time = isp_yuv->img_info.frame_id >> 29; // 高3位是左右激光管状态;
#if EN_PRINT_INFO
        printf("pipe:%d dump normal raw frame id(%ld),plane(%d)size(%d)\n",
               pipe_id, image_frame->image_id, isp_yuv->img_info.planeCount,
               data_size);
#endif
    } else {
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

    vps_yuv = new hb_vio_buffer_t();

    ret = x3_vps_get_output(pipe_id, chn_id, vps_yuv, timeout);
    if (ret) {
        delete vps_yuv;
        vps_yuv = nullptr;
        return -1;
    }

    if (vps_yuv->img_info.planeCount == 2) { // yuv的 planeCount是2
        data_size = vps_yuv->img_info.size[0] + vps_yuv->img_info.size[1];
        if (data_size == 0) {
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
               data_size);
#endif
    } else {
        printf("pipe:%d isp yuv buf planeCount wrong !!!\n", pipe_id);
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
int VPPCamera::GetImageFrame(ImageFrame *image_frame, DevModule module,
                int width, int height, const int timeout)
{
    int ret = 0;
    int chn_id = -1;

    switch (module) {
    case Dev_IPU:
        if (m_x3_modules_info.m_vps_enable == 0) {
            printf("Error: vps was not enable\n");
            return -1;
        }
        chn_id = GetChnId(VPP_CAMERA, 0, width, height);
        if (chn_id == -1) {
            printf("Error: no vps chn can be get\n");
            return -1;
        }
        ret = GetVpsChnData(m_pipe_id, chn_id, image_frame, timeout);
        break;
    case Dev_ISP:
        if ((m_x3_modules_info.m_vin_enable == 0) ||
            (m_x3_modules_info.m_vin_info.isp_enable == 0)) {
            printf("Error: vin or isp was not enable\n");
            return -1;
        }
        ret = GetISPYuvData(m_pipe_id, image_frame, timeout);
        break;
    case Dev_SIF:
        if (m_x3_modules_info.m_vin_enable == 0) {
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

void VPPCamera::ReturnImageFrame(ImageFrame *image_frame, DevModule module,
        int width, int height)
{
    int chn_id = -1;

    switch (module) {
    case Dev_IPU:
        chn_id = GetChnId(VPP_CAMERA, 0, width, height);
        if (chn_id == -1) {
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

// 对一路的三个数据处理模块取数据
int VPPCamera::SetImageFrame(ImageFrame *image_frame, DevModule module)
{
    int ret = 0;
    hb_vio_buffer_t vio_buf = {0};
    int width, height;
    static int buf_cnt = 0;
    int buf_index = buf_cnt % VPS_FEEDBACK_BUF_BUM;

    width = m_x3_modules_info.m_vps_infos.m_vps_info[0].m_vps_grp_attr.maxW;
    height = m_x3_modules_info.m_vps_infos.m_vps_info[0].m_vps_grp_attr.maxH;

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

    switch (module) {
    case Dev_IPU:
        if (m_x3_modules_info.m_vps_enable == 0) {
            printf("Error: vps was not enable\n");
            return -1;
        }
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

int VPPCamera::GetPipeId()
{
    return m_pipe_id;
}

int VPPCamera::GetChnId(Sdk_Object_e object, int for_bind, int width, int height)
{
    x3_vps_chn_attr_t *chn_attr = NULL;
    for (int i = 0; i < m_x3_modules_info.m_vps_infos.m_vps_info[0].m_chn_num; i++) {
        chn_attr = &m_x3_modules_info.m_vps_infos.m_vps_info[0].m_vps_chn_attrs[i];
        if (for_bind) {
            // for bind
            if (chn_attr->m_is_bind == VPP_CAMERA) {
                if(width == 0 && height == 0){
                    return chn_attr->m_chn_id;
                }
                if ((width != (int)chn_attr->m_chn_attr.width ||
                    height != (int)chn_attr->m_chn_attr.height)) {
                    continue;
                }
                chn_attr->m_is_bind = object;
                return chn_attr->m_chn_id;
            }
        } else {
            // for unbind
            if (chn_attr->m_is_bind == object) {
                if(width == 0 && height == 0){
                    return chn_attr->m_chn_id;
                }
                if ((width != (int)chn_attr->m_chn_attr.width ||
                    height != (int)chn_attr->m_chn_attr.height)) {
                    continue;
                }
                chn_attr->m_is_bind = VPP_CAMERA;
                return chn_attr->m_chn_id;
            }
        }
    }
    return -1;
}

} // namespace srpy_cam
