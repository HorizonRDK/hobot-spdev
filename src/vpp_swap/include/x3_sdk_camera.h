/*
 * @Author: yaqiang.li
 * @Date:   2021-07-12 11:34:41
 * @Last Modified by:   yaqiang.li
 * @Last Modified time: 2021-07-12 11:35:15
 */
#ifndef __X3_SDK_CAM_H__
#define __X3_SDK_CAM_H__

#include <sstream>
#include <string>

#include "x3_sdk_wrap.h"

namespace srpy_cam
{

#define srpy_PIXEL_FORMAT_NV12 1
#define srpy_PIXEL_FORMAT_RAW  2
#define CAMERA_CHN_NUM 6
#define VPS_FEEDBACK_BUF_BUM 2

enum DevModule {
    Dev_SIF,
    Dev_ISP,
    Dev_IPU
};

enum VPS_PROCESS_MODE {
    VPS_SCALE = 1,
    VPS_SCALE_CROP = 2,
    VPS_SCALE_ROTATE = 3,
    VPS_SCALE_ROTATE_CROP = 4,
};

typedef struct {
    int m_enable;
    int m_vin_enable;         // 使能标志
    x3_vin_info_t m_vin_info; // 包括 sensor、 mipi、isp、 ldc、dis的配置
    int m_vps_enable;
    x3_vps_infos_t m_vps_infos; // vps的配置，支持多个vps group
} x3_modules_info_t;

class VPPCamera
{
  public:
    VPPCamera() = default;
    virtual ~VPPCamera() = default;

    /**
     * @brief 开启相机
     * @param [in] pipe_id       视频pipeline id
     * @param [in] video_index   相机节点
     * @param [in] format        图像格式（NV12，raw12，raw10）
     * @param [in] fps           帧率
     * @param [in] width         图像宽度
     * @param [in] height        图像高度
     *
     * @retval 0      成功
     * @retval -1     失败
     */
    int OpenCamera(const int pipe_id, const int video_index, int fps,
          int chn_num, int *width, int *height);

    /**
     * @brief 指定原始分辨率并打开摄像头，适用于有多种分辨率的摄像头
     * 
     * @param pipe_id           视频pipeline id
     * @param video_index       相机节点
     * @param fps               帧率
     * @param chn_num           VPS通道数量
     * @param parameters        raw图相关参数 参见 x3_sensors_parameters 声明部分
     * @param width             VPS宽数组
     * @param height            VPS高数组
     * @return int 
     */
    int OpenCamera(const int pipe_id, const int video_index, int fps,
          int chn_num,x3_sensors_parameters *parameters, int *width, int *height);

    /**
     * @brief 开启VPS
     * @param [in] pipe_id       视频pipeline id
     * @param [in] proc_mode     vps 处理模式
     * @param [in] src_width     输入宽
     * @param [in] src_height    输入高
     * @param [in] dst_width     输出宽
     * @param [in] dst_height    输出高
     * @param [in] crop_x        裁剪坐标x
     * @param [in] crop_y        裁剪坐标y
     * @param [in] crop_width    裁剪宽
     * @param [in] crop_height   裁剪高
     * @param [in] rotate      旋转角度
     *
     * @retval 0      成功
     * @retval -1     失败
     */
    int OpenVPS(const int pipe_id, int chn_num, int proc_mode,
          int src_width, int src_height, int *dst_width, int *dst_height,
          int *crop_x, int *crop_y, int *crop_width, int *crop_height, int *rotate);

    /**
     * @brief 关闭相机
     * @param void
     *
     * @retval 0      成功
     * @retval -1      失败
     */
    int CloseCamera(void);

    /**
     * @brief 设置相机曝光增益
     * @param [in] exp_val       曝光值
     * @param [in] gain_val      增益值
     *
     * @retval 0      成功
     * @retval -1      失败
     */
    int setExposureGain(int exp_val, int gain_val);

    /**
     * @brief 重置相机同步信号，图像ID重置
     * @param void
     *
     * @retval 0      成功
     * @retval -1      失败
     */
    int ResetSync(void);

    /**
     * @brief 获取图像数据（dqbuf）
     * @param [out] image_frame  用于保存图像数据的内存地址
     *
     * @retval 0      成功
     * @retval -1      失败
     */
    int GetImageFrame(ImageFrame *image_frame, const int timeout = 0);

    /**
     * @brief 获取图像数据（dqbuf）
     * @param [out] image_frame    用于保存YUV图像数据的内存地址
     * @param [in] module        从哪个模块取图：0:SIF 1:ISP: 2: IPU CHN(ds2 default)
     *
     * @retval 0        成功
     * @retval -1     失败
     */
    int GetImageFrame(ImageFrame *image_frame, DevModule module,
            int width, int height, const int timeout = 0);

    /**
     * @brief 设置图像数据（dqbuf）
     * @param [in] image_frame    用于保存YUV图像数据的内存地址
     * @param [in] module        从哪个模块取图：0:SIF 1:ISP: 2: IPU CHN(ds2 default)
     *
     * @retval 0        成功
     * @retval -1     失败
     */
    int SetImageFrame(ImageFrame *image_frame, DevModule module);

    /**
     * @brief 释放图像数据（qbuf）
     * @param [in] image_frame   保存图像数据的内存地址
     *
     * @retval 0      成功
     * @retval -1      失败
     */
    void ReturnImageFrame(ImageFrame *image_frame);

    /**
     * @brief 释放图像数据（qbuf）
     * @param [in] image_frame   保存图像数据的内存地址
     * @param [out] format       保存图像数据的格式， 0：YUV 1：RAW
     *
     * @retval 0      成功
     * @retval -1      失败
     */
    void ReturnImageFrame(ImageFrame *image_frame, DevModule module,
              int width, int height);

    /**
     * @brief 获取pipe id
     *
     * @retval 非-1      成功
     * @retval -1        失败
     */
    int GetPipeId();

    /**
     * @brief 获取chn id
     * @param [in] Sdk_Object_e   当前需要获取chn id的模块
     * @param [in] width   当前需要获取chn id的宽
     * @param [in] height   当前需要获取chn id的高
     *
     * @retval 非-1      成功
     * @retval -1        失败
     */
    int GetChnId(Sdk_Object_e object, int for_bind, int width, int height);

  private:
    int m_pipe_id = -1;
    int init_ = 0;
    int video_format = srpy_PIXEL_FORMAT_NV12;
    int last_frame_id = 0;
    vp_param_t m_vp_param = {0};
    x3_modules_info_t m_x3_modules_info;
};

} // namespace srpy_cam

#endif
