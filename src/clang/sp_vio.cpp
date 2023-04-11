#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>

#include "x3_sdk_camera.h"

#include "sp_vio.h"

using namespace srpy_cam;

void *sp_init_vio_module()
{
    return new VPPCamera();
}

void sp_release_vio_module(void *obj)
{
    if (obj != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        delete sp; // safe
    }
}

int32_t sp_open_camera(void *obj, const int32_t pipe_id, const int32_t video_index, int32_t chn_num, int32_t *input_width, int32_t *input_height)
{
    if (obj != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        int32_t width[CAMERA_CHN_NUM] = {0};
        int32_t height[CAMERA_CHN_NUM] = {0};
        memcpy(width, input_width, sizeof(int) * chn_num);
        memcpy(height, input_height, sizeof(int) * chn_num);
        if (chn_num < (CAMERA_CHN_NUM - 1))
        {
            // set 0 means default size
            width[chn_num] = 0;
            height[chn_num] = 0;
            chn_num++;
        }
        return sp->OpenCamera(pipe_id, video_index, 30, chn_num, width, height);
    }
    return -1;
}

int32_t sp_open_vps(void *obj, const int32_t pipe_id, int32_t chn_num, int32_t proc_mode, int32_t src_width, int32_t src_height, int32_t *dst_width, int32_t *dst_height, int32_t *crop_x, int32_t *crop_y, int32_t *crop_width, int32_t *crop_height, int32_t *rotate)
{
    if (obj != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        return sp->OpenVPS(pipe_id, chn_num, proc_mode, src_width, src_height, dst_width, dst_height, crop_x, crop_y, crop_width, crop_height, rotate);
    }
    return -1;
}

int32_t sp_vio_close(void *obj)
{
    if (obj != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        return sp->CloseCamera();
    }
    return -1;
}

int32_t sp_vio_get_frame(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout)
{
    if (obj != NULL && frame_buffer != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        auto module_enum = static_cast<DevModule>(SP_DEV_IPU);
        ImageFrame *temp_ptr = new ImageFrame;
        if (!sp->GetImageFrame(temp_ptr, module_enum, width, height, timeout))
        {
            memcpy(frame_buffer, temp_ptr->data[0], temp_ptr->data_size[0]);
            if (temp_ptr->plane_count > 1)
                memcpy(frame_buffer + temp_ptr->data_size[0], temp_ptr->data[1], temp_ptr->data_size[1]);
            sp->ReturnImageFrame(temp_ptr, module_enum, width, height); // delete temp_ptr->frame_info
        }
        else
        {
            delete temp_ptr;
            return -1;
        }
        delete temp_ptr;
        return 0;
    }
    return -1;
}

int32_t sp_vio_get_raw(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout)
{
    if (obj != NULL && frame_buffer != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        auto module_enum = static_cast<DevModule>(Dev_SIF);
        ImageFrame *temp_ptr = new ImageFrame;
        if (!sp->GetImageFrame(temp_ptr, module_enum, width, height, timeout))
        {
            memcpy(frame_buffer, temp_ptr->data[0], temp_ptr->data_size[0]);
            if (temp_ptr->plane_count > 1)
                memcpy(frame_buffer + temp_ptr->data_size[0], temp_ptr->data[1], temp_ptr->data_size[1]);
            sp->ReturnImageFrame(temp_ptr, module_enum, width, height); // delete temp_ptr->frame_info
        }
        else
        {
            delete temp_ptr;
            return -1;
        }
        delete temp_ptr;
        return 0;
    }
    return -1;
}

int32_t sp_vio_get_yuv(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout)
{
    if (obj != NULL && frame_buffer != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        auto module_enum = static_cast<DevModule>(Dev_ISP);
        ImageFrame *temp_ptr = new ImageFrame;
        if (!sp->GetImageFrame(temp_ptr, module_enum, width, height, timeout))
        {
            memcpy(frame_buffer, temp_ptr->data[0], temp_ptr->data_size[0]);
            if (temp_ptr->plane_count > 1)
                memcpy(frame_buffer + temp_ptr->data_size[0], temp_ptr->data[1], temp_ptr->data_size[1]);
            sp->ReturnImageFrame(temp_ptr, module_enum, width, height); // delete temp_ptr->frame_info
        }
        else
        {
            delete temp_ptr;
            return -1;
        }
        delete temp_ptr;
        return 0;
    }
    return -1;
}

int32_t sp_vio_set_frame(void *obj, void *frame_buffer, int32_t size)
{
    if (obj != NULL && frame_buffer != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        auto module_enum = static_cast<DevModule>(SP_DEV_IPU);
        ImageFrame temp_image;
        temp_image.data[0] = static_cast<uint8_t *>(frame_buffer);
        // printf("input buffer:0x%x\n",frame_buffer);
        temp_image.data_size[0] = size;
        return sp->SetImageFrame(&temp_image, module_enum);
    }
    return -1;
}