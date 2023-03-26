/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-03-05 16:55:13
 * @LastEditTime: 2023-03-05 16:56:44
 ***************************************************************************/
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
    return new SrPyCamera();
}

void sp_release_vio_module(void *obj)
{
    if (obj != NULL)
    {
        auto sp = static_cast<SrPyCamera *>(obj);
        delete sp; // safe
    }
}

int sp_open_camera(void *obj, const int pipe_id, int chn_num, int *input_width, int *input_height)
{
    if (obj != NULL)
    {
        auto sp = static_cast<SrPyCamera *>(obj);
        int width[CAMERA_CHN_NUM] = {0};
        int height[CAMERA_CHN_NUM] = {0};
        memcpy(width, input_width, sizeof(int) * chn_num);
        memcpy(height, input_height, sizeof(int) * chn_num);
        if (chn_num < (CAMERA_CHN_NUM - 1))
        {
            // set 0 means default size
            width[chn_num] = 0;
            height[chn_num] = 0;
            chn_num++;
        }
        return sp->OpenCamera(pipe_id, 1, 30, chn_num, width, height);
    }
    return -1;
}

int sp_open_vps(void *obj, const int pipe_id, int chn_num, int proc_mode, int src_width, int src_height, int *dst_width, int *dst_height, int *crop_x, int *crop_y, int *crop_width, int *crop_height, int *rotate)
{
    if (obj != NULL)
    {
        auto sp = static_cast<SrPyCamera *>(obj);
        return sp->OpenVPS(pipe_id, chn_num, proc_mode, src_width, src_height, dst_width, dst_height, crop_x, crop_y, crop_width, crop_height, rotate);
    }
    return -1;
}

int sp_vio_close(void *obj)
{
    if (obj != NULL)
    {
        auto sp = static_cast<SrPyCamera *>(obj);
        return sp->CloseCamera();
    }
    return -1;
}

int sp_vio_get_frame(void *obj, char *frame_buffer, int width, int height, const int timeout)
{
    if (obj != NULL && frame_buffer != NULL)
    {
        auto sp = static_cast<SrPyCamera *>(obj);
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

int sp_vio_get_raw(void *obj, char *frame_buffer, int width, int height, const int timeout)
{
    if (obj != NULL && frame_buffer != NULL)
    {
        auto sp = static_cast<SrPyCamera *>(obj);
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

int sp_vio_get_yuv(void *obj, char *frame_buffer, int width, int height, const int timeout)
{
    if (obj != NULL && frame_buffer != NULL)
    {
        auto sp = static_cast<SrPyCamera *>(obj);
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

int sp_vio_set_frame(void *obj, void *frame_buffer, int size)
{
    if (obj != NULL && frame_buffer != NULL)
    {
        auto sp = static_cast<SrPyCamera *>(obj);
        auto module_enum = static_cast<DevModule>(SP_DEV_IPU);
        ImageFrame temp_image;
        temp_image.data[0] = static_cast<uint8_t *>(frame_buffer);
        // printf("input buffer:0x%x\n",frame_buffer);
        temp_image.data_size[0] = size;
        return sp->SetImageFrame(&temp_image, module_enum);
    }
    return -1;
}