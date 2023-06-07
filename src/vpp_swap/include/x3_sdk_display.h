/*
 * Horizon Robotics
 *
 * Copyright (C) 2022 Horizon Robotics Inc.
 * All rights reserved.
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _X3_VPP_DISPLAY_H_
#define _X3_VPP_DISPLAY_H_

#include <atomic>
#include <cstdbool>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>

#include "vio/hb_comm_vdec.h"
#include "vio/hb_comm_venc.h"
#include "vio/hb_common_vot.h"
#include "vio/hb_sys.h"
#include "vio/hb_vdec.h"
#include "vio/hb_venc.h"
#include "vio/hb_vio_interface.h"
#include "vio/hb_vot.h"
#include "vio/hb_vp_api.h"

#include "x3_sdk_camera.h"

using namespace std;

namespace srpy_cam
{

#define VOT_LAYER_NUM 4
#define VOT_VIDEO_LAYER_NUM 2
#define VOT_GRAPH_LAYER_NUM 2

class VPPDisplay
{
  public:
    VPPDisplay(int chn, int width, int height,
            int vot_intf = 0, int vot_out_mode = 1)
        : m_width(width),
          m_height(height),
          m_vot_intf(vot_intf),
          m_vot_out_mode(vot_out_mode) {}

    VPPDisplay() {}

    virtual ~VPPDisplay() = default;

  public:
    int x3_vot_deinit();

    int x3_vot_init(int chn, int width, int height, int vot_intf, int vot_out_mode, int chn_width, int chn_height);

    int set_img(void *addr, int size, int chn);

    int set_graph_rect(int x0, int y0, int x1, int y1, int chn,
        int flush, uint32_t color, int line_width);

    int set_graph_word(int x, int y, char *str, int chn,
        int flush, uint32_t color, int line_width);

    int get_video_chn();

  public:
    /// Vot has 2 video channels(0,1) and 2 frame buffer layers(2,3)
    int m_vot_chn[VOT_LAYER_NUM] = {-1, -1, -1, -1};

    int m_width = 1920;

    int m_height = 1080;

    int m_chn_width[VOT_LAYER_NUM];

    int m_chn_height[VOT_LAYER_NUM];

  private:
    /// default VOT_OUTPUT_1080P30
    int m_vot_intf = 0;

    /// default HB_VOT_OUTPUT_BT1120
    int m_vot_out_mode = 1;

    int m_fbfd[VOT_GRAPH_LAYER_NUM] = {-1, -1};
    uint8_t *m_fbp[VOT_GRAPH_LAYER_NUM] = {NULL, NULL};

    atomic_flag m_disp_inited = ATOMIC_FLAG_INIT;
};

}; // namespace srpy_cam

#endif
