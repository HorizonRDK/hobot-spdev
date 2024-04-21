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

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <atomic>
#include <cstdbool>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

#include <functional>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "utils_log.h"
#include "x3_sdk_display.h"

using namespace std;

namespace srpy_cam
{

/// Class VPPDisplay related

#define VOT_ARGB_BYTES 4

#define FONT_HZK16_FILE "/etc/vio/HZK16"
#define FONT_ASC16_FILE "/etc/vio/ASC16"

#define FONT_MASK_80 0x80
#define FONT_MASK_40 0x40
#define FONT_MASK_20 0x20
#define FONT_MASK_10 0x10
#define FONT_MASK_08 0x08
#define FONT_MASK_04 0x04
#define FONT_MASK_02 0x02
#define FONT_MASK_01 0x01

#define FONT_INTERVAL_CN_WORD_CNT 94
#define FONT_CN_WORD_START_ENCODE 0xa0
#define FONT_CN_WORD_BYTES 32
#define FONT_EN_WORD_BYTES 16

#define FONT_WORD_HEIGHT 16
#define FONT_ONE_ENCODE_WIDTH 8
#define FONT_CN_ENCODE_NUM 2
#define FONT_EN_ENCODE_NUM 1
#define FONT_CN_WORD_WIDTH \
    (FONT_CN_ENCODE_NUM * FONT_ONE_ENCODE_WIDTH)
#define FONT_EN_WORD_WIDTH \
    (FONT_EN_ENCODE_NUM * FONT_ONE_ENCODE_WIDTH)
#define FONT_HARD_PIXEL_BITS 4
#define ONE_BYTE_BIT_CNT 8

static void draw_dot(uint8_t *frame, int x, int y, int color,
                     int screen_width, int screen_height)
{
    int pbyte = VOT_ARGB_BYTES;

    if (x >= screen_width || y >= screen_height)
        return;

    frame += ((y * screen_width) + x) * pbyte;
    while (pbyte) {
        pbyte--;
        frame[pbyte] = (color >> (pbyte * 8)) & 0xFF;
    }
}
static void draw_hline(uint8_t *frame, int x0, int x1, int y, int color,
                       int screen_width, int screen_height)
{
    int xi, xa;

    xi = (x0 < x1) ? x0 : x1;
    xa = (x0 > x1) ? x0 : x1;
    while (xi <= xa) {
        draw_dot(frame, xi, y, color, screen_width, screen_height);
        xi++;
    }
}

static void draw_vline(uint8_t *frame, int x, int y0, int y1, int color,
                       int screen_width, int screen_height)
{
    int yi, ya;

    yi = (y0 < y1) ? y0 : y1;
    ya = (y0 > y1) ? y0 : y1;
    while (yi <= ya) {
        draw_dot(frame, x, yi, color, screen_width, screen_height);
        yi++;
    }
}

static void draw_rect(uint8_t *frame, int x0, int y0, int x1, int y1, int color,
                      int fill, int screen_width, int screen_height, int line_width)
{
    int xi, xa, yi, ya;
    int i = 0;

    xi = (x0 < x1) ? x0 : x1; // left
    xa = (x0 > x1) ? x0 : x1; // right
    yi = (y0 < y1) ? y0 : y1; // bottom
    ya = (y0 > y1) ? y0 : y1; // top
    if (fill) {
        while (yi <= ya) {
            draw_hline(frame, xi, xa, yi, color, screen_width, screen_height);
            yi++;
        }
    } else {
        if (ya < line_width || yi > (screen_height - line_width) ||
            xi > (screen_width - line_width) ||
            xa > (screen_width - line_width)) {
            LOGD_print("point is 0, x0=%d, y0=%d, x1=%d, y1=%d,"
                "screen_width=%d, screen_height=%d, line_width= %d, return\n",
                x0, y0, x1, y1,
                screen_width, screen_height, line_width);
            return;
        }
        for (i = 0; i < line_width; i++) {
            draw_hline(frame, xi, xa, yi + i, color, screen_width, screen_height);
            draw_hline(frame, xi, xa, ya - i, color, screen_width, screen_height);
            draw_vline(frame, xi + i, yi, ya, color, screen_width, screen_height);
            draw_vline(frame, xa + i, yi, ya, color, screen_width, screen_height);
        }
    }
}

static void osd_draw_word_row(uint8_t *addr, uint32_t width,
    uint32_t line_width, uint32_t color)
{
    uint32_t addr_offset;
    uint32_t m, w, i;

    for (m = 0; m < line_width; m++) {
        for (w = 0; w < line_width; w++) {
            addr_offset = ((width * m) + w) * VOT_ARGB_BYTES;
            for (i = 0; i < VOT_ARGB_BYTES; i++) {
                addr[addr_offset + i] = (color >> (i * 8)) & 0xff;
            }
        }
    }
}

// draw chinese word
static int32_t osd_draw_cn_word(uint8_t *addr, uint32_t width,
    uint32_t line_width, uint32_t color, uint8_t *cn_word)
{
    FILE *file;
    uint8_t flag;
    uint64_t offset;
    uint8_t *addr_word, *addr_src;
    uint8_t buffer[32];
    uint8_t key[8] = {FONT_MASK_80, FONT_MASK_40, FONT_MASK_20,
                      FONT_MASK_10, FONT_MASK_08, FONT_MASK_04,
                      FONT_MASK_02, FONT_MASK_01};
    uint32_t row_bytes;
    uint32_t k, j, i;
    size_t size;

    file = fopen(FONT_HZK16_FILE, "rb");
    if (file == NULL) {
        LOGE_print("open HZK16 file fail %d %s\n", errno, strerror(errno));
        return -1;
    }

    offset = ((FONT_INTERVAL_CN_WORD_CNT *
        ((uint64_t)cn_word[0] - FONT_CN_WORD_START_ENCODE - 1u)) +
        ((uint64_t)cn_word[1] - FONT_CN_WORD_START_ENCODE) - 1u)
        * FONT_CN_WORD_BYTES;
    (void)fseek(file, (int64_t)offset, SEEK_SET);
    size = fread((void *)buffer, 1, FONT_CN_WORD_BYTES, file);
    if (size != FONT_CN_WORD_BYTES) {
        LOGE_print("fread font file:%s error\n", FONT_HZK16_FILE);
        (void)fclose(file);
        return -1;
    }

    row_bytes = FONT_CN_WORD_WIDTH / ONE_BYTE_BIT_CNT;
    addr_src = addr;

    for (k = 0; k < FONT_WORD_HEIGHT; k++) {
        addr_word = addr_src;
        for (j = 0; j < row_bytes; j++) {
            for (i = 0; i < ONE_BYTE_BIT_CNT; i++) {
                flag = buffer[(k * row_bytes) + j] & key[i];
                if (flag != 0u) {
                    osd_draw_word_row(addr_word, width, line_width, color);
                }
                addr_word = &addr_word[line_width * VOT_ARGB_BYTES];
            }
        }
        addr_src = &addr_src[width * line_width * VOT_ARGB_BYTES];
    }

    (void)fclose(file);

    return 0;
}

// draw endlish word
static int32_t osd_draw_en_word(uint8_t *addr, uint32_t width,
    uint32_t line_width, uint32_t color, uint8_t en_word)
{
    FILE *file;
    uint8_t flag;
    uint32_t offset;
    uint8_t *addr_word, *addr_src;
    uint8_t buffer[16];
    uint8_t key[8] = {FONT_MASK_80, FONT_MASK_40, FONT_MASK_20,
                      FONT_MASK_10, FONT_MASK_08, FONT_MASK_04,
                      FONT_MASK_02, FONT_MASK_01};
    uint32_t k, i;
    size_t size;

    file = fopen(FONT_ASC16_FILE, "rb");
    if (file == NULL) {
        LOGE_print("open ASC16 file fail\n");
        return -1;
    }

    offset = (uint32_t)en_word * FONT_EN_WORD_BYTES;
    (void)fseek(file, (int32_t)offset, SEEK_SET);
    size = fread((void *)buffer, 1, FONT_EN_WORD_BYTES, file);
    if (size != FONT_EN_WORD_BYTES) {
        LOGE_print("fread font file:%s error\n", FONT_ASC16_FILE);
        (void)fclose(file);
        return -1;
    }

    addr_src = addr;

    for (k = 0; k < FONT_WORD_HEIGHT; k++) {
        addr_word = addr_src;
        for (i = 0; i < ONE_BYTE_BIT_CNT; i++) {
            flag = buffer[k] & key[i];
            if (flag != 0u) {
                osd_draw_word_row(addr_word, width, line_width, color);
            }
            addr_word = &addr_word[line_width * VOT_ARGB_BYTES];
        }
        addr_src = &addr_src[width * line_width * VOT_ARGB_BYTES];
    }

    (void)fclose(file);

    return 0;
}

static int32_t draw_word(uint8_t *addr, int x, int y, char *str, int width, int color, int line_width)
{
    uint32_t str_len, i, word_offs = 1u;
    uint8_t cn_word[2], en_word;
    int32_t ret;
    uint32_t addr_offset;

    if (addr == NULL) {
        LOGE_print("draw word addr was NULL\n");
        return -1;
    }

    str_len = (uint32_t)strlen(str);

    addr_offset = ((y * width) + x) * VOT_ARGB_BYTES;
    addr = &(addr)[addr_offset];

    for (i = 0; i < str_len; i += word_offs) {
        if (str[i] >= (uint8_t)FONT_CN_WORD_START_ENCODE) {
            cn_word[0] = str[i];
            cn_word[1] = str[i + 1u];
            if (cn_word[1] == '\0') {
                word_offs = FONT_CN_ENCODE_NUM;
                continue;
            }

            ret = osd_draw_cn_word(addr, width, line_width, color, cn_word);
            if (ret < 0) {
                return ret;
            }
            word_offs = FONT_CN_ENCODE_NUM;
            addr = &addr[line_width * FONT_CN_WORD_WIDTH * VOT_ARGB_BYTES];
        }
        if (str[i] < (uint8_t)FONT_CN_WORD_START_ENCODE) {
            en_word = str[i];

            ret = osd_draw_en_word(addr, width, line_width, color, en_word);
            if (ret < 0) {
                return ret;
            }
            word_offs = FONT_EN_ENCODE_NUM;
            addr = &addr[line_width * FONT_EN_WORD_WIDTH * VOT_ARGB_BYTES];
        }
    }
    return 0;
}


char* find_resolution(const char* filename, int width, int height) {
    FILE* file = fopen(filename, "r");
    if (file == NULL) {
        LOGE_print("Error opening file");
        return NULL;
    }

    char line[256];
    while (fgets(line, sizeof(line), file) != NULL) {
        int w, h;
        char format[10];
        int matched = sscanf(line, "%*[^:]:%dx%d%[^-]-%*d", &w, &h, format);

        if (matched == 3 && w == width && h == height) {
            // Check if the resolution is 640x480 and skip if it is
            if (w == 640 && h == 480) {
                // Skip resolutions with 60HZ refresh rate
                if (strstr(line, "p-60") != NULL) {
                    continue;
                }
            }

            fclose(file);
            return strdup(line);
        }
    }

    fclose(file);
    return NULL;  // Return NULL if no match is found
}

int write_to_node(const char* node_path, const char* content) {
    FILE* node = fopen(node_path, "w");
    if (node == NULL) {
        LOGE_print("Error opening node");
        return -1;
    }

    fputs(content, node);

    fclose(node);
    return 0;
}

static void vot_set_upscale_attr(VOT_UPSCALE_ATTR_S *upscale_attr, int width, int height)
{
    float k_up, k_up_w, k_up_h;

    k_up_w = (float)width / (float)upscale_attr->src_width;
    k_up_h = (float)height / (float)upscale_attr->src_height;
    k_up = (k_up_w > k_up_h) ? k_up_h : k_up_w;
    upscale_attr->tgt_width = upscale_attr->src_width * k_up;
    upscale_attr->tgt_height = upscale_attr->src_height * k_up;
}

int VPPDisplay::x3_vot_init(int chn = 0, int width = 1920, int height = 1080,
            int vot_intf = VOT_OUTPUT_1080P30, int vot_out_mode = HB_VOT_OUTPUT_BT1120,
            int chn_width = 1920, int chn_height = 1080)
{
    int ret = 0;
    VOT_PUB_ATTR_S devAttr = {0};
    VOT_VIDEO_LAYER_ATTR_S stLayerAttr = {0};
    VOT_CHN_ATTR_S stChnAttr = {0};
    VOT_CROP_INFO_S cropAttrs = {0};
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    long int screensize = 0;
    VOT_CHN_ATTR_EX_S extern_attr = {0};
    VOT_UPSCALE_ATTR_S upscale_attr = {0};
    int fb0_fd = 0;

    char *buffer_ = NULL;
    int image_data_size_ = chn_width * chn_height * 3 / 2;

    char fb_str[10];
    int init_layer = 0;

    if ((chn < 0) || (chn >= VOT_LAYER_NUM)) {
        LOGE_print("only can set [0, %d] chn\n", VOT_LAYER_NUM - 1);
        goto err;
    }

    if (!m_disp_inited.test_and_set()) {
        init_layer = 1;
        m_width = width;
        m_height = height;
        m_vot_intf = vot_intf;
        m_vot_out_mode = vot_out_mode;

        devAttr.enIntfSync = static_cast<VOT_INTF_SYNC_E>(m_vot_intf);
        devAttr.u32BgColor = 0x8080;
        devAttr.enOutputMode = static_cast<VOT_OUTPUT_MODE_E>(m_vot_out_mode);


        fb0_fd = open("/dev/fb0", O_RDWR);
        /* Get variable screen information */
        if (ioctl(fb0_fd, FBIOGET_VSCREENINFO, &vinfo)) {
            LOGE_print("Error reading variable information.\n");
            goto err3;
        }
        //Store origin crop prop(w,h)
        origin_src_height = vinfo.yres_virtual;
        origin_src_width = vinfo.xres_virtual;
        origin_crop_height = vinfo.yres;
        origin_crop_width = vinfo.xres;
        //If input width and height equal current screen size
        //Use fb var timing to config hdmi and iar
        if(m_width != vinfo.xres && m_height != vinfo.yres){
            const char* fb_modes = "/sys/class/graphics/fb0/modes";
            const char* fb_mode = "/sys/class/graphics/fb0/mode";
            char* result = find_resolution(fb_modes, m_width, m_height);
            if (result != NULL) {
                LOGI_print("Matching resolution: %s", result);
                // Write the matching resolution to the specified node
                if (write_to_node(fb_mode, result) == 0) {
                    LOGI_print("Successfully wrote to %s\n", fb_mode);
                } else {
                    LOGE_print("Error writing to %s\n", fb_mode);
                }

                free(result);
            } else {
                LOGE_print("No matching resolution found.\n");
            }
        }
        if (ioctl(fb0_fd, FBIOGET_VSCREENINFO, &vinfo)) {
            LOGE_print("Error reading variable information.\n");
            goto err3;
        }

        devAttr.enIntfSync = VO_OUTPUT_USER;
        devAttr.stSyncInfo.hfp =vinfo.right_margin;
        devAttr.stSyncInfo.hbp =vinfo.left_margin;
        devAttr.stSyncInfo.vfp =vinfo.lower_margin;
        devAttr.stSyncInfo.vbp =vinfo.upper_margin;
        devAttr.stSyncInfo.hs = vinfo.hsync_len;
        devAttr.stSyncInfo.vs = vinfo.vsync_len;
        devAttr.stSyncInfo.vfp_cnt = 0X0;
        devAttr.stSyncInfo.pixel_clk = 1000000000000 / vinfo.pixclock;
        devAttr.stSyncInfo.width = m_width;
        devAttr.stSyncInfo.height = m_height;


        //LOGE_print("ori w:%d,ori h:%d\n",origin_src_width,origin_src_width);

        ret = HB_VOT_SetPubAttr(0, &devAttr);
        if (ret) {
            LOGE_print("HB_VOT_SetPubAttr failed\n");
            goto err;
        }

        ret = HB_VOT_Enable(0);
        if (ret) {
            LOGE_print("HB_VOT_Enable failed.\n");
            goto err;
        }

        ret = HB_VOT_GetVideoLayerAttr(0, &stLayerAttr);
        if (ret) {
            LOGE_print("HB_VOT_GetVideoLayerAttr failed.\n");
            goto err1;
        }

        stLayerAttr.stImageSize.u32Width = m_width;
        stLayerAttr.stImageSize.u32Height = m_height;

        stLayerAttr.panel_type = 0;
        stLayerAttr.rotate = 0;
        stLayerAttr.dithering_flag = 0;
        stLayerAttr.dithering_en = 0;
        stLayerAttr.gamma_en = 0;
        stLayerAttr.hue_en = 0;
        stLayerAttr.sat_en = 0;
        stLayerAttr.con_en = 0;
        stLayerAttr.bright_en = 0;
        stLayerAttr.theta_sign = 0;
        stLayerAttr.contrast = 0;
        stLayerAttr.theta_abs = 0;
        stLayerAttr.saturation = 0;
        stLayerAttr.off_contrast = 0;
        stLayerAttr.off_bright = 0;
        stLayerAttr.user_control_disp = 1;
        stLayerAttr.user_control_disp_layer1 = 1;
        stLayerAttr.big_endian = 0;
        ret = HB_VOT_SetVideoLayerAttr(0, &stLayerAttr);
        if (ret) {
            LOGE_print("HB_VOT_SetVideoLayerAttr failed.\n");
            goto err1;
        }

        ret = HB_VOT_EnableVideoLayer(0);
        if (ret) {
            LOGE_print("HB_VOT_EnableVideoLayer failed.\n");
            // for sdb ubuntu, this error can be ignored
            // goto err1;
        }
    }

    m_vot_chn[chn] = chn;
    m_chn_width[chn] = chn_width;
    m_chn_height[chn] = chn_height;
    // 0 is highest priority, graph layer need higher than video layer
    stChnAttr.u32Priority = (chn >= 2) ? (chn - 2) : (chn + 2);
    stChnAttr.s32X = 0;
    stChnAttr.s32Y = 0;
    stChnAttr.u32SrcWidth = m_chn_width[chn];
    stChnAttr.u32SrcHeight = m_chn_height[chn];
    stChnAttr.u32DstWidth = m_chn_width[chn];
    stChnAttr.u32DstHeight = m_chn_height[chn];
    ret = HB_VOT_SetChnAttr(0, m_vot_chn[chn], &stChnAttr);
    if (ret) {
        LOGE_print("HB_VOT_SetChnAttr failed: %d\n", ret);
        goto err2;
    }

    if (m_vot_chn[chn] >= VOT_GRAPH_LAYER_NUM) {
        ret = HB_VOT_GetChnAttrEx(0, m_vot_chn[chn], &extern_attr);
        if (ret) {
            LOGE_print("HB_VOT_GetChnAttrEx failed.\n");
            // break;
        }
        extern_attr.format = PIXEL_FORMAT_ARGB8888;
        ret = HB_VOT_SetChnAttrEx(0, m_vot_chn[chn], &extern_attr);
        if (ret) {
            LOGE_print("HB_VOT_SetChnAttrEx failed.\n");
            // break;
        }
    }

    cropAttrs.u32Width = stChnAttr.u32DstWidth;   // - stChnAttr.s32X;
    cropAttrs.u32Height = stChnAttr.u32DstHeight; //- stChnAttr.s32Y;
    ret = HB_VOT_SetChnCrop(0, m_vot_chn[chn], &cropAttrs);
    if (ret < 0) {
        LOGE_print("HB_VOT_SetChnCrop failed: %d\n", ret);
    }

    ret = HB_VOT_EnableChn(0, m_vot_chn[chn]);
    if (ret) {
        LOGE_print("HB_VOT_EnableChn failed: %d\n", ret);
        goto err2;
    }

    if (m_vot_chn[chn] < 2)
    {
        buffer_ = (char *)malloc(image_data_size_);
        memset(buffer_, 0x1b, chn_width * chn_height);
        for (uint32_t i = chn_width * chn_height; i < image_data_size_; i+=2) {
            buffer_[i] = 0x80;
            buffer_[i+1] = 0x80;
        }

        static VOT_FRAME_INFO_S stFrame;
        stFrame.addr = buffer_;
        stFrame.size = image_data_size_;
        HB_VOT_SendFrame(0, m_vot_chn[chn], &stFrame, -1);
    }

    upscale_attr.upscale_en = 0;
    upscale_attr.pos_x = 0;
    upscale_attr.pos_y = 0;
    upscale_attr.src_width = m_chn_width[chn];
    upscale_attr.src_height = m_chn_height[chn];
    vot_set_upscale_attr(&upscale_attr, m_width, m_height);
    if ((m_chn_width[chn] != m_width) || (m_chn_height[chn] != m_height)) {
        upscale_attr.upscale_en = 1;
    }
    ret = HB_VOT_SetVideoLayerUpScale(0, &upscale_attr);
    if (ret != 0) {
        LOGE_print("Error: failed to HB_VOT_SetVideoLayerUpScale.\n");
    }

    if (chn >= VOT_GRAPH_LAYER_NUM) {
        snprintf(fb_str, sizeof(fb_str), "/dev/fb%d", chn - VOT_GRAPH_LAYER_NUM);
        m_fbfd[chn - VOT_GRAPH_LAYER_NUM] = open(fb_str, O_RDWR);
        if (m_fbfd[chn - VOT_GRAPH_LAYER_NUM] < 0) {
            LOGE_print("Error: cannot open framebuffer device(%s).\n", fb_str);
            goto err2;
        }
        /* Get fixed screen information */
        if (ioctl(m_fbfd[chn - VOT_GRAPH_LAYER_NUM], FBIOGET_FSCREENINFO, &finfo)) {
            LOGE_print("Error reading fixed information.\n");
            goto err3;
        }

        /* Get variable screen information */
        if (ioctl(m_fbfd[chn - VOT_GRAPH_LAYER_NUM], FBIOGET_VSCREENINFO, &vinfo)) {
            LOGE_print("Error reading variable information.\n");
            goto err3;
        }

        screensize = m_chn_width[chn] * m_chn_height[chn] * 4;

        m_fbp[chn - VOT_GRAPH_LAYER_NUM] =
            (uint8_t *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, m_fbfd[chn - VOT_GRAPH_LAYER_NUM], 0);
        if (m_fbp[chn - VOT_GRAPH_LAYER_NUM] == MAP_FAILED) {
            LOGE_print("Error: failed to map framebuffer device to memory.\n");
            goto err3;
        }
    }

    return ret;
err3:
    if (m_fbfd[chn - VOT_GRAPH_LAYER_NUM] >= 0) {
        close(m_fbfd[chn - VOT_GRAPH_LAYER_NUM]);
    }
    m_fbfd[chn - VOT_GRAPH_LAYER_NUM] = -1;
    m_fbp[chn - VOT_GRAPH_LAYER_NUM] = NULL;
    HB_VOT_DisableChn(0, chn);
err2:
    if (init_layer) {
        HB_VOT_DisableVideoLayer(0);
    }
err1:
    if (init_layer) {
        HB_VOT_Disable(0);
    }
err:
    return ret;
}




int VPPDisplay::reset_iar_crop(void){
    channel_base_cfg_t channel_base_cfg[1] = {0};
	uint32_t ret = 0;

    int iar_fd = open(IAR_DEV_PATH, O_RDWR | O_NONBLOCK, 0);
	channel_base_cfg[0].enable = 1;
	channel_base_cfg[0].channel = IAR_CHANNEL_3;
	channel_base_cfg[0].pri = 0;
	channel_base_cfg[0].width = origin_src_width;
	channel_base_cfg[0].height = origin_src_height;
	channel_base_cfg[0].buf_width = origin_src_width;
	channel_base_cfg[0].buf_height = origin_src_height;
	channel_base_cfg[0].format = 3;
	channel_base_cfg[0].alpha_sel = 0;
	channel_base_cfg[0].ov_mode = 0;
	channel_base_cfg[0].alpha_en = 1;
	channel_base_cfg[0].alpha = 255;
	channel_base_cfg[0].crop_width = origin_crop_width;
	channel_base_cfg[0].crop_height = origin_crop_height;
    ret = ioctl(iar_fd, IAR_CHANNEL_CFG, &channel_base_cfg[0]);
    if(ret){
        LOGE_print("Error: failed to send IAR_CHANNEL_CFG cmd to iar_cdev.\n");
    }
    const char* fb_modes = "/sys/class/graphics/fb0/modes";
    const char* fb_mode = "/sys/class/graphics/fb0/mode";
    char* result = find_resolution(fb_modes, origin_crop_width, origin_crop_height);
    if (result != NULL) {
        LOGI_print("Matching resolution: %s", result);
        // Write the matching resolution to the specified node
        if (write_to_node(fb_mode, result) == 0) {
            LOGI_print("Successfully wrote to %s\n", fb_mode);
        } else {
            LOGE_print("Error writing to %s\n", fb_mode);
        }

        free(result);
    } else {
        LOGE_print("No matching resolution found.\n");
    }
    return ret;
}

int VPPDisplay::x3_vot_deinit()
{
    int ret = 0;
    int screensize;

    for (int i = 0; i < VOT_LAYER_NUM; i++) {
        if (m_vot_chn[i] != -1) {
            screensize = m_chn_width[i] * m_chn_height[i] * 4;

            ret = HB_VOT_DisableChn(0, m_vot_chn[i]);
            if (ret) {
                LOGE_print("HB_VOT_DisableChn failed.\n");
            }
            m_vot_chn[i] = -1;

            if ((i >= 2) && (m_fbp[i - VOT_VIDEO_LAYER_NUM] != NULL)) {
                munmap(m_fbp[i - VOT_VIDEO_LAYER_NUM], screensize);
                m_fbp[i - VOT_VIDEO_LAYER_NUM] = NULL;
            }
            if ((i >= 2) && (m_fbfd[i - VOT_VIDEO_LAYER_NUM] >= 0)) {
                close(m_fbfd[i - VOT_VIDEO_LAYER_NUM]);
                m_fbfd[i - VOT_VIDEO_LAYER_NUM] = -1;
            }
        }
    }

    ret = HB_VOT_DisableVideoLayer(0);
    if (ret) {
        LOGE_print("HB_VOT_DisableVideoLayer failed.\n");
    }

    ret = HB_VOT_Disable(0);
    if (ret) {
        LOGE_print("HB_VOT_Disable failed.\n");
    }
    m_disp_inited.clear();
    ret = reset_iar_crop();
    if(ret) {
        LOGE_print("reset_iar_crop failed.\n");
    }
    return ret;
}

int VPPDisplay::set_img(void *addr, int size, int chn = 0)
{
    int s32Ret = 0;
    VOT_FRAME_INFO_S frame_info = {0};

    if ((chn > 1) || (chn < 0)) {
        LOGE_print("set_img only can set chn 0 or 1\n");
        return -1;
    }

    if (addr == nullptr || size == 0) {
        LOGE_print("Display set image failed, addr:%p size:%d\n", addr, size);
        return -1;
    }

    frame_info.addr = addr;
    frame_info.size = size;

    s32Ret = HB_VOT_SendFrame(0, chn, &frame_info, -1);
    return s32Ret;
}

int VPPDisplay::set_graph_rect(int x0, int y0, int x1, int y1, int chn = 2,
    int flush = 0, uint32_t color = 0xffff0000, int line_width = 4)
{
    int s32Ret = 0;

    if ((chn < 2) || (chn > 3)) {
        LOGE_print("set_img only can set chn 2 or 3n");
        return -1;
    }

    if ((m_vot_chn[chn] < 0) || (m_fbp[chn - 2] == NULL)) {
        LOGE_print("please init chn:%d first\n", chn);
        return -1;
    }

    x0 = (x0 < (m_chn_width[chn] - line_width)) ? ((x0 >= 0) ? x0 : 0) : (m_chn_width[chn] - line_width);
    y0 = (y0 < (m_chn_height[chn] - line_width)) ? ((y0 >= 0) ? y0 : 0) : (m_chn_height[chn] - line_width);
    x1 = (x1 < (m_chn_width[chn] - line_width)) ? ((x1 >= 0) ? x1 : 0) : (m_chn_width[chn] - line_width);
    y1 = (y1 < (m_chn_height[chn] - line_width)) ? ((y1 >= 0) ? y1 : 0) : (m_chn_height[chn] - line_width);

    if (flush) {
        memset(m_fbp[chn - 2], 0, m_chn_width[chn] * m_chn_height[chn] * VOT_ARGB_BYTES);
    }

    draw_rect(m_fbp[chn - 2], x0, y0, x1, y1, color, 0, m_chn_width[chn], m_chn_height[chn], line_width);

    return s32Ret;
}

int VPPDisplay::set_graph_word(int x, int y, char *str, int chn = 2,
    int flush = 0, uint32_t color = 0xffff0000, int line_width = 1)
{
    int s32Ret = 0;
    int len;

    if ((chn < 2) || (chn > 3)) {
        LOGE_print("set_img only can set chn 2 or 3n");
        return -1;
    }

    if ((m_vot_chn[chn] < 0) || (m_fbp[chn - 2] == NULL)) {
        LOGE_print("please init chn:%d first\n", chn);
        return -1;
    }

    if (str == NULL) {
        LOGE_print("string was NULL\n");
        return -1;
    }

    if ((x < 0) || (x > m_chn_width[chn]) || (y < 0) || (y > m_chn_height[chn]) ||
        ((line_width * FONT_ONE_ENCODE_WIDTH + y) > m_chn_height[chn])) {
        LOGE_print("parameter error, coordinate (%d, %d) string:%s line_width:%d\n",
            x, y, str, line_width);
        return -1;
    }
    if (((int)strlen(str) * line_width * FONT_ONE_ENCODE_WIDTH + x) > m_chn_width[chn]) {
        len = (m_chn_width[chn] - x) / (line_width * FONT_ONE_ENCODE_WIDTH);
        str[len] = '\0';
    }

    if (flush) {
        memset(m_fbp[chn - 2], 0, m_chn_width[chn] * m_chn_height[chn] * VOT_ARGB_BYTES);
    }

    s32Ret = draw_word(m_fbp[chn - 2], x, y, str, m_chn_width[chn], color, line_width);

    return s32Ret;
}

int VPPDisplay::get_video_chn()
{
    for (int i = 0; i < VOT_VIDEO_LAYER_NUM; i++) {
        if (m_vot_chn[i] >= 0) {
            return m_vot_chn[i];
        }
    }

    return -1;
}

}; // namespace srpy_cam
