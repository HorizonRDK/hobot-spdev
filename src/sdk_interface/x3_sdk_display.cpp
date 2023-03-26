/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-01-30 11:27:41
 * @LastEditTime: 2023-03-05 16:38:23
 ***************************************************************************/
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
#include "iar/iar_interface.h"

using namespace std;

namespace srpy_cam
{

/// Class SrPyDisplay related

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
            printf("========point is 0,return========\n");
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
        printf("open HZK16 file fail %d %s\n", errno, strerror(errno));
        return -1;
    }

    offset = ((FONT_INTERVAL_CN_WORD_CNT *
        ((uint64_t)cn_word[0] - FONT_CN_WORD_START_ENCODE - 1u)) +
        ((uint64_t)cn_word[1] - FONT_CN_WORD_START_ENCODE) - 1u)
        * FONT_CN_WORD_BYTES;
    (void)fseek(file, (int64_t)offset, SEEK_SET);
    size = fread((void *)buffer, 1, FONT_CN_WORD_BYTES, file);
    if (size != FONT_CN_WORD_BYTES) {
        printf("fread font file:%s error\n", FONT_HZK16_FILE);
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
        printf("open ASC16 file fail\n");
        return -1;
    }

    offset = (uint32_t)en_word * FONT_EN_WORD_BYTES;
    (void)fseek(file, (int32_t)offset, SEEK_SET);
    size = fread((void *)buffer, 1, FONT_EN_WORD_BYTES, file);
    if (size != FONT_EN_WORD_BYTES) {
        printf("fread font file:%s error\n", FONT_ASC16_FILE);
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
        printf("draw word addr was NULL\n");
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

int SrPyDisplay::x3_vot_init(int chn = 0, int width = 1920, int height = 1080,
            int vot_intf = 0, int vot_out_mode = 1,
            int chn_width = 1920, int chn_height = 1080)
{
    int ret = 0;
    char iar_cfg_file[] = "/etc/iar/iar_x3sdb_hdmi_1080p.json";
    ret = hb_disp_init_cfg(iar_cfg_file);
    if (ret < 0)
    {
        printf("hb_disp_init_cfg fail\n");
        return -1;
    }
    ret = hb_disp_start();
    if (ret < 0)
    {
        printf("display start fail, do cam&vio&display deinit.\n");
        return -1;
    }
    hb_disp_layer_on(0);
    hb_disp_layer_off(2);
    return ret;
}

int SrPyDisplay::x3_vot_deinit()
{
    int ret = 0;
    hb_disp_stop();
    hb_disp_close();

    return ret;
}

int SrPyDisplay::set_img(void *addr, int size, int chn = 0)
{
    int s32Ret = 0;

    s32Ret = hb_set_video_bufaddr(addr, NULL, NULL, NULL);

    return s32Ret;
}

int SrPyDisplay::set_graph_rect(int x0, int y0, int x1, int y1, int chn = 2,
    int flush = 0, uint32_t color = 0xffff0000, int line_width = 4)
{
    int s32Ret = 0;

    if ((chn < 2) || (chn > 3)) {
        printf("set_img only can set chn 2 or 3n");
        return -1;
    }

    if ((m_vot_chn[chn] < 0) || (m_fbp[chn - 2] == NULL)) {
        printf("please init chn:%d first\n", chn);
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

int SrPyDisplay::set_graph_word(int x, int y, char *str, int chn = 2,
    int flush = 0, uint32_t color = 0xffff0000, int line_width = 1)
{
    int s32Ret = 0;
    int len;

    if ((chn < 2) || (chn > 3)) {
        printf("set_img only can set chn 2 or 3n");
        return -1;
    }

    if ((m_vot_chn[chn] < 0) || (m_fbp[chn - 2] == NULL)) {
        printf("please init chn:%d first\n", chn);
        return -1;
    }

    if (str == NULL) {
        printf("string was NULL\n");
        return -1;
    }

    if ((x < 0) || (x > m_chn_width[chn]) || (y < 0) || (y > m_chn_height[chn]) ||
        ((line_width * FONT_ONE_ENCODE_WIDTH + y) > m_chn_height[chn])) {
        printf("parameter error, coordinate (%d, %d) string:%s line_width:%d\n",
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

int SrPyDisplay::get_video_chn()
{
    for (int i = 0; i < VOT_VIDEO_LAYER_NUM; i++) {
        if (m_vot_chn[i] >= 0) {
            return m_vot_chn[i];
        }
    }

    return -1;
}

}; // namespace srpy_cam
