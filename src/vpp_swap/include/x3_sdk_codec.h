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

#ifndef _X3_SDK_CODEC_H_
#define _X3_SDK_CODEC_H_

#include <semaphore.h>
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

#include "x3_vio_vdec.h"
#include "x3_vio_venc.h"
#include "x3_vio_vp.h"

using namespace std;

namespace srpy_cam
{

#define EN_VPS_DS2     2
#define EN_VPS_US      5
#define COM_FRAME_RATE 30

enum {
    TYPE_H264 = 1,
    TYPE_H265,
    TYPE_JPEG
};

enum {
    MODE_VENC,
    MODE_VDEC,
    MODE_INVAL
};

typedef struct codec_av_param {
    int count;
    int videoIndex;
    int bufSize;
    int firstPacket;
} codec_av_param_t;

struct x3_codec_param_t {
    bool is_quit = false;
    const char *fname;
    int frame_count;
    sem_t read_done;
    struct {
        int alloc_nums = 0;
        int codec_type = MODE_INVAL;
        vp_param_t *vp_param = nullptr;
        codec_av_param_t *av_param = nullptr;
    };
};

class VPPCodec
{
  public:
    using get_dec_name_ptr = function<string()>;

  private:
#define ENC_BUF_NUMS 7
#define DEC_BUF_NUMS 7

  public:
    VPPCodec(int video_chn, int type,
              int width, int height,
              int bits = 8000, int dec_mode = 1)
    {
        m_chn = video_chn;
        m_type = type;
        m_width = width;
        m_height = height;
        m_enc_bits = bits;
        m_dec_mode = static_cast<VIDEO_MODE_E>(dec_mode);
    }

    VPPCodec() {}

    virtual ~VPPCodec() = default;

  public:
    int x3_venc_bind_vps();

    int x3_venc_unbind_vps();

    int x3_venc_init();

    int x3_venc_deinit()
    {
        if (m_enc_param) {
            x3_codec_vp_deinit(m_enc_param.get());
        }
        m_is_enc_file = false;
        m_enc_param.reset();
        return HB_VENC_DestroyChn(m_chn);
    }

    int x3_venc_start()
    {
        VENC_RECV_PIC_PARAM_S pstRecvParam;
        memset(&pstRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
        pstRecvParam.s32RecvPicNum = 0; // unchangable

        return HB_VENC_StartRecvFrame(m_chn, &pstRecvParam);
    }

    int x3_venc_stop()
    {
        return HB_VENC_StopRecvFrame(m_chn);
    }

    int x3_venc_file(char *addr, int32_t size);

    ImageFrame *x3_venc_get_frame();

    int x3_venc_put_frame(ImageFrame *frame);

    int x3_vdec_init();

    int x3_vdec_deinit()
    {
        return HB_VDEC_DestroyChn(m_chn);
    }

    int x3_vdec_start()
    {
        return HB_VDEC_StartRecvStream(m_chn);
    }

    int x3_vdec_stop()
    {
        return HB_VDEC_StopRecvStream(m_chn);
    }

    int x3_vdec_restart();

    void x3_do_sync_decoding(void *param);

    ImageFrame *x3_vdec_get_frame();

    int x3_vdec_put_frame(ImageFrame *frame);

    int x3_codec_vp_init(x3_codec_param_t *p_param);

    int x3_codec_vp_deinit(x3_codec_param_t *p_param);

  private:
    void VencChnAttrInit(VENC_CHN_ATTR_S *pVencChnAttr, PAYLOAD_TYPE_E p_enType,
                         int p_Width, int p_Height, PIXEL_FORMAT_E pixFmt);

    void VdecChnAttrInit(VDEC_CHN_ATTR_S *pVdecChnAttr, PAYLOAD_TYPE_E p_enType,
                         VIDEO_MODE_E p_mode, int p_Width, int p_Height);

    int x3_av_open_stream(x3_codec_param_t *p_param,
                          AVFormatContext **p_avContext, AVPacket *p_avpacket);

  protected:
    std::atomic_flag m_vp_inited = ATOMIC_FLAG_INIT;

  public:

    int m_chn = 0;

    int m_width = 1920;

    int m_height = 1080;

  private:

    /* default H264(1) H265(2) JPEG(3) */
    int m_type = TYPE_H264;

    int m_enc_bits = 8000;

    bool m_is_enc_file = false;

    unique_ptr<x3_codec_param_t> m_enc_param = nullptr;

    unique_ptr<ImageFrame> m_enc_frame = nullptr;

    VIDEO_STREAM_S m_enc_pstStream;

    std::mutex m_enc_mtx;

    VIDEO_MODE_E m_dec_mode = VIDEO_MODE_FRAME;

    unique_ptr<ImageFrame> m_dec_frame = nullptr;

    VIDEO_FRAME_S m_dec_pstFrame;

    std::mutex m_dec_mtx;
};

class VPPEncode
{
  public:
    VPPEncode() {}

    virtual ~VPPEncode() = default;

    int do_encoding(int video_chn, int type, int width,
                    int height, int bits = 8000);

    /// default 1920 * 1080 -- H264 -- chn0 -- bits = 8000
    int do_encoding();

    int undo_encoding();

    ImageFrame *get_frame();

    ImageFrame *get_file_frame();

    int put_frame(ImageFrame *frame);

    int encode_file(char *addr, int32_t size);

  public:
    std::unique_ptr<VPPCodec> m_enc_obj = nullptr;

  private:
    atomic_flag m_enc_inited = ATOMIC_FLAG_INIT;
};

class VPPDecode
{
  public:
    VPPDecode() {}

    virtual ~VPPDecode() = default;

  public:
    int do_decoding(const char *file_name, int video_chn, int type, int width,
                    int height, int *frame_cnt, int dec_mode = 1);

    /// default 1920 * 1080 -- H264 -- chn0 -- mode: frame
    int do_decoding();

    int undo_decoding();

    void decode_func(void *param);

    ImageFrame *get_frame();

    int send_frame(int chn, void *addr, int size, int eos);

    int put_frame(ImageFrame *frame);

  public:
    std::unique_ptr<VPPCodec> m_dec_obj = nullptr;

  private:
    atomic_flag m_dec_inited = ATOMIC_FLAG_INIT;

    unique_ptr<x3_codec_param_t> m_decode_param = nullptr;

    string m_dec_file;

    atomic_flag m_start_once = ATOMIC_FLAG_INIT;

    shared_ptr<thread> m_running_thread;
};

}; // namespace srpy_cam

#endif
