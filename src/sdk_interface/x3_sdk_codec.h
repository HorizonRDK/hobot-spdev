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

#ifdef __cplusplus
extern "C" {
#endif

#include <libmm/hb_media_codec.h>
#include <libmm/hb_media_error.h>

#include "x3_sdk_wrap.h"

#ifdef __cplusplus
}
#endif /* extern "C" */

using namespace std;

namespace srpy_cam
{

#define EN_VPS_DS2 2
#define EN_VPS_US 5
#define COM_FRAME_RATE 30

  enum
  {
    TYPE_H264 = 1,
    TYPE_H265,
    TYPE_JPEG,
    TYPE_MJPEG
  };

  enum
  {
    MODE_VENC,
    MODE_VDEC,
    MODE_INVAL
  };

  typedef struct
  {
    bool is_quit = false;
    const char *fname;
    int frame_count;
    sem_t read_done;
    int codec_type = MODE_INVAL;
  } sr_codec_param_t;

  class SrPyEncode
  {
  public:
    SrPyEncode() {}

    virtual ~SrPyEncode() = default;

    int do_encoding(int video_chn, int type, int width,
                    int height, int bits = 8000);

    /// default 1920 * 1080 -- H264 -- chn0 -- bits = 8000
    int do_encoding();

    int undo_encoding();

    int venc_init();

    int venc_deinit();

    int venc_start();

    int venc_stop();

    ImageFrame *get_frame();

    int put_frame(ImageFrame *frame);

    int send_frame(char *addr, int32_t size);

  private:
    atomic_flag m_enc_inited = ATOMIC_FLAG_INIT;
    media_codec_context_t *context = nullptr;
    media_codec_callback_t callback;
    bool m_is_enc_file = false;
    unique_ptr<sr_codec_param_t> m_enc_param = nullptr;
    unique_ptr<ImageFrame> m_enc_frame = nullptr;
    std::mutex m_enc_mtx;

  public:
    int m_chn = 0;
    int m_width = 1920;
    int m_height = 1080;
    int m_type = TYPE_H264;
    int m_enc_bits = 8000;
  };

  class SrPyDecode
  {
  public:
    SrPyDecode() {}

    virtual ~SrPyDecode() = default;

  public:
    int do_decoding(const char *file_name, int video_chn, int type, int width,
                    int height, int *frame_cnt, int feed_mode = 1);

    /// default 1920 * 1080 -- H264 -- chn0 -- mode: frame
    int do_decoding();

    int undo_decoding();

    int vdec_init();

    int vdec_deinit();

    int vdec_start();

    int vdec_stop();

    int vdec_restart();

    ImageFrame *get_frame();

    int send_frame(int chn, void *addr, int size, int eos);

    int put_frame(ImageFrame *frame);

    void do_sync_decoding(void *param);

    void decode_func(void *param);

  private:
    atomic_flag m_dec_inited = ATOMIC_FLAG_INIT;
    unique_ptr<sr_codec_param_t> m_decode_param = nullptr;
    string m_dec_file;
    atomic_flag m_start_once = ATOMIC_FLAG_INIT;
    shared_ptr<thread> m_running_thread;
    media_codec_context_t *context = nullptr;
    media_codec_callback_t callback;
    unique_ptr<ImageFrame> m_dec_frame = nullptr;
    std::mutex m_dec_mtx;

  public:
    int m_chn = 0;
    int m_width = 1920;
    int m_height = 1080;
    int m_type = TYPE_H264;
    int m_feed_mode = 0;
  };

}; // namespace srpy_cam

#endif
