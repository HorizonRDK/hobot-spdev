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

#include "utils_log.h"
#include "x3_sdk_codec.h"
#include "x3_vio_vp.h"

using namespace std;

namespace srpy_cam
{

void VPPCodec::VencChnAttrInit(VENC_CHN_ATTR_S *pVencChnAttr, PAYLOAD_TYPE_E p_enType,
                                int p_Width, int p_Height, PIXEL_FORMAT_E pixFmt)
{
    int streambuf = 0;

    memset(pVencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    pVencChnAttr->stVencAttr.enType = p_enType;

    pVencChnAttr->stVencAttr.u32PicWidth = p_Width;
    pVencChnAttr->stVencAttr.u32PicHeight = p_Height;

    pVencChnAttr->stVencAttr.enMirrorFlip = DIRECTION_NONE;
    pVencChnAttr->stVencAttr.enRotation = CODEC_ROTATION_0;
    pVencChnAttr->stVencAttr.stCropCfg.bEnable = HB_FALSE;

    if (p_Width * p_Height > 2688 * 1522) {
        pVencChnAttr->stVencAttr.vlc_buf_size = 7900 * 1024;
    } else if (p_Width * p_Height > 1920 * 1080) {
        pVencChnAttr->stVencAttr.vlc_buf_size = 4 * 1024 * 1024;
    } else if (p_Width * p_Height > 1280 * 720) {
        pVencChnAttr->stVencAttr.vlc_buf_size = 2100 * 1024;
    } else if (p_Width * p_Height > 704 * 576) {
        pVencChnAttr->stVencAttr.vlc_buf_size = 2100 * 1024;
    } else {
        pVencChnAttr->stVencAttr.vlc_buf_size = 2048 * 1024;
    }

    streambuf = (p_Width * p_Height) & 0xfffff000;
    if (p_enType == PT_JPEG || p_enType == PT_MJPEG) {
        pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 1;
        pVencChnAttr->stVencAttr.u32FrameBufferCount = 2;
        pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
        pVencChnAttr->stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
        pVencChnAttr->stVencAttr.stAttrJpeg.quality_factor = 0;
        pVencChnAttr->stVencAttr.stAttrJpeg.restart_interval = 0;
        pVencChnAttr->stVencAttr.u32BitStreamBufSize = streambuf;
    } else {
        pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = ENC_BUF_NUMS;
        pVencChnAttr->stVencAttr.u32FrameBufferCount = ENC_BUF_NUMS;
        pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
        pVencChnAttr->stVencAttr.u32BitStreamBufSize = streambuf;
    }

    if (p_enType == PT_H265) {
        pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
        pVencChnAttr->stRcAttr.stH265Vbr.bQpMapEnable = HB_TRUE;
        pVencChnAttr->stRcAttr.stH265Vbr.u32IntraQp = 20;
        pVencChnAttr->stRcAttr.stH265Vbr.u32IntraPeriod = 60;
        pVencChnAttr->stRcAttr.stH265Vbr.u32FrameRate = COM_FRAME_RATE;
    }
    if (p_enType == PT_H264) {
        pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
        pVencChnAttr->stRcAttr.stH264Vbr.bQpMapEnable = HB_TRUE;
        pVencChnAttr->stRcAttr.stH264Vbr.u32IntraQp = 20;
        pVencChnAttr->stRcAttr.stH264Vbr.u32IntraPeriod = 60;
        pVencChnAttr->stRcAttr.stH264Vbr.u32FrameRate = COM_FRAME_RATE;
        pVencChnAttr->stVencAttr.stAttrH264.h264_profile = HB_H264_PROFILE_UNSPECIFIED;
        pVencChnAttr->stVencAttr.stAttrH264.h264_level = HB_H264_LEVEL_UNSPECIFIED;
    }

    /*
     * gop 2: 只有I帧和P帧, P帧参考两个前向参考帧, 低延时, 单线关系
     * gop 6: 只有I帧和P帧, P帧参考两个前向参考帧, 低延时, 更复杂
     */
    pVencChnAttr->stGopAttr.u32GopPresetIdx = 2;
    pVencChnAttr->stGopAttr.s32DecodingRefreshType = 2;
}

void VPPCodec::VdecChnAttrInit(VDEC_CHN_ATTR_S *pVdecChnAttr, PAYLOAD_TYPE_E p_enType,
                                VIDEO_MODE_E p_mode, int p_Width, int p_Height)
{
    memset(pVdecChnAttr, 0, sizeof(VDEC_CHN_ATTR_S));
    pVdecChnAttr->enType = p_enType;
    pVdecChnAttr->enMode = p_mode;
    pVdecChnAttr->enPixelFormat = HB_PIXEL_FORMAT_NV12;
    pVdecChnAttr->u32FrameBufCnt = DEC_BUF_NUMS;
    pVdecChnAttr->u32StreamBufCnt = DEC_BUF_NUMS;
    pVdecChnAttr->u32StreamBufSize = (p_Width * p_Height * 3 / 2 + 1024) & ~0x3ff;
    pVdecChnAttr->bExternalBitStreamBuf = HB_TRUE;

    if (p_enType == PT_H265) {
        pVdecChnAttr->stAttrH265.bandwidth_Opt = HB_TRUE;
        pVdecChnAttr->stAttrH265.enDecMode = VIDEO_DEC_MODE_NORMAL;
        pVdecChnAttr->stAttrH265.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
        pVdecChnAttr->stAttrH265.cra_as_bla = HB_FALSE;
        pVdecChnAttr->stAttrH265.dec_temporal_id_mode = 0;
        pVdecChnAttr->stAttrH265.target_dec_temporal_id_plus1 = 2;
    }

    if (p_enType == PT_H264) {
        pVdecChnAttr->stAttrH264.bandwidth_Opt = HB_TRUE;
        pVdecChnAttr->stAttrH264.enDecMode = VIDEO_DEC_MODE_NORMAL;
        pVdecChnAttr->stAttrH264.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
    }

    if (p_enType == PT_JPEG) {
        pVdecChnAttr->bExternalBitStreamBuf = HB_FALSE;
        pVdecChnAttr->u32StreamBufSize = (p_Width * p_Height * 3 / 2 + 1024) & ~0x3ff;
        pVdecChnAttr->stAttrJpeg.enMirrorFlip = DIRECTION_NONE;
        pVdecChnAttr->stAttrJpeg.enRotation = CODEC_ROTATION_0;
        pVdecChnAttr->stAttrJpeg.stCropCfg.bEnable = HB_FALSE;
    }
}

int VPPCodec::x3_venc_init()
{
    int s32Ret = 0;
    VENC_CHN_ATTR_S vencChnAttr;
    VENC_RC_ATTR_S *pstRcParam;
    PAYLOAD_TYPE_E ptype;

    if (m_type == TYPE_H264) {
        ptype = PT_H264;
    } else if (m_type == TYPE_H265) {
        ptype = PT_H265;
    } else if (m_type == TYPE_JPEG) {
        ptype = PT_JPEG;
    } else {
        LOGE_print("error type:%d\n", m_type);
        return -1;
    }

    memset(&vencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    VencChnAttrInit(&vencChnAttr, ptype, m_width, m_height, HB_PIXEL_FORMAT_NV12);

    s32Ret = HB_VENC_CreateChn(m_chn, &vencChnAttr);
    if (s32Ret != 0) {
        LOGE_print("HB_VENC_CreateChn %d failed, %x.\n", m_chn, s32Ret);
        return -1;
    }

    if (ptype == PT_H264) {
        pstRcParam = &vencChnAttr.stRcAttr;
        vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        s32Ret = HB_VENC_GetRcParam(m_chn, pstRcParam);
        if (s32Ret != 0) {
            LOGE_print("HB_VENC_GetRcParam failed.\n");
            return -1;
        }

        LOGD_print(" -------- vencChnAttr.stRcAttr.enRcMode = %d --------\n",
                   vencChnAttr.stRcAttr.enRcMode);
        LOGD_print(" -------- u32VbvBufferSize = %d --------m\n",
                   vencChnAttr.stRcAttr.stH264Cbr.u32VbvBufferSize);

        pstRcParam->stH264Cbr.u32BitRate = m_enc_bits;
        pstRcParam->stH264Cbr.u32FrameRate = COM_FRAME_RATE;
        pstRcParam->stH264Cbr.u32IntraPeriod = 60;
        pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
    } else if (ptype == PT_H265) {
        pstRcParam = &(vencChnAttr.stRcAttr);
        vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
        s32Ret = HB_VENC_GetRcParam(m_chn, pstRcParam);
        if (s32Ret != 0) {
            LOGE_print("HB_VENC_GetRcParam failed.\n");
            return -1;
        }
        LOGD_print(" -------- m_VencChnAttr.stRcAttr.enRcMode = %d -------- \n",
                   vencChnAttr.stRcAttr.enRcMode);
        LOGD_print(" -------- u32VbvBufferSize = %d -------- \n",
                   vencChnAttr.stRcAttr.stH265Cbr.u32VbvBufferSize);

        pstRcParam->stH265Cbr.u32BitRate = m_enc_bits;
        pstRcParam->stH265Cbr.u32FrameRate = COM_FRAME_RATE;
        pstRcParam->stH265Cbr.u32IntraPeriod = 30;
        pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;
    }

    s32Ret = HB_VENC_SetChnAttr(m_chn, &vencChnAttr); // config
    if (s32Ret != 0) {
        LOGE_print("HB_VENC_SetChnAttr failed\n");
        return -1;
    }

    return 0;
}

int VPPCodec::x3_vdec_init()
{
    int s32Ret = 0;
    PAYLOAD_TYPE_E p_decType;
    VDEC_CHN_ATTR_S VdecChnAttr;

    if (m_type == TYPE_H264) {
        p_decType = PT_H264;
    } else if (m_type == TYPE_H265) {
        p_decType = PT_H265;
    } else if (m_type == TYPE_JPEG) {
        p_decType = PT_JPEG;
    } else {
        LOGE_print("error type:%d\n", m_type);
        return -1;
    }

    memset(&VdecChnAttr, 0, sizeof(VDEC_CHN_ATTR_S));
    VdecChnAttrInit(&VdecChnAttr, p_decType, m_dec_mode, m_width, m_height);

    s32Ret = HB_VDEC_CreateChn(m_chn, &VdecChnAttr);
    if (s32Ret != 0) {
        LOGE_print("HB_VDEC_CreateChn failed %x\n", s32Ret);
        return s32Ret;
    }

    s32Ret = HB_VDEC_SetChnAttr(m_chn, &VdecChnAttr);
    if (s32Ret != 0) {
        LOGE_print("HB_VDEC_SetChnAttr failed %x\n", s32Ret);
        return s32Ret;
    }

    return s32Ret;
}

int VPPCodec::x3_vdec_restart()
{
    int ret = 0;

    ret = HB_VDEC_StopRecvStream(m_chn);
    if (ret < 0) {
        return ret;
    }
    ret = HB_VDEC_DestroyChn(m_chn);
    if (ret < 0) {
        return ret;
    }
    ret = x3_vdec_init();
    if (ret < 0) {
        return ret;
    }
    ret = HB_VDEC_StartRecvStream(m_chn);
    if (ret < 0) {
        return ret;
    }

    return ret;
}

int VPPCodec::x3_codec_vp_init(x3_codec_param_t *p_param)
{
    int s32Ret = 0;
    int mmz_size = 0;

    if (p_param->codec_type == MODE_VDEC)
        mmz_size = m_width * m_height;
    else if (p_param->codec_type == MODE_VENC)
        mmz_size = m_width * m_height * 3 / 2;
    else
        return -1;

    if (!m_vp_inited.test_and_set()) {
        if (!p_param->vp_param) {
            p_param->vp_param = new vp_param_t();
        }
        p_param->vp_param->mmz_size = mmz_size;
        if (p_param->alloc_nums > 0)
            p_param->vp_param->mmz_cnt = p_param->alloc_nums;
        else
            p_param->vp_param->mmz_cnt = 1;

        s32Ret = x3_vp_init();
        if (s32Ret != 0) {
            LOGE_print("vp_init fail s32Ret = %d !\n", s32Ret);
            m_vp_inited.clear();
            return s32Ret;
        }
        s32Ret = x3_vp_alloc(p_param->vp_param);
        if (s32Ret != 0) {
            LOGE_print("vp_alloc fail s32Ret = %d !\n", s32Ret);
            m_vp_inited.clear();
            return s32Ret;
        }
    }

    return s32Ret;
}

int VPPCodec::x3_codec_vp_deinit(x3_codec_param_t *p_param)
{
    int s32Ret = 0;

    if (m_vp_inited.test_and_set()) {

        x3_vp_free(p_param->vp_param);
        x3_vp_deinit();

        if (p_param->vp_param) {
            delete p_param->vp_param;
            p_param->vp_param = nullptr;
        }
        if (p_param->av_param) {
            delete p_param->av_param;
            p_param->av_param = nullptr;
        }
    }
    m_vp_inited.clear();

    return s32Ret;
}

int VPPCodec::x3_av_open_stream(x3_codec_param_t *p_param,
                                 AVFormatContext **p_avContext, AVPacket *p_avpacket)
{
    int s32Ret = 0;
    uint8_t retry = 10;
    int video_idx = -1;

    if (!p_param || !p_avContext || !p_avpacket)
        return -1;

    if (!p_param->av_param) {
        p_param->av_param = new codec_av_param_t();
    }

    AVDictionary *option = nullptr;

    av_dict_set(&option, "stimeout", "3000000", 0);
    av_dict_set(&option, "bufsize", "1024000", 0);
    av_dict_set(&option, "rtsp_transport", "tcp", 0);
    do {
        s32Ret = avformat_open_input(p_avContext, p_param->fname, 0, &option);
        if (s32Ret != 0) {
            LOGE_print("avformat_open_input: %d, retry\n", s32Ret);
        }
    } while (retry-- && s32Ret != 0 && !p_param->is_quit);

    if (!retry) {
        LOGE_print("Failed to avformat open %s\n", p_param->fname);
        return -1;
    }

    s32Ret = avformat_find_stream_info(*p_avContext, 0);
    if (s32Ret < 0) {
        LOGE_print("avformat_find_stream_info failed\n");
        return -1;
    }
    LOGI_print("probesize: %ld\n", (*p_avContext)->probesize);

    /* dump input information to stderr */
    // av_dump_format(*p_avContext, 0, p_param->fname, 0);
    video_idx = av_find_best_stream(*p_avContext, AVMEDIA_TYPE_VIDEO,
                                    -1, -1, NULL, 0);
    if (video_idx < 0) {
        LOGE_print("av_find_best_stream failed, ret: %d\n", video_idx);
        return -1;
    }
    av_init_packet(p_avpacket);

    p_param->fname = NULL;
    p_param->frame_count = (*p_avContext)->streams[video_idx]->codec_info_nb_frames;

    sem_post(&p_param->read_done);

    return video_idx;
}

void VPPCodec::x3_do_sync_decoding(void *param)
{
    if (!param) {
        LOGE_print("Invalid param.\n");
        return;
    }

    int error = 0;
    int s32Ret = 0;
    AVFormatContext *avContext = nullptr;
    AVPacket avpacket = {0};
    int video_idx = -1;
    uint8_t *seqHeader = nullptr;
    int seqHeaderSize = 0;
    int firstPacket = 1;
    bool eos = false;
    int bufSize = 0;
    int pkt_cnt = 0;
    int mmz_size = m_width * m_height;
    int mmz_index = 0;
    int mmz_cnt = 0;

    x3_codec_param_t *p_dec_param = static_cast<x3_codec_param_t *>(param);

    VDEC_CHN vdec_chn = static_cast<VDEC_CHN>(m_chn);
    PAYLOAD_TYPE_E codec_type;
    if (m_type == TYPE_H264) {
        codec_type = PT_H264;
    } else if (m_type == TYPE_H265) {
        codec_type = PT_H265;
    } else if (m_type == TYPE_JPEG) {
        codec_type = PT_JPEG;
    } else {
        LOGE_print("codec error type:%d\n", m_type);
        return;
    }

    VIDEO_STREAM_S pstStream;
    memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));

    video_idx = x3_av_open_stream(p_dec_param, &avContext, &avpacket);
    if (video_idx < 0) {
        LOGE_print("failed to x3_av_open_stream\n");
        goto err_av_open;
    }

    do {
        VDEC_CHN_STATUS_S pstStatus;
        HB_VDEC_QueryStatus(vdec_chn, &pstStatus);

        mmz_cnt = p_dec_param->vp_param->mmz_cnt;
        if (pstStatus.cur_input_buf_cnt >= (uint32_t)mmz_cnt) {
            usleep(10 * 1000);
            continue;
        }

        /// wait for each frame for decoding
        usleep(30 * 1000);
        if (p_dec_param->is_quit) {
            eos = true;
            break;
        }

        if (!avpacket.size) {
            error = av_read_frame(avContext, &avpacket);
        }

        if (error < 0) {
            if (error == AVERROR_EOF || avContext->pb->eof_reached == HB_TRUE) {
                LOGD_print("There is no more input data, %d!\n", avpacket.size);
                eos = false;
                break;
            } else {
                LOGE_print("Failed to av_read_frame error(0x%08x)\n", error);
            }

            if (avContext) {
                avformat_close_input(&avContext);
            }
            if (p_dec_param->fname != NULL) {
                avContext = nullptr;
                memset(&avpacket, 0, sizeof(avpacket));
                video_idx = x3_av_open_stream(p_dec_param, &avContext, &avpacket);
                if (video_idx < 0) {
                    LOGE_print("failed to x3_av_open_stream\n");
                    goto err_av_open;
                }
            } else {
                eos = true;
            }
        } else {
            seqHeaderSize = 0;
            mmz_index = pkt_cnt % mmz_cnt;
            if (firstPacket) {
                AUTO_UNIQUE_MTX_LOCK(m_dec_mtx);
                AVCodecParameters *codec;
                int retSize = 0;
                codec = avContext->streams[video_idx]->codecpar;
                seqHeader = (uint8_t *)calloc(1U, codec->extradata_size + 1024);
                if (seqHeader == nullptr) {
                    LOGE_print("Failed to mallock seqHeader\n");
                    eos = true;
                    break;
                }

                seqHeaderSize = AV_build_dec_seq_header(seqHeader,
                                                        codec_type,
                                                        avContext->streams[video_idx], &retSize);
                if (seqHeaderSize < 0) {
                    LOGE_print("Failed to build seqHeader\n");
                    eos = true;
                    break;
                }
                firstPacket = 0;
            }
            if (avpacket.size <= mmz_size) {
                AUTO_UNIQUE_MTX_LOCK(m_dec_mtx);
                if (seqHeaderSize) {
                    memcpy((void *)p_dec_param->vp_param->mmz_vaddr[mmz_index],
                           (void *)seqHeader, seqHeaderSize);
                    bufSize = seqHeaderSize;
                } else {
                    memcpy((void *)p_dec_param->vp_param->mmz_vaddr[mmz_index],
                           (void *)avpacket.data, avpacket.size);
                    bufSize = avpacket.size;
                    av_packet_unref(&avpacket);
                    avpacket.size = 0;
                }
            } else {
                LOGE_print("The external stream buffer is too small!"
                      "avpacket.size:%d, mmz_size:%d\n",
                      avpacket.size, mmz_size);
                eos = true;
                break;
            }

            if (seqHeader) {
                free(seqHeader);
                seqHeader = nullptr;
            }
        }

        /// Decode of sending frame
        {
            AUTO_UNIQUE_MTX_LOCK(m_dec_mtx);
            pstStream.pstPack.phy_ptr =
                p_dec_param->vp_param->mmz_paddr[mmz_index];
            pstStream.pstPack.vir_ptr =
                p_dec_param->vp_param->mmz_vaddr[mmz_index];
            pstStream.pstPack.pts = pkt_cnt++;
            pstStream.pstPack.src_idx = mmz_index;
            if (eos == false) {
                pstStream.pstPack.size = bufSize;
                pstStream.pstPack.stream_end = HB_FALSE;
            } else {
                pstStream.pstPack.size = 0;
                pstStream.pstPack.stream_end = HB_TRUE;
            }
            LOGD_print("[pstStream] pts:%lu, vir_ptr:%p, size:%d\n",
                pstStream.pstPack.pts,
                pstStream.pstPack.vir_ptr,
                pstStream.pstPack.size);

            s32Ret = HB_VDEC_SendStream(vdec_chn, &pstStream, 3000);
            if (s32Ret == -HB_ERR_VDEC_OPERATION_NOT_ALLOWDED ||
                s32Ret == -HB_ERR_VDEC_UNKNOWN) {
                LOGE_print("ERROR:HB_VDEC_SendStream failed\n");
            }
        }

        if (eos) {
            p_dec_param->is_quit = 1;
        }
    } while (!p_dec_param->is_quit);

    if (eos) {
        AUTO_UNIQUE_MTX_LOCK(m_dec_mtx);
        pstStream.pstPack.size = 0;
        pstStream.pstPack.stream_end = HB_TRUE;
        HB_VDEC_SendStream(vdec_chn, &pstStream, 3000);
    }

    if (seqHeader) {
        free(seqHeader);
        seqHeader = nullptr;
    }

err_av_open:
    if (avContext)
        avformat_close_input(&avContext);
}

int VPPCodec::x3_venc_file(char *addr, int32_t size)
{
    if (addr == NULL) {
        LOGE_print("Invalid image address!\n");
        return -1;
    }

    int s32Ret = 0, i = 0, offset = 0;
    if (!m_enc_param.get()) {
        m_enc_param = make_unique<x3_codec_param_t>();
        m_enc_param->codec_type = MODE_VENC;
        m_enc_param->alloc_nums = 1;
    }

    VDEC_CHN venc_chn = static_cast<VDEC_CHN>(m_chn);

    VIDEO_FRAME_S pstFrame;
    memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));

    s32Ret = x3_codec_vp_init(m_enc_param.get());
    if (s32Ret != 0) {
        LOGE_print("failed to x3_codec_vp_init\n");
        goto err_vp_alloc;
    }

    if (size != m_enc_param->vp_param->mmz_size) {
        LOGE_print("Invalid image size:%d!\n", size);
        goto err_vp_alloc;
    }

    for (i = 0; i < m_enc_param->vp_param->mmz_cnt; i++) {
        memcpy(m_enc_param->vp_param->mmz_vaddr[i], addr, size);
    }

    { /// encode file logic
        AUTO_UNIQUE_MTX_LOCK(m_enc_mtx);
        pstFrame.stVFrame.width = m_width;
        pstFrame.stVFrame.height = m_height;
        pstFrame.stVFrame.size = m_enc_param->vp_param->mmz_size;
        pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
        offset = m_width * m_height;
        for (i = 0; i < m_enc_param->vp_param->mmz_cnt; i++) {
            pstFrame.stVFrame.phy_ptr[0] = m_enc_param->vp_param->mmz_paddr[i];
            pstFrame.stVFrame.phy_ptr[1] = m_enc_param->vp_param->mmz_paddr[i] + offset;
            pstFrame.stVFrame.vir_ptr[0] = m_enc_param->vp_param->mmz_vaddr[i];
            pstFrame.stVFrame.vir_ptr[1] = m_enc_param->vp_param->mmz_vaddr[i] + offset;

            s32Ret = HB_VENC_SendFrame(venc_chn, &pstFrame, 3000);
            if (s32Ret != 0) {
                LOGE_print("HB_VENC_SendFrame error!!!\n");
                goto err_vp_alloc;
            }
        }

        m_is_enc_file = true;
    }

    return s32Ret;

err_vp_alloc:
    x3_codec_vp_deinit(m_enc_param.get());
    m_enc_param.reset();

    return -1;
}

ImageFrame *VPPCodec::x3_venc_get_frame()
{
    int ret = 0;
    static int64_t frame_id = 0;
    VENC_CHN venc_chn = static_cast<VENC_CHN>(m_chn);

    memset(&m_enc_pstStream, 0, sizeof(VIDEO_STREAM_S));

    ret = HB_VENC_GetStream(venc_chn, &m_enc_pstStream, 2000);
    if (ret < 0) {
        LOGE_print("HB_VENC_GetStream error!!!\n");
        return nullptr;
    }

    AUTO_UNIQUE_MTX_LOCK(m_enc_mtx);
    /// get venc frame
    m_enc_frame = make_unique<ImageFrame>();
    m_enc_frame->data[0] = reinterpret_cast<uint8_t *>(m_enc_pstStream.pstPack.vir_ptr);
    m_enc_frame->image_id = frame_id++;
    m_enc_frame->image_timestamp = static_cast<int64_t>(time(nullptr));
    m_enc_frame->data_size[0] = m_enc_pstStream.pstPack.size;
    m_enc_frame->frame_info = static_cast<void *>(&m_enc_pstStream);
    m_enc_frame->plane_count = 1;

    return m_enc_frame.get();
}

int VPPCodec::x3_venc_put_frame(ImageFrame *frame)
{
    int ret = 0;
    VENC_CHN venc_chn = static_cast<VENC_CHN>(m_chn);

    ret = HB_VENC_ReleaseStream(venc_chn, &m_enc_pstStream);
    if (ret < 0) {
        LOGE_print("HB_VENC_ReleaseStream error!!!\n");
        return ret;
    }

    if (m_enc_frame) {
        m_enc_frame.reset();
    }

    return ret;
}

ImageFrame *VPPCodec::x3_vdec_get_frame()
{
    int ret = 0;
    static int64_t frame_id = 0;
    VDEC_CHN vdec_chn = static_cast<VDEC_CHN>(m_chn);

    ret = HB_VDEC_GetFrame(vdec_chn, &m_dec_pstFrame, 1000);
    if (ret < 0) {
        LOGE_print("HB_VDEC_GetFrame error!!!\n");
        return nullptr;
    }

    AUTO_UNIQUE_MTX_LOCK(m_dec_mtx);
    m_dec_frame = make_unique<ImageFrame>();
    m_dec_frame->data[0] = reinterpret_cast<uint8_t *>(m_dec_pstFrame.stVFrame.vir_ptr[0]);
    m_dec_frame->data[1] = reinterpret_cast<uint8_t *>(m_dec_pstFrame.stVFrame.vir_ptr[1]);
    m_dec_frame->image_id = frame_id++;
    m_dec_frame->image_timestamp = static_cast<int64_t>(time(nullptr));
    m_dec_frame->data_size[0] = m_width * m_height;
    m_dec_frame->data_size[1] = m_width * m_height / 2;
    m_dec_frame->frame_info = static_cast<void *>(&m_dec_pstFrame);
    m_dec_frame->plane_count = 2;

    return m_dec_frame.get();
}

int VPPCodec::x3_vdec_put_frame(ImageFrame *frame)
{
    int ret = 0;
    VDEC_CHN vdec_chn = static_cast<VDEC_CHN>(m_chn);

    ret = HB_VDEC_ReleaseFrame(vdec_chn, &m_dec_pstFrame);
    if (ret < 0) {
        LOGE_print("HB_VDEC_ReleaseFrame error!!!\n");
        return ret;
    }

    if (m_dec_frame) {
        m_dec_frame.reset();
    }

    return ret;
}

/// Class VPPEncode related

int VPPEncode::do_encoding(int video_chn, int type, int width,
                            int height, int bits)
{
    static int l_chn, l_type, l_width, l_height, l_bits;

    if (!m_enc_inited.test_and_set()) {
        m_enc_obj = make_unique<VPPCodec>(video_chn,
                                           type, width, height, bits);

        if (x3_venc_common_init())
            return -1;

        if (m_enc_obj->x3_venc_init())
            return -1;

        if (m_enc_obj->x3_venc_start())
            return -1;

        l_chn = video_chn;
        l_type = type;
        l_width = width;
        l_height = height;
        l_bits = bits;
    }

    if (l_chn != video_chn || type != l_type ||
        width != l_width || height != l_height || bits != l_bits) {
        LOGE_print("Invalid encode param, must be same as before(chn:%d typt:%d w:%d h:%d bits:%d)\n",
              l_chn, l_type, l_width, l_height, l_bits);
        return -1;
    }

    return 0;
}

int VPPEncode::do_encoding()
{
    if (!m_enc_inited.test_and_set()) {
        m_enc_obj = make_unique<VPPCodec>();

        if (x3_venc_common_init())
            return -1;

        if (m_enc_obj->x3_venc_init())
            return -1;

        if (m_enc_obj->x3_venc_start())
            return -1;
    }

    return 0;
}

int VPPEncode::encode_file(char *addr, int32_t size)
{
    return m_enc_obj->x3_venc_file(addr, size);
}

int VPPEncode::undo_encoding()
{
    if (!m_enc_obj) {
        LOGE_print("Invalid param!\n");
        return -1;
    }

    m_enc_inited.clear();
    m_enc_obj->x3_venc_stop();
    m_enc_obj->x3_venc_deinit();
    x3_venc_common_deinit();

    return 0;
}

ImageFrame *VPPEncode::get_frame()
{
    if (!m_enc_obj) {
        LOGE_print("Invalid param!\n");
        return nullptr;
    }

    return m_enc_obj->x3_venc_get_frame();
}

int VPPEncode::put_frame(ImageFrame *frame)
{
    if (!m_enc_obj) {
        LOGE_print("Invalid param!\n");
        return -1;
    }

    return m_enc_obj->x3_venc_put_frame(frame);
}

/// Class VPPDecode related
int VPPDecode::do_decoding(const char *file_name, int video_chn,
        int type, int width, int height, int *frame_cnt, int dec_mode)
{

    static int l_chn, l_type, l_width, l_height, l_mode;

    if (!m_dec_inited.test_and_set()) {
        m_dec_obj = make_unique<VPPCodec>(video_chn, type, width,
                                           height, 8000, dec_mode);

        if (x3_vdec_common_init())
            return -1;

        if (m_dec_obj->x3_vdec_init())
            return -1;

        if (m_dec_obj->x3_vdec_start())
            return -1;

        l_chn = video_chn;
        l_type = type;
        l_width = width;
        l_height = height;
        l_mode = dec_mode;
    }

    m_dec_file = file_name;

    if (!m_start_once.test_and_set()) {
        m_decode_param = make_unique<x3_codec_param_t>();
        m_decode_param->codec_type = MODE_VDEC;
        m_decode_param->alloc_nums = 3;
        if (m_dec_obj->x3_codec_vp_init(m_decode_param.get()) != 0) {
            LOGE_print("failed to x3_codec_vp_init\n");
            return -1;
        }
        m_decode_param->fname = m_dec_file.data();
        m_decode_param->is_quit = true;
        sem_init(&m_decode_param->read_done, 0, 0);
        if ((m_decode_param->fname != NULL) && (strlen(m_decode_param->fname) > 0)) {
            m_decode_param->is_quit = false;
            m_running_thread =
                make_shared<thread>(&VPPDecode::decode_func,
                                this, static_cast<void *>(m_decode_param.get()));
        }
    } else {
        m_decode_param->fname = m_dec_file.data();
        if (m_decode_param->is_quit == true) {
            if (m_dec_obj->x3_vdec_restart())
                return -1;
            if (m_running_thread && m_running_thread->joinable()) {
                m_running_thread->join();
            }
            m_decode_param->is_quit = true;
            if ((m_decode_param->fname != NULL) && (strlen(m_decode_param->fname) > 0)) {
                m_decode_param->is_quit = false;
                m_running_thread =
                    make_shared<thread>(&VPPDecode::decode_func,
                                this, static_cast<void *>(m_decode_param.get()));
            }
        }
    }

    if (!m_decode_param->is_quit) {
        sem_wait(&m_decode_param->read_done);
        *frame_cnt = m_decode_param->frame_count;
    }

    if (l_chn != video_chn || type != l_type ||
        width != l_width || height != l_height || dec_mode != l_mode) {
        LOGE_print("Invalid decode param, must be same as before(chn:%d type:%d w:%d h:%d mode:%d)\n",
              l_chn, l_type, l_width, l_height, l_mode);
        return -1;
    }

    return 0;
}

int VPPDecode::do_decoding()
{
    if (!m_dec_inited.test_and_set()) {
        m_dec_obj = make_unique<VPPCodec>();

        if (x3_vdec_common_init())
            return -1;

        if (m_dec_obj->x3_vdec_init())
            return -1;

        if (!m_start_once.test_and_set()) {
            m_decode_param = make_unique<x3_codec_param_t>();
            m_decode_param->fname = m_dec_file.data();
            m_decode_param->is_quit = false;
            if ((m_decode_param->fname != NULL) && (strlen(m_decode_param->fname) > 0)) {
                m_running_thread =
                    make_shared<thread>(&VPPDecode::decode_func,
                                    this, static_cast<void *>(m_decode_param.get()));
            }
        }

        if (m_dec_obj->x3_vdec_start())
            return -1;
    }

    return 0;
}

int VPPDecode::undo_decoding()
{
    if (!m_dec_obj) {
        LOGE_print("Invalid param!\n");
        return -1;
    }

    m_dec_inited.clear();
    m_decode_param->is_quit = true;
    if (m_running_thread && m_running_thread->joinable()) {
        m_running_thread->join();
    }

    m_dec_obj->x3_vdec_stop();

    m_start_once.clear();

    sem_destroy(&m_decode_param->read_done);

    if (m_decode_param->vp_param) {
        m_dec_obj->x3_codec_vp_deinit(m_decode_param.get());
        delete m_decode_param->vp_param;
        m_decode_param->vp_param = nullptr;
    }

    if (m_decode_param->av_param) {
        delete m_decode_param->av_param;
        m_decode_param->av_param = nullptr;
    }
    m_dec_obj->x3_vdec_deinit();
    x3_vdec_common_deinit();

    return 0;
}

void VPPDecode::decode_func(void *param)
{
    if (!m_dec_obj || !param) {
        LOGE_print("Invalid param!\n");
        return;
    }

    m_dec_obj->x3_do_sync_decoding(param);
}

ImageFrame *VPPDecode::get_frame()
{
    if (!m_dec_obj) {
        LOGE_print("Invalid param!\n");
        return nullptr;
    }

    return m_dec_obj->x3_vdec_get_frame();
}

int VPPDecode::put_frame(ImageFrame *frame)
{
    if (!m_dec_obj) {
        LOGE_print("Invalid param!\n");
        return -1;
    }

    return m_dec_obj->x3_vdec_put_frame(frame);
}

int VPPDecode::send_frame(int chn, void *addr, int size, int eos)
{
    VIDEO_STREAM_S pstStream = {0};
    static int pkt_cnt = 0;
    int mmz_index = 0;
    int ret = 0;

    if (!m_dec_obj) {
        LOGE_print("Invalid param!\n");
        return -1;
    }

    mmz_index = pkt_cnt % m_decode_param->alloc_nums;
    memcpy(m_decode_param->vp_param->mmz_vaddr[mmz_index], addr, size);

    pstStream.pstPack.phy_ptr =
        m_decode_param->vp_param->mmz_paddr[mmz_index];
    pstStream.pstPack.vir_ptr =
        m_decode_param->vp_param->mmz_vaddr[mmz_index];
    pstStream.pstPack.pts = pkt_cnt++;
    pstStream.pstPack.src_idx = mmz_index;
    if (eos == false) {
        pstStream.pstPack.size = size;
        pstStream.pstPack.stream_end = HB_FALSE;
    } else {
        pstStream.pstPack.size = 0;
        pstStream.pstPack.stream_end = HB_TRUE;
    }
    LOGD_print("[pstStream] pts:%lu, vir_ptr:%p, size:%d\n",
        pstStream.pstPack.pts,
        pstStream.pstPack.vir_ptr,
        pstStream.pstPack.size);

    ret = HB_VDEC_SendStream(chn, &pstStream, 3000);
    if (ret < 0) {
        LOGE_print("ERROR:HB_VDEC_SendStream failed\n");
    }

    return ret;
}

}; // namespace srpy_cam
