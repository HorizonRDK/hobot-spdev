/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2019 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "vio/hb_comm_vdec.h"
#include "vio/hb_vdec.h"
#include "vio/hb_comm_video.h"
#include "vio/hb_type.h"
#include "vio/hb_common.h"
#include "utils_log.h"
#include "logging.h"

#include "x3_vio_vdec.h"

static int s_vdec_init_cnt = 0;

int x3_vdec_common_init()
{
    int s32Ret;

    if (s_vdec_init_cnt > 0) {
        s_vdec_init_cnt++;
        return 0;
    }

    s32Ret = HB_VDEC_Module_Init();
    if (s32Ret) {
        printf("HB_VDEC_Module_Init: %d\n", s32Ret);
    }

    if (s32Ret == 0) {
        s_vdec_init_cnt++;
    }

    return s32Ret;
}

int x3_vdec_common_deinit()
{
    int s32Ret;

    s_vdec_init_cnt--;
    if (s_vdec_init_cnt > 0) {
        return 0;
    }

    s32Ret = HB_VDEC_Module_Uninit();
    if (s32Ret) {
        printf("HB_VDEC_Module_Uninit: %d\n", s32Ret);
    }

    return s32Ret;
}

int x3_vdec_init(VDEC_CHN vdecChn, VDEC_CHN_ATTR_S *vdecChnAttr)
{
    int s32Ret = 0;

    s32Ret = HB_VDEC_CreateChn(vdecChn, vdecChnAttr);
    if (s32Ret != 0) {
        printf("HB_VDEC_CreateChn failed %x\n", s32Ret);
        return s32Ret;
    }

    s32Ret = HB_VDEC_SetChnAttr(vdecChn, vdecChnAttr); // config
    if (s32Ret != 0) {
        printf("HB_VDEC_SetChnAttr failed %x\n", s32Ret);
        return s32Ret;
    }

    LOGI_print("ok!\n");
    return 0;
}

int x3_vdec_start(VDEC_CHN vdecChn)
{
    int s32Ret = HB_VDEC_StartRecvStream(vdecChn);
    if (s32Ret != 0) {
        printf("HB_VDEC_StartRecvStream failed %x\n", s32Ret);
        return s32Ret;
    }
    LOGI_print("ok!\n");
    return 0;
}

int x3_vdec_stop(VDEC_CHN vdecChn)
{
    int s32Ret = 0;

    s32Ret = HB_VDEC_StopRecvStream(vdecChn);
    if (s32Ret != 0) {
        printf("HB_VDEC_StopRecvStream failed %x\n", s32Ret);
        return s32Ret;
    }
    LOGI_print("ok!\n");

    return 0;
}

int x3_vdec_deinit(VDEC_CHN vdecChn)
{
    int s32Ret = 0;

    s32Ret = HB_VDEC_DestroyChn(vdecChn);
    if (s32Ret != 0) {
        printf("HB_VDEC_ReleaseFrame failed %d\n", s32Ret);
        return s32Ret;
    }
    LOGI_print("ok!\n");
    return 0;
}

#define SET_BYTE(_p, _b) \
    *_p++ = (unsigned char)_b;

#define SET_BUFFER(_p, _buf, _len) \
    memcpy(_p, _buf, _len);        \
    (_p) += (_len);

int AV_build_dec_seq_header(uint8_t *pbHeader,
                            const PAYLOAD_TYPE_E p_enType,
                            const AVStream *st, int *sizelength)
{
    AVCodecParameters *avc = st->codecpar;

    uint8_t *pbMetaData = avc->extradata;
    int nMetaData = avc->extradata_size;
    uint8_t *p = pbMetaData;
    uint8_t *a = p + 4 - ((long)p & 3);
    uint8_t *t = pbHeader;
    int size;
    int sps, pps, i, nal;

    size = 0;
    *sizelength = 4; // default size length(in bytes) = 4
    if (p_enType == PT_H264) {
        if (nMetaData > 1 && pbMetaData && pbMetaData[0] == 0x01) {
            // check mov/mo4 file format stream
            p += 4;
            *sizelength = (*p++ & 0x3) + 1;
            sps = (*p & 0x1f); // Number of sps
            p++;
            for (i = 0; i < sps; i++) {
                nal = (*p << 8) + *(p + 1) + 2;
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x01);
                SET_BUFFER(t, p + 2, nal - 2);
                p += nal;
                size += (nal - 2 + 4); // 4 => length of start code to be inserted
            }

            pps = *(p++); // number of pps
            for (i = 0; i < pps; i++) {
                nal = (*p << 8) + *(p + 1) + 2;
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x01);
                SET_BUFFER(t, p + 2, nal - 2);
                p += nal;
                size += (nal - 2 + 4); // 4 => length of start code to be inserted
            }
        } else if (nMetaData > 3) {
            size = -1; // return to meaning of invalid stream data;
            for (; p < a; p++) {
                if (p[0] == 0 && p[1] == 0 && p[2] == 1) {
                    // find startcode
                    size = avc->extradata_size;
                    if (pbMetaData && 0x00 == pbMetaData[size-1]) {
                        size-=1;
                    }
                    if (!pbHeader || !pbMetaData)
                        return 0;
                    SET_BUFFER(pbHeader, pbMetaData, size);
                    break;
                }
            }
        }
    } else if (p_enType == PT_H265) {
        if (nMetaData > 1 && pbMetaData && pbMetaData[0] == 0x01) {
            static const int8_t nalu_header[4] = {0, 0, 0, 1};
            int numOfArrays = 0;
            uint16_t numNalus = 0;
            uint16_t nalUnitLength = 0;
            uint32_t offset = 0;

            p += 21;
            *sizelength = (*p++ & 0x3) + 1;
            numOfArrays = *p++;

            while (numOfArrays--) {
                p++; // NAL type
                numNalus = (*p << 8) + *(p + 1);
                p += 2;
                for (i = 0; i < numNalus; i++) {
                    nalUnitLength = (*p << 8) + *(p + 1);
                    p += 2;
                    // if(i == 0)
                    {
                        memcpy(pbHeader + offset, nalu_header, 4);
                        offset += 4;
                        memcpy(pbHeader + offset, p, nalUnitLength);
                        offset += nalUnitLength;
                    }
                    p += nalUnitLength;
                }
            }

            size = offset;
        } else if (nMetaData > 3) {
            size = -1; // return to meaning of invalid stream data;

            for (; p < a; p++) {
                if (p[0] == 0 && p[1] == 0 && p[2] == 1) // find startcode
                {
                    size = avc->extradata_size;
                    if (!pbHeader || !pbMetaData)
                        return 0;
                    SET_BUFFER(pbHeader, pbMetaData, size);
                    break;
                }
            }
        }
    } else {
        SET_BUFFER(pbHeader, pbMetaData, nMetaData);
        size = nMetaData;
    }

    return size;
}

// 需要引用ffpemg库
int AV_open_stream_file(char *FileName, AVFormatContext **avContext,
                        AVPacket *avpacket)
{
    int ret = 0;

    ret = avformat_open_input(avContext, FileName, 0, 0);
    if (ret < 0) {
        printf("avformat_open_input failed\n");
        return -1;
    }
    ret = avformat_find_stream_info(*avContext, 0);
    if (ret < 0) {
        printf("avformat_find_stream_info failed\n");
        return -1;
    }
    printf("probesize: %ld\n", (*avContext)->probesize);

    /* dump input information to stderr */
    av_dump_format(*avContext, 0, FileName, 0);
    int index = av_find_best_stream(*avContext, AVMEDIA_TYPE_VIDEO,
                                    -1, -1, NULL, 0);
    if (index < 0) {
        printf("av_find_best_stream failed, ret: %d\n", index);
        return -1;
    }
    av_init_packet(avpacket);

    return index;
}

int AV_read_frame(AVFormatContext *avContext, AVPacket *avpacket,
                  av_param_t *av_param, vp_param_t *vp_param)
{
    uint8_t *seqHeader = NULL;
    int seqHeaderSize = 0, error = 0;

    if (!avpacket->size) {
        error = av_read_frame(avContext, avpacket);
    }
    if (error < 0) {
        if (error == AVERROR_EOF || avContext->pb->eof_reached == 1) {
            LOGD_print("There is no more input data, %d!\n", avpacket->size);
        } else {
            printf("Failed to av_read_frame error(0x%08x)\n", error);
        }
        return -1;
    } else {
        seqHeaderSize = 0;
        int mmz_index = av_param->count % vp_param->mmz_cnt;
        if (av_param->firstPacket) {
            AVCodecParameters *codec;
            int retSize = 0;
            codec = avContext->streams[av_param->videoIndex]->codecpar;
            seqHeader = (uint8_t *)malloc(codec->extradata_size + 1024);
            if (seqHeader == NULL) {
                printf("Failed to mallock seqHeader\n");
                return -1;
            }
            memset((void *)seqHeader, 0x00, codec->extradata_size + 1024);

            seqHeaderSize = AV_build_dec_seq_header(seqHeader,
                                                    PT_H264, avContext->streams[av_param->videoIndex], &retSize);
            if (seqHeaderSize < 0) {
                printf("Failed to build seqHeader\n");
                return -1;
            }
            av_param->firstPacket = 0;
        }
        if (avpacket->size <= vp_param->mmz_size) {
            if (seqHeaderSize) {
                memcpy((void *)vp_param->mmz_vaddr[mmz_index],
                       (void *)seqHeader, seqHeaderSize);
                av_param->bufSize = seqHeaderSize;
            } else {
                memcpy((void *)vp_param->mmz_vaddr[mmz_index],
                       (void *)avpacket->data, avpacket->size);
                av_param->bufSize = avpacket->size;
                av_packet_unref(avpacket);
                avpacket->size = 0;
            }
        } else {
            printf("The external stream buffer is too small!"
                   "avpacket.size:%d, mmz_size:%d\n",
                   avpacket->size, vp_param->mmz_size);
            return -1;
        }
        if (seqHeader) {
            free(seqHeader);
            seqHeader = NULL;
        }
        ++av_param->count;
        return mmz_index;
    }
}