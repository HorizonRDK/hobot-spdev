/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-03-03 21:45:00
 * @LastEditTime: 2023-03-05 16:34:22
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

#include "x3_sdk_codec.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/avutil.h"

#ifdef __cplusplus
}
#endif /* extern "C" */

#include "utils_log.h"

using namespace std;

namespace srpy_cam
{
    int AV_open_stream(sr_codec_param_t *p_param,
                          AVFormatContext **p_avContext, AVPacket *p_avpacket)
    {
        int ret = 0;
        uint8_t retry = 10;
        int video_idx = -1;

        if (!p_param || !p_avContext || !p_avpacket)
            return -1;

        AVDictionary *option = nullptr;

        av_dict_set(&option, "stimeout", "3000000", 0);
        av_dict_set(&option, "bufsize", "1024000", 0);
        av_dict_set(&option, "rtsp_transport", "tcp", 0);
        do
        {
            ret = avformat_open_input(p_avContext, p_param->fname, 0, &option);
            if (ret != 0)
            {
                printf("avformat_open_input: %d, retry\n", ret);
            }
        } while (retry-- && ret != 0 && !p_param->is_quit);

        if (!retry)
        {
            printf("Failed to avformat open %s\n", p_param->fname);
            return -1;
        }

        ret = avformat_find_stream_info(*p_avContext, 0);
        if (ret < 0)
        {
            printf("avformat_find_stream_info failed\n");
            return -1;
        }
        printf("probesize: %ld\n", (*p_avContext)->probesize);

        /* dump input information to stderr */
        // av_dump_format(*p_avContext, 0, p_param->fname, 0);
        video_idx = av_find_best_stream(*p_avContext, AVMEDIA_TYPE_VIDEO,
                                        -1, -1, NULL, 0);
        if (video_idx < 0)
        {
            printf("av_find_best_stream failed, ret: %d\n", video_idx);
            return -1;
        }
        av_init_packet(p_avpacket);

        p_param->fname = NULL;
        p_param->frame_count = (*p_avContext)->streams[video_idx]->codec_info_nb_frames;

        sem_post(&p_param->read_done);

        return video_idx;
    }

#define SET_BYTE(_p, _b) \
    *_p++ = (unsigned char)_b;

#define SET_BUFFER(_p, _buf, _len) \
    memcpy(_p, _buf, _len);        \
    (_p) += (_len);

    int AV_build_dec_seq_header(uint8_t *pbHeader,
                                const media_codec_id_t codec_id,
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
        if (codec_id == MEDIA_CODEC_ID_H264)
        {
            if (nMetaData > 1 && pbMetaData && pbMetaData[0] == 0x01)
            {
                // check mov/mo4 file format stream
                p += 4;
                *sizelength = (*p++ & 0x3) + 1;
                sps = (*p & 0x1f); // Number of sps
                p++;
                for (i = 0; i < sps; i++)
                {
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
                for (i = 0; i < pps; i++)
                {
                    nal = (*p << 8) + *(p + 1) + 2;
                    SET_BYTE(t, 0x00);
                    SET_BYTE(t, 0x00);
                    SET_BYTE(t, 0x00);
                    SET_BYTE(t, 0x01);
                    SET_BUFFER(t, p + 2, nal - 2);
                    p += nal;
                    size += (nal - 2 + 4); // 4 => length of start code to be inserted
                }
            }
            else if (nMetaData > 3)
            {
                size = -1; // return to meaning of invalid stream data;
                for (; p < a; p++)
                {
                    if (p[0] == 0 && p[1] == 0 && p[2] == 1)
                    {
                        // find startcode
                        size = avc->extradata_size;
                        if (pbMetaData && 0x00 == pbMetaData[size - 1])
                        {
                            size -= 1;
                        }
                        if (!pbHeader || !pbMetaData)
                            return 0;
                        SET_BUFFER(pbHeader, pbMetaData, size);
                        break;
                    }
                }
            }
        }
        else if (codec_id == MEDIA_CODEC_ID_H265)
        {
            if (nMetaData > 1 && pbMetaData && pbMetaData[0] == 0x01)
            {
                static const int8_t nalu_header[4] = {0, 0, 0, 1};
                int numOfArrays = 0;
                uint16_t numNalus = 0;
                uint16_t nalUnitLength = 0;
                uint32_t offset = 0;

                p += 21;
                *sizelength = (*p++ & 0x3) + 1;
                numOfArrays = *p++;

                while (numOfArrays--)
                {
                    p++; // NAL type
                    numNalus = (*p << 8) + *(p + 1);
                    p += 2;
                    for (i = 0; i < numNalus; i++)
                    {
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
            }
            else if (nMetaData > 3)
            {
                size = -1; // return to meaning of invalid stream data;

                for (; p < a; p++)
                {
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
        }
        else
        {
            SET_BUFFER(pbHeader, pbMetaData, nMetaData);
            size = nMetaData;
        }

        return size;
    }

    /// Class SrPyEncode related

    int SrPyEncode::do_encoding(int video_chn, int type, int width,
                                int height, int bits)
    {
        static int l_chn, l_type, l_width, l_height, l_bits;

        if (!m_enc_inited.test_and_set())
        {
            m_chn = video_chn;
            m_type = type;
            m_width = width;
            m_height = height;
            m_enc_bits = bits;

            if (venc_init())
                return -1;

            if (venc_start())
                return -1;

            l_chn = video_chn;
            l_type = type;
            l_width = width;
            l_height = height;
            l_bits = bits;
        }

        if (l_chn != video_chn || type != l_type ||
            width != l_width || height != l_height || bits != l_bits)
        {
            printf("Invalid encode param, must be same as before(chn:%d typt:%d w:%d h:%d bits:%d)\n",
                   l_chn, l_type, l_width, l_height, l_bits);
            return -1;
        }

        return 0;
    }

    int SrPyEncode::do_encoding()
    {
        if (!m_enc_inited.test_and_set())
        {
            if (venc_init())
                return -1;

            if (venc_start())
                return -1;
        }

        return 0;
    }

    int SrPyEncode::undo_encoding()
    {
        if (!m_enc_inited.test_and_set())
        {
            printf("Encoder channel dose not created!\n");
            m_enc_inited.clear();
            return -1;
        }
        m_enc_inited.clear();
        venc_stop();
        venc_deinit();

        return 0;
    }

    int SrPyEncode::venc_init()
    {
        int ret = 0;
        mc_video_codec_enc_params_t *params;
        context = (media_codec_context_t *)malloc(sizeof(media_codec_context_t));
        if (context == NULL)
        {
            printf("malloc media_codec_context_t failed.\n");
            return -1;
        }

        memset(context, 0x00, sizeof(media_codec_context_t));
        switch (m_type)
        {
        case TYPE_H264:
            context->codec_id = MEDIA_CODEC_ID_H264;
            context->encoder = true;
            context->instance_index = m_chn;
            params = &context->video_enc_params;
            params->width = m_width;
            params->height = m_height;
            params->pix_fmt = MC_PIXEL_FORMAT_NV12;
            params->frame_buf_count = 3;
            params->external_frame_buf = false;
            params->bitstream_buf_count = 3;
            params->rc_params.mode = MC_AV_RC_MODE_H264CBR;
            params->rc_params.h264_cbr_params.bit_rate = m_enc_bits;
            params->rc_params.h264_cbr_params.frame_rate = COM_FRAME_RATE;
            params->rc_params.h264_avbr_params.intra_period = 60;
            params->rc_params.h264_cbr_params.vbv_buffer_size = 3000;
            params->gop_params.gop_preset_idx = 2;
            params->rot_degree = MC_CCW_0;
            params->mir_direction = MC_DIRECTION_NONE;
            params->frame_cropping_flag = false;
            break;
        case TYPE_H265:
        case TYPE_JPEG:
        case TYPE_MJPEG:
            default:
            printf("Not Support decoding type: %d!\n", m_type);
            free(context);
            return -1;
        }

        ret = hb_mm_mc_initialize(context);
        if (0 != ret)
        {
            printf("hb_mm_mc_initialize failed.\n");
            free(context);
            return -1;
        }

        if ((m_type == TYPE_H264) || (m_type == TYPE_H265))
        {
            ret = hb_mm_mc_request_idr_header(context, 1);
            if (0 != ret)
            {
                printf("hb_mm_mc_request_idr_header Failed .ret = %d\n", ret);
                return -1;
            }

            // callback.on_vlc_buffer_message = on_vlc_buffer_message;
            // if (context.vlc_buf_size > 0)
            // {
            //     ret = hb_mm_mc_set_vlc_buffer_listener(context, &callback, context);
            //     if (0 != ret)
            //     {
            //         printf("hb_mm_mc_set_vlc_buffer_listener failed.\n");
            //         hb_mm_mc_release(context);
            //         free(context);
            //         return -1;
            //     }
            // }
        }

        ret = hb_mm_mc_configure(context);
        if (0 != ret)
        {
            printf("hb_mm_mc_configure failed.\n");
            hb_mm_mc_release(context);
            free(context);
            return -1;
        }

        return 0;
    }

    int SrPyEncode::venc_deinit()
    {
        if (!m_enc_inited.test_and_set())
        {
            printf("Encoder channel dose not created!\n");
            m_enc_inited.clear();
            return -1;
        }

        int ret = 0;
        media_codec_state_t state;
        ret = hb_mm_mc_get_state(context, &state);
        if (ret != 0)
        {
            printf("Failed to hb_mm_mc_get_state ret = %d \n", ret);
            return -1;
        }

        ret = hb_mm_mc_release(context);
        if (ret != 0)
        {
            printf("Failed to hb_mm_mc_release ret = %d \n", ret);
            return -1;
        }

        m_is_enc_file = false;
        return 0;
    }

    int SrPyEncode::venc_start()
    {
        if (!m_enc_inited.test_and_set())
        {
            printf("Encoder channel dose not created!\n");
            m_enc_inited.clear();
            return -1;
        }
        int ret = 0;
        mc_av_codec_startup_params_t startup_params;
        startup_params.video_enc_startup_params.receive_frame_number = 0;
        ret = hb_mm_mc_start(context, &startup_params);
        if (ret != 0)
        {
            printf("hb_mm_mc_start failed.\n");
            return -1;
        }
        return 0;
    }

    int SrPyEncode::venc_stop()
    {
        if (!m_enc_inited.test_and_set())
        {
            printf("Encoder channel dose not created!\n");
            m_enc_inited.clear();
            return -1;
        }

        int ret = 0;
        ret = hb_mm_mc_pause(context);
        if (ret != 0)
        {
            printf("Failed to hb_mm_mc_pause ret = %d \n", ret);
            return -1;
        }
        return 0;
    }

    int SrPyEncode::send_frame(char *addr, int32_t size)
    {
        int ret = 0, i = 0, offset = 0;

        if (!m_enc_inited.test_and_set())
        {
            printf("Encoder channel dose not created!\n");
            m_enc_inited.clear();
            return -1;
        }

        if (addr == NULL)
        {
            printf("Invalid image address!\n");
            return -1;
        }

        media_codec_buffer_t buffer;
        buffer.type = MC_VIDEO_FRAME_BUFFER;
        ret = hb_mm_mc_dequeue_input_buffer(context, &buffer, 3000);
        if (ret != 0)
        {
            printf("hb_mm_mc_dequeue_input_buffer failed ret = %d \n", ret);
            return -1;
        }

        buffer.type = MC_VIDEO_FRAME_BUFFER;
        buffer.vframe_buf.size = size;
        buffer.vframe_buf.width = m_width;
        buffer.vframe_buf.height = m_height;
        buffer.vframe_buf.pix_fmt = MC_PIXEL_FORMAT_NV12;
        memcpy(buffer.vframe_buf.vir_ptr[0], addr, size);

        ret = hb_mm_mc_queue_input_buffer(context, &buffer, 100);
        if (ret != 0)
        {
            printf("hb_mm_mc_queue_input_buffer failed, ret = 0x%x\n", ret);
            return -1;
        }

        return ret;
    }

    ImageFrame *SrPyEncode::SrPyEncode::get_frame()
    {
        int ret = 0;
        media_codec_buffer_t *buffer;
        media_codec_output_buffer_info_t info;
        memset(&info, 0x00, sizeof(media_codec_output_buffer_info_t));

        if (!m_enc_inited.test_and_set())
        {
            printf("Encoder channel dose not created!\n");
            m_enc_inited.clear();
            return nullptr;
        }

        buffer = (media_codec_buffer_t *)malloc(sizeof(media_codec_buffer_t));
        if (buffer == NULL)
        {
            printf("malloc media_codec_buffer_t failed.\n");
            return nullptr;
        }

        ret = hb_mm_mc_dequeue_output_buffer(context, buffer, &info, 2000);
        if (ret != 0)
        {
            printf("hb_mm_mc_dequeue_output_buffer failed ret = %d \n", ret);
            free(buffer);
            return nullptr;
        }
        /// get venc frame
        media_codec_id_t codec_type = context->codec_id;
        if (codec_type == MEDIA_CODEC_ID_H264 ||
            codec_type == MEDIA_CODEC_ID_H265)
        {
            mc_h264_h265_output_stream_info_t *stream_info =
                &(info.video_stream_info);
            m_enc_frame = make_unique<ImageFrame>();
            m_enc_frame->data[0] = reinterpret_cast<uint8_t *>(buffer->vstream_buf.vir_ptr);
            m_enc_frame->image_id = stream_info->frame_index;
            m_enc_frame->image_timestamp = buffer->vstream_buf.pts;
            m_enc_frame->data_size[0] = buffer->vstream_buf.size;
            m_enc_frame->frame_info = static_cast<void *>(buffer);
            m_enc_frame->plane_count = 1;
        }

        return m_enc_frame.get();
    }

    int SrPyEncode::put_frame(ImageFrame *frame)
    {
        if (!m_enc_inited.test_and_set())
        {
            printf("Encoder channel dose not created!\n");
            m_enc_inited.clear();
            return -1;
        }

        int ret = 0;
        media_codec_buffer_t *buffer = static_cast<media_codec_buffer_t *>(frame->frame_info);

        ret = hb_mm_mc_queue_output_buffer(context, buffer, 0);
        if (ret != 0)
        {
            printf("hb_mm_mc_queue_output_buffer failed ret = %d \n", ret);
            free(buffer);
            return -1;
        }

        free(buffer);

        if (m_enc_frame)
        {
            m_enc_frame.reset();
        }
        return ret;
    }

    /// Class SrPyDecode related
    int SrPyDecode::do_decoding(const char *file_name, int video_chn,
                                int type, int width, int height, int *frame_cnt, int feed_mode)
    {

        static int l_chn, l_type, l_width, l_height, l_mode;

        if (!m_dec_inited.test_and_set())
        {
            m_chn = video_chn;
            m_type = type;
            m_width = width;
            m_height = height;
            m_feed_mode = feed_mode;

            if (vdec_init())
                return -1;

            if (vdec_start())
                return -1;

            l_chn = video_chn;
            l_type = type;
            l_width = width;
            l_height = height;
            l_mode = feed_mode;
        }

        m_dec_file = file_name;

        if (!m_start_once.test_and_set())
        {
            m_decode_param = make_unique<sr_codec_param_t>();
            m_decode_param->codec_type = MODE_VDEC;
            m_decode_param->fname = m_dec_file.data();
            m_decode_param->is_quit = true;
            sem_init(&m_decode_param->read_done, 0, 0);
            if ((m_decode_param->fname != NULL) && (strlen(m_decode_param->fname) > 0))
            {
                m_decode_param->is_quit = false;
                m_running_thread =
                    make_shared<thread>(&SrPyDecode::decode_func,
                                        this, static_cast<void *>(m_decode_param.get()));
            }
        }
        else
        {
            m_decode_param->fname = m_dec_file.data();
            if (m_decode_param->is_quit == true)
            {
                if (vdec_restart())
                    return -1;
                if (m_running_thread && m_running_thread->joinable())
                {
                    m_running_thread->join();
                }
                m_decode_param->is_quit = true;
                if ((m_decode_param->fname != NULL) && (strlen(m_decode_param->fname) > 0))
                {
                    m_decode_param->is_quit = false;
                    m_running_thread =
                        make_shared<thread>(&SrPyDecode::decode_func,
                                            this, static_cast<void *>(m_decode_param.get()));
                }
            }
        }

        if (!m_decode_param->is_quit)
        {
            sem_wait(&m_decode_param->read_done);
            *frame_cnt = m_decode_param->frame_count;
        }

        if (l_chn != video_chn || type != l_type ||
            width != l_width || height != l_height || feed_mode != l_mode)
        {
            printf("Invalid decode param, must be same as before(chn:%d type:%d w:%d h:%d mode:%d)\n",
                   l_chn, l_type, l_width, l_height, l_mode);
            return -1;
        }

        return 0;
    }

    int SrPyDecode::do_decoding()
    {
        if (!m_dec_inited.test_and_set())
        {
            if (vdec_init())
                return -1;

            if (!m_start_once.test_and_set())
            {
                m_decode_param = make_unique<sr_codec_param_t>();
                m_decode_param->fname = m_dec_file.data();
                m_decode_param->is_quit = false;
                if ((m_decode_param->fname != NULL) && (strlen(m_decode_param->fname) > 0))
                {
                    m_running_thread =
                        make_shared<thread>(&SrPyDecode::decode_func,
                                            this, static_cast<void *>(m_decode_param.get()));
                }
            }

            if (vdec_start())
                return -1;
        }

        return 0;
    }

    int SrPyDecode::undo_decoding()
    {
        if (!m_dec_inited.test_and_set())
        {
            printf("Decoder channel dose not created!\n");
            m_dec_inited.clear();
            return -1;
        }

        m_dec_inited.clear();
        m_decode_param->is_quit = true;
        if (m_running_thread && m_running_thread->joinable())
        {
            m_running_thread->join();
        }

        vdec_stop();

        m_start_once.clear();

        sem_destroy(&m_decode_param->read_done);

        vdec_deinit();

        return 0;
    }
    int SrPyDecode::vdec_init()
    {
        int ret = 0;
        mc_video_codec_dec_params_t *params;

        context = (media_codec_context_t *)malloc(sizeof(media_codec_context_t));
        if (!context)
        {
            printf("%s:%d Failed to allocate memory for new media_codec_context_t.\n",
                   __FUNCTION__, __LINE__);
            return -1;
        }

        memset(context, 0x00, sizeof(media_codec_context_t));

        context->encoder = false; // decoder output
        context->instance_index = m_chn;
        params = &context->video_dec_params;
        params->feed_mode = static_cast<mc_av_stream_feeding_mode_t>(m_feed_mode);
        params->pix_fmt = MC_PIXEL_FORMAT_NV12;
        params->bitstream_buf_size = m_width * m_height * 3 / 2;
        params->bitstream_buf_count = 6;
        params->frame_buf_count = 6;

        switch (m_type)
        {
        case TYPE_H264:
            context->codec_id = MEDIA_CODEC_ID_H264;
            params->h264_dec_config.bandwidth_Opt = true;
            params->h264_dec_config.reorder_enable = true;
            params->h264_dec_config.skip_mode = 0;
            break;
        case TYPE_H265:
            context->codec_id = MEDIA_CODEC_ID_H265;
            params->h265_dec_config.bandwidth_Opt = true;
            params->h265_dec_config.reorder_enable = true;
            params->h265_dec_config.skip_mode = 0;
            params->h265_dec_config.cra_as_bla = false;
            params->h265_dec_config.dec_temporal_id_mode = 0;
            params->h265_dec_config.target_dec_temporal_id_plus1 = 0;
            break;
        case TYPE_MJPEG:
            context->codec_id = MEDIA_CODEC_ID_MJPEG;
            break;
        case TYPE_JPEG:
            context->codec_id = MEDIA_CODEC_ID_JPEG;
            break;
        default:
            printf("%s:%d Not Support decoding type: %d!\n", __FUNCTION__,
                   __LINE__, m_type);
            free(context);
            return -1;
        }

        ret = hb_mm_mc_initialize(context);
        if (0 != ret)
        {
            printf("%s:%d hb_mm_mc_initialize Failed. ret = %d \n",
                   __FUNCTION__, __LINE__, ret);
            return -1;
        }

        ret = hb_mm_mc_configure(context);
        if (0 != ret)
        {
            printf("%s:%d hb_mm_mc_configure Failed. ret = %d \n",
                   __FUNCTION__, __LINE__, ret);
            hb_mm_mc_release(context);
            free(context);
            return -1;
        }

        return ret;
    }

    int SrPyDecode::vdec_deinit()
    {
        if (!m_dec_inited.test_and_set())
        {
            printf("Decoder channel dose not created!\n");
            m_dec_inited.clear();
            return -1;
        }
        int ret = 0;
        ret = hb_mm_mc_release(context);
        if (ret != 0)
        {
            printf("Failed to hb_mm_mc_release ret = %d \n", ret);
            return -1;
        }
        return ret;
    }

    int SrPyDecode::vdec_start()
    {
        if (!m_dec_inited.test_and_set())
        {
            printf("Decoder channel dose not created!\n");
            m_dec_inited.clear();
            return -1;
        }
        int ret = 0;
        mc_av_codec_startup_params_t startup_params;
        memset(&startup_params, 0x00, sizeof(mc_av_codec_startup_params_t));
        ret = hb_mm_mc_start(context, &startup_params);
        if (ret != 0)
        {
            printf("%s:%d hb_mm_mc_start failed.\n", __FUNCTION__, __LINE__);
            return -1;
        }
        return 0;
    }

    int SrPyDecode::vdec_stop()
    {
        if (!m_dec_inited.test_and_set())
        {
            printf("Decoder channel dose not created!\n");
            m_dec_inited.clear();
            return -1;
        }
        int ret = 0;
        ret = hb_mm_mc_pause(context);
        if (ret != 0)
        {
            printf("%s:%d Failed to hb_mm_mc_pause ret = %d \n",
                   __FUNCTION__, __LINE__, ret);
            return -1;
        }
        return 0;
    }

    int SrPyDecode::vdec_restart()
    {
        if (!m_dec_inited.test_and_set())
        {
            printf("Decoder channel dose not created!\n");
            m_dec_inited.clear();
            return -1;
        }

        int ret = 0;
        ret = hb_mm_mc_stop(context);
        if (ret != 0)
        {
            printf("%s:%d Failed to hb_mm_mc_stop ret = %d \n",
                   __FUNCTION__, __LINE__, ret);
            return -1;
        }
        return 0;
    }

    ImageFrame *SrPyDecode::get_frame()
    {
        if (!m_dec_inited.test_and_set())
        {
            printf("Decoder channel dose not created!\n");
            m_dec_inited.clear();
            return nullptr;
        }

        int ret = 0;
        static int64_t frame_id = 0;

        media_codec_buffer_t *buffer;
        media_codec_output_buffer_info_t info;
        memset(&info, 0x00, sizeof(media_codec_output_buffer_info_t));

        buffer = (media_codec_buffer_t *)malloc(sizeof(media_codec_buffer_t));
        if (buffer == NULL)
        {
            printf("malloc media_codec_buffer_t failed.\n");
            return nullptr;
        }

        ret = hb_mm_mc_dequeue_output_buffer(context, buffer, &info, 2000);
        if (ret != 0)
        {
            printf("hb_mm_mc_dequeue_output_buffer failed ret = %d \n", ret);
            free(buffer);
            return nullptr;
        }

        if (buffer->type != MC_VIDEO_FRAME_BUFFER)
        {
            free(buffer);
            return nullptr;
        }

        media_codec_id_t codec_type = context->codec_id;
        if (codec_type == MEDIA_CODEC_ID_H264 || codec_type == MEDIA_CODEC_ID_H265)
        {
            // mc_h264_h265_output_frame_info_t *frame_info = &info.video_frame_info;

            m_dec_frame = make_unique<ImageFrame>();
            m_dec_frame->data[0] = reinterpret_cast<uint8_t *>(buffer->vframe_buf.vir_ptr[0]);
            m_dec_frame->data[1] = reinterpret_cast<uint8_t *>(buffer->vframe_buf.vir_ptr[1]);
            m_dec_frame->image_id = frame_id++;
            m_dec_frame->image_timestamp = static_cast<int64_t>(time(nullptr));
            m_dec_frame->data_size[0] = m_width * m_height;
            m_dec_frame->data_size[1] = m_width * m_height / 2;
            m_dec_frame->frame_info = static_cast<void *>(buffer);
            m_dec_frame->plane_count = 2;
        }

        return m_dec_frame.get();
    }

    int SrPyDecode::put_frame(ImageFrame *frame)
    {
        if (!m_dec_inited.test_and_set())
        {
            printf("Decoder channel dose not created!\n");
            m_dec_inited.clear();
            return -1;
        }

        int ret = 0;
        media_codec_buffer_t *buffer = static_cast<media_codec_buffer_t *>(frame->frame_info);

        ret = hb_mm_mc_queue_output_buffer(context, buffer, 0);
        if (ret != 0)
        {
            printf("hb_mm_mc_queue_output_buffer failed ret = %d \n", ret);
            free(buffer);
            return -1;
        }

        free(buffer);

        if (m_dec_frame)
        {
            m_dec_frame.reset();
        }
        return ret;
    }

    int SrPyDecode::send_frame(int chn, void *addr, int size, int eos)
    {
        int ret = 0;

        printf("%s:%d \n", __FUNCTION__, __LINE__);

        if (!m_dec_inited.test_and_set())
        {
            printf("Decoder channel dose not created!\n");
            m_dec_inited.clear();
            return -1;
        }

        if (addr == NULL)
        {
            printf("Invalid image address!\n");
            return -1;
        }

        media_codec_buffer_t buffer;
        buffer.type = MC_VIDEO_STREAM_BUFFER;
        ret = hb_mm_mc_dequeue_input_buffer(context, &buffer, 3000);
        if (ret != 0)
        {
            printf("%s:%d hb_mm_mc_dequeue_input_buffer failed ret = %d \n", __FUNCTION__,
                   __LINE__, ret);
            return -1;
        }

        if (buffer.vstream_buf.size < size)
        {
            printf("The input stream/frame data is larger than the stream buffer size\n");
            hb_mm_mc_queue_input_buffer(context, &buffer, 3000);
            return -1;
        }

        buffer.type = MC_VIDEO_STREAM_BUFFER;
        if (eos == 0)
        {
            buffer.vframe_buf.size = size;
            buffer.vstream_buf.stream_end = 0;
        }
        else
        {
            buffer.vframe_buf.size = 0;
            buffer.vstream_buf.stream_end = 1;
        }

        memcpy(buffer.vstream_buf.vir_ptr, addr, size);

        ret = hb_mm_mc_queue_input_buffer(context, &buffer, 100);
        if (ret != 0)
        {
            printf("hb_mm_mc_queue_input_buffer failed, ret = 0x%x\n", ret);
            return -1;
        }

        return ret;
    }

    void SrPyDecode::do_sync_decoding(void *param)
    {
        int error = 0;
        int ret = 0;
        AVFormatContext *avContext = nullptr;
        AVPacket avpacket = {0};
        int video_idx = -1;
        uint8_t *seqHeader = nullptr;
        int seqHeaderSize = 0;
        int firstPacket = 1;
        bool eos = false;
        int bufSize = 0;
        int pkt_cnt = 0;

        sr_codec_param_t *p_dec_param = static_cast<sr_codec_param_t *>(param);

        media_codec_id_t codec_id;
        if (m_type == TYPE_H264)
        {
            codec_id = MEDIA_CODEC_ID_H264;
        }
        else if (m_type == TYPE_H265)
        {
            codec_id = MEDIA_CODEC_ID_H265;
        }
        else if (m_type == TYPE_JPEG)
        {
            codec_id = MEDIA_CODEC_ID_JPEG;
        }
        else
        {
            printf("codec error type:%d\n", m_type);
            return;
        }

        video_idx = AV_open_stream(p_dec_param, &avContext, &avpacket);
        if (video_idx < 0)
        {
            printf("failed to x3_av_open_stream\n");
            goto err_av_open;
        }

        do
        {
            mc_inter_status_t pstStatus;
            hb_mm_mc_get_status(context, &pstStatus);

            // printf("Status: curIn=%d, curOut=%d, totalIn=%d, totalOut=%d.\n",
            //             pstStatus.cur_input_buf_cnt,
            //             pstStatus.cur_output_buf_cnt,
            //             pstStatus.total_input_buf_cnt,
            //             pstStatus.total_output_buf_cnt);

            // if (pstStatus.cur_output_buf_cnt >= pstStatus.total_output_buf_cnt)
            // {
            //     usleep(10 * 1000);
            //     continue;
            // }

            // wait for each frame for decoding
            // usleep(30 * 1000);
            if (p_dec_param->is_quit)
            {
                eos = true;
                break;
            }

            if (!avpacket.size)
            {
                error = av_read_frame(avContext, &avpacket);
            }

            if (error < 0)
            {
                if (error == AVERROR_EOF || avContext->pb->eof_reached == true)
                {
                    printf("There is no more input data, %d!\n", avpacket.size);
                    eos = false;
                    break;
                }
                else
                {
                    printf("Failed to av_read_frame error(0x%08x)\n", error);
                }

                if (avContext)
                {
                    avformat_close_input(&avContext);
                }
                if (p_dec_param->fname != NULL)
                {
                    avContext = nullptr;
                    memset(&avpacket, 0, sizeof(avpacket));
                    video_idx = AV_open_stream(p_dec_param, &avContext, &avpacket);
                    if (video_idx < 0)
                    {
                        printf("failed to x3_av_open_stream\n");
                        goto err_av_open;
                    }
                }
                else
                {
                    eos = true;
                }
            }
            else
            {
                seqHeaderSize = 0;
                if (firstPacket)
                {
                    AVCodecParameters *codec;
                    int retSize = 0;
                    codec = avContext->streams[video_idx]->codecpar;
                    seqHeader = (uint8_t *)calloc(1U, codec->extradata_size + 1024);
                    if (seqHeader == nullptr)
                    {
                        printf("Failed to mallock seqHeader\n");
                        eos = true;
                        break;
                    }

                    seqHeaderSize = AV_build_dec_seq_header(seqHeader,
                                                            codec_id,
                                                            avContext->streams[video_idx], &retSize);
                    if (seqHeaderSize < 0)
                    {
                        printf("Failed to build seqHeader\n");
                        eos = true;
                        break;
                    }
                    firstPacket = 0;
                }
                if (avpacket.size <= m_width * m_height * 3 / 2)
                {
                    printf("%s:%d \n", __FUNCTION__, __LINE__);
                    if (seqHeaderSize)
                    {
                        send_frame(m_chn, (void *)seqHeader, seqHeaderSize, eos);
                    }
                    else
                    {
                        send_frame(m_chn, (void *)avpacket.data, avpacket.size, eos);
                        av_packet_unref(&avpacket);
                        avpacket.size = 0;
                    }
                }
                else
                {
                    printf("The external stream buffer is too small!"
                           "avpacket.size:%d, buffer size:%d\n",
                           avpacket.size, m_width * m_height * 3 / 2);
                    eos = true;
                    break;
                }

                if (seqHeader)
                {
                    free(seqHeader);
                    seqHeader = nullptr;
                }
            }

            if (eos)
            {
                p_dec_param->is_quit = 1;
            }
        } while (!p_dec_param->is_quit);

        if (eos)
        {
            send_frame(m_chn, (void *)seqHeader, seqHeaderSize, eos);
        }

        if (seqHeader)
        {
            free(seqHeader);
            seqHeader = nullptr;
        }

    err_av_open:
        if (avContext)
            avformat_close_input(&avContext);
    }

    void SrPyDecode::decode_func(void *param)
    {
        if (!m_dec_inited.test_and_set())
        {
            printf("Decoder channel dose not created!\n");
            m_dec_inited.clear();
            return;
        }
        if (!param)
        {
            printf("Invalid param!\n");
            return;
        }

        do_sync_decoding(param);
    }

} // namespace srpy_cam