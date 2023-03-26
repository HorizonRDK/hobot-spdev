/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-03-02 09:50:40
 * @LastEditTime: 2023-03-02 09:50:41
 ***************************************************************************/
#ifndef SP_CODEC_H_
#define SP_CODEC_H_
#define SP_ENCODER_H264  1
#define SP_ENCODER_H265  2
#define SP_ENCODER_MJPEG 3
#ifdef __cplusplus
extern "C"
{
#endif
       // encoder
       void *sp_init_encoder_module();
       void sp_release_encoder_module(void *obj);
       int sp_start_encode(void *obj, int chn, int type, int width, int height, int bits);
       int sp_stop_encode(void *obj);
       int sp_encoder_set_frame(void *obj, char *frame_buffer, int size);
       int sp_encoder_get_stream(void *obj, char *stream_buffer);

       // decoder
       void *sp_init_decoder_module();
       void sp_release_decoder_module(void *obj);
       int sp_start_decode(void *obj, const char *stream_file, int video_chn, int type, int width, int height);
       int sp_decoder_get_image(void *obj, char *image_buffer);
       int sp_decoder_set_image(void *obj, char *image_buffer, int chn, int size, int eos);
       int sp_stop_decode(void *obj);
#ifdef __cplusplus
}
#endif /* End of #ifdef __cplusplus */

#endif /* SP_CODEC_H_ */