#include <thread>
#include <sys/stat.h>
using namespace std;

#include "x3_sdk_codec.h"
#include "sp_codec.h"

using namespace srpy_cam;

void *sp_init_encoder_module()
{
    return new VPPEncode();
}

void sp_release_encoder_module(void *obj)
{
    if (obj != NULL)
    {
        delete static_cast<VPPEncode *>(obj);
        obj = NULL;
    }
}

int32_t sp_start_encode(void *obj, int32_t chn, int32_t type, int32_t width, int32_t height, int32_t bits)
{
    if (obj != NULL)
    {
        auto encoder_obj = static_cast<VPPEncode *>(obj);
        return encoder_obj->do_encoding(chn, type, width, height, bits);
    }
    return -1;
}

int32_t sp_stop_encode(void *obj)
{
    if (obj != NULL)
    {
        auto encoder_obj = static_cast<VPPEncode *>(obj);
        return encoder_obj->undo_encoding();
    }
    return -1;
}

int32_t sp_encoder_set_frame(void *obj, char *frame_buffer, int32_t size)
{
    if (obj != NULL)
    {
        auto encoder_obj = static_cast<VPPEncode *>(obj);
        return encoder_obj->encode_file(frame_buffer, size);
    }
    return -1;
}
int32_t sp_encoder_get_stream(void *obj, char *stream_buffer)
{
    if (obj != NULL)
    {
        auto encoder_obj = static_cast<VPPEncode *>(obj);
        auto frame_obj = encoder_obj->get_frame();
        if (frame_obj)
        {
            //printf("get frame success!,data addr:0x%x,size:%d\n", frame_obj->data[0], frame_obj->data_size[0]);
            int32_t frame_size = frame_obj->data_size[0];
            memcpy(stream_buffer, frame_obj->data[0], frame_size);
            encoder_obj->put_frame(frame_obj);
            return frame_size;
        }
    }
    return -1;
}

// decoder
void *sp_init_decoder_module()
{
    return new VPPDecode();
}

void sp_release_decoder_module(void *decoder_object)
{
    if (decoder_object != NULL)
    {
        delete static_cast<VPPDecode *>(decoder_object);
        decoder_object = NULL;
    }
}

int32_t sp_start_decode(void *decoder_object, const char *stream_file, int32_t video_chn, int32_t type, int32_t width, int32_t height)
{
    int32_t frame_rate = 0;
    if (decoder_object != NULL)
    {
        if (strlen(stream_file) > 0) {
            struct stat buffer; 
            if(stat(stream_file,&buffer)){
                printf("sp_start_decode(%s):%s \n", stream_file, strerror(errno));
                return -1;
            }
        }
        auto decoder_obj = static_cast<VPPDecode *>(decoder_object);
        return decoder_obj->do_decoding(stream_file, video_chn, type, width, height, &frame_rate, 1);
    }
    return -1;
}

int32_t sp_decoder_get_image(void *decoder_object, char *image_buffer)
{
    if (decoder_object != NULL && image_buffer != NULL)
    {
        auto decoder_obj = static_cast<VPPDecode *>(decoder_object);
        auto frame = decoder_obj->get_frame();
        if (frame)
        {
            memcpy(image_buffer, frame->data[0], frame->data_size[0]);
            memcpy(image_buffer + frame->data_size[0], frame->data[1], frame->data_size[1]);
            return decoder_obj->put_frame(frame);
        }
    }
    return -1;
}

int32_t sp_decoder_set_image(void *decoder_object, char *image_buffer, int32_t chn, int32_t size, int32_t eos)
{
    if (decoder_object != NULL && image_buffer != NULL)
    {
        auto decoder_obj = static_cast<VPPDecode *>(decoder_object);
        return decoder_obj->send_frame(chn, image_buffer, size, eos);
    }
    return 0;
}

int32_t sp_stop_decode(void *decoder_object)
{
    if (decoder_object != NULL)
    {
        auto decoder_obj = static_cast<VPPDecode *>(decoder_object);
        return decoder_obj->undo_decoding();
    }
    return -1;
}