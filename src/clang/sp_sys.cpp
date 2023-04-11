#include <thread>
using namespace std;

#include "x3_sdk_codec.h"
#include "x3_sdk_display.h"
#include "x3_sdk_camera.h"
#include "sp_sys.h"

using namespace srpy_cam;


int32_t sp_module_bind(void *src, int32_t src_type, void *dst, int32_t dst_type)
{
    struct HB_SYS_MOD_S src_mod, dst_mod;
    int32_t ret = 0, width = 0, height = 0;
    if (dst_type == VPP_ENCODE)
    {
        dst_mod.enModId = HB_ID_VENC; 
        auto dst_obj = static_cast<VPPEncode *>(dst);
        dst_mod.s32DevId = dst_obj->m_enc_obj.get()->m_chn;
        dst_mod.s32ChnId = 0;
        width = dst_obj->m_enc_obj.get()->m_width;
        height = dst_obj->m_enc_obj.get()->m_height;
    }
    else if (dst_type == VPP_DECODE)
    {
        dst_mod.enModId = HB_ID_VDEC;
        auto dst_obj = static_cast<VPPDecode *>(dst);
        dst_mod.s32DevId = dst_obj->m_dec_obj.get()->m_chn;
        dst_mod.s32ChnId = 0;
        width = dst_obj->m_dec_obj.get()->m_width;
        height = dst_obj->m_dec_obj.get()->m_height;
    }
    else if (dst_type == VPP_DISPLAY)
    {
        dst_mod.enModId = HB_ID_VOT;
        dst_mod.s32DevId = 0;
        auto dst_obj = static_cast<VPPDisplay *>(dst);
        dst_mod.s32ChnId = dst_obj->get_video_chn();
        width = dst_obj->m_chn_width[dst_mod.s32ChnId];
        height = dst_obj->m_chn_height[dst_mod.s32ChnId];
    }
    else if (dst_type == VPP_CAMERA)
    {
        dst_mod.enModId = HB_ID_VPS;
        dst_mod.s32DevId = 0;
        dst_mod.s32ChnId = 0;
    }
    else
    {
        printf("bind error dst object:%d\n", dst_type);
    }
    if (src_type == VPP_CAMERA)
    {
        src_mod.enModId = HB_ID_VPS;
        auto src_obj = static_cast<VPPCamera *>(src);
        src_mod.s32DevId = src_obj->GetPipeId();
        src_mod.s32ChnId = src_obj->GetChnId(static_cast<Sdk_Object_e>(dst_type), 1, width, height);
    }
    else if (src_type == VPP_ENCODE)
    {
        src_mod.enModId = HB_ID_VENC;
        auto src_obj = static_cast<VPPEncode *>(src);
        src_mod.s32DevId = src_obj->m_enc_obj.get()->m_chn;
        src_mod.s32ChnId = 0;
        width = src_obj->m_enc_obj.get()->m_width;
        height = src_obj->m_enc_obj.get()->m_height;
    }
    else if (src_type == VPP_DECODE)
    {
        src_mod.enModId = HB_ID_VDEC;
        auto src_obj = static_cast<VPPDecode *>(src);
        src_mod.s32DevId = src_obj->m_dec_obj.get()->m_chn;
        src_mod.s32ChnId = 0;
        width = src_obj->m_dec_obj.get()->m_width;
        height = src_obj->m_dec_obj.get()->m_height;
    }
    else
    {
        printf("bind error src object:%d\n", src_type);
    }
    ret = HB_SYS_Bind(&src_mod, &dst_mod);
    if (ret != 0)
    {
        printf("HB_SYS_Bind failed, src:%d pipe:%d chn:%d dst:%d pipe:%d chn:%d\n",
              src_mod.enModId, src_mod.s32DevId, src_mod.s32ChnId,
              dst_mod.enModId, dst_mod.s32DevId, dst_mod.s32ChnId);
    }

    return ret;
}

int32_t sp_module_unbind(void *src, int32_t src_type, void *dst, int32_t dst_type)
{
    struct HB_SYS_MOD_S src_mod, dst_mod;
    int32_t ret = 0, width = 0, height = 0;
    if (dst_type == VPP_ENCODE)
    {
        dst_mod.enModId = HB_ID_VENC;
        auto dst_obj = static_cast<VPPEncode *>(dst);
        dst_mod.s32DevId = dst_obj->m_enc_obj.get()->m_chn;
        dst_mod.s32ChnId = 0;
        width = dst_obj->m_enc_obj.get()->m_width;
        height = dst_obj->m_enc_obj.get()->m_height;
    }
    else if (dst_type == VPP_DECODE)
    {
        dst_mod.enModId = HB_ID_VDEC;
        auto dst_obj = static_cast<VPPDecode *>(dst);
        dst_mod.s32DevId = dst_obj->m_dec_obj.get()->m_chn;
        dst_mod.s32ChnId = 0;
        width = dst_obj->m_dec_obj.get()->m_width;
        height = dst_obj->m_dec_obj.get()->m_height;
    }
    else if (dst_type == VPP_DISPLAY)
    {
        dst_mod.enModId = HB_ID_VOT;
        dst_mod.s32DevId = 0;
        auto dst_obj = static_cast<VPPDisplay *>(dst);
        dst_mod.s32ChnId = dst_obj->get_video_chn();
        width = dst_obj->m_chn_width[dst_mod.s32ChnId];
        height = dst_obj->m_chn_height[dst_mod.s32ChnId];
    }
    else if (dst_type == VPP_CAMERA)
    {
        dst_mod.enModId = HB_ID_VPS;
        dst_mod.s32DevId = 0;
        dst_mod.s32ChnId = 0;
    }
    else
    {
        printf("unbind error dst object:%d\n", dst_type);
    }
    if (src_type == VPP_CAMERA)
    {
        src_mod.enModId = HB_ID_VPS;
        auto src_obj = static_cast<VPPCamera *>(src);
        src_mod.s32DevId = src_obj->GetPipeId();
        src_mod.s32ChnId = src_obj->GetChnId(static_cast<Sdk_Object_e>(dst_type), 0, width, height);
    }
    else if (src_type == VPP_ENCODE)
    {
        src_mod.enModId = HB_ID_VENC;
        auto src_obj = static_cast<VPPEncode *>(src);
        src_mod.s32DevId = src_obj->m_enc_obj.get()->m_chn;
        src_mod.s32ChnId = 0;
        width = src_obj->m_enc_obj.get()->m_width;
        height = src_obj->m_enc_obj.get()->m_height;
    }
    else if (src_type == VPP_DECODE)
    {
        src_mod.enModId = HB_ID_VDEC;
        auto src_obj = static_cast<VPPDecode *>(src);
        src_mod.s32DevId = src_obj->m_dec_obj.get()->m_chn;
        src_mod.s32ChnId = 0;
        width = src_obj->m_dec_obj.get()->m_width;
        height = src_obj->m_dec_obj.get()->m_height;
    }
    else
    {
        printf("unbind error src object:%d\n", src_type);
    }
    ret = HB_SYS_UnBind(&src_mod, &dst_mod);

    return ret;
}