#ifndef X3_PREPARAM_H
#define X3_PREPARAM_H

#include "x3_sdk_wrap.h"

#ifdef __cplusplus
extern "C" {
#endif

#if 0
int f37_linear_vin_param_init(x3_vin_info_t* vin_info);
int os8a10_linear_vin_param_init(x3_vin_info_t* vin_info);
int imx415_linear_vin_param_init(x3_vin_info_t* vin_info);
int ov8856_linear_vin_param_init(x3_vin_info_t* vin_info);
int sc031gs_linear_vin_param_init(x3_vin_info_t* vin_info);
#endif

int vin_param_init(const int video_index, x3_vin_info_t* vin_info);

int vps_grp_param_init(x3_vps_info_t *vps_info, int pipe_id, int width, int height);
int vps_chn_param_init(x3_vps_chn_attr_t *vps_chn_attr, int chn_id, int width, int height, int fps);
int vps_chn_crop_param_init(x3_vps_chn_attr_t *vps_chn_attr, int x, int y, int width, int height);
int vps_chn_rotate_param_init(x3_vps_chn_attr_t *vps_chn_attr, int rotate);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // X3_PREPARAM_H
