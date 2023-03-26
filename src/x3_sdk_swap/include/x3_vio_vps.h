/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-01-30 11:27:41
 * @LastEditTime: 2023-03-05 15:57:35
 ***************************************************************************/
#ifndef X3_VIO_VPS_H_
#define x3_VIO_VPS_H_

#include "vio/hb_vio_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

int x3_vps_select_chn(int *chn_en, int src_width, int src_height, int dst_width, int dst_height);
int x3_vps_start(uint32_t pipeline_id);
void x3_vps_stop(int pipeline_id);
void x3_vps_deinit(int pipeline_id);
int x3_vps_input(uint32_t pipeline_id, hb_vio_buffer_t *buffer);
int x3_vps_get_output(uint32_t pipeline_id, int channel, hb_vio_buffer_t *buffer,
        const int timeout);
int x3_vps_get_output_cond(uint32_t pipeline_id, int channel,
        hb_vio_buffer_t *buffer, const int timeout);
int x3_vps_output_release(uint32_t pipeline_id, int channel,
                          hb_vio_buffer_t *buffer);
#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // X3_VIO_VPS_H_
