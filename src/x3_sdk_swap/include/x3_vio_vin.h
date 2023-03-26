/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-02-23 11:57:45
 * @LastEditTime: 2023-03-05 15:56:36
 ***************************************************************************/
#ifndef X3_VIO_VIN_H_
#define X3_VIO_VIN_H_

#include "stddef.h"
#include "stdint.h"

#include "vio/hb_vio_interface.h"
#include "x3_sdk_wrap.h"

#ifdef __cplusplus
extern "C" {
#endif

int x3_vin_init(x3_vin_info_t *vin_info);
int x3_vin_start(x3_vin_info_t *vin_info);
void x3_vin_stop(x3_vin_info_t *vin_info);
void x3_vin_deinit(x3_vin_info_t *vin_info);
int x3_vin_sif_get_data(const int pipe_id, hb_vio_buffer_t *sif_raw, int timeout);
int x3_vin_sif_release_data(const int pipe_id, hb_vio_buffer_t *sif_raw);
int x3_vin_isp_get_data(const int pipe_id, hb_vio_buffer_t *isp_yuv, int timeout);
int x3_vin_isp_release_data(const int pipe_id, hb_vio_buffer_t *isp_yuv);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // X3_VIO_VIN_H_
