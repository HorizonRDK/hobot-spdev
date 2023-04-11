/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_VIO_VIN_H_
#define X3_VIO_VIN_H_

#include "stddef.h"
#include "stdint.h"
#include "vio/hb_mode.h"
#include "vio/hb_vio_interface.h"
#include "x3_sdk_wrap.h"

#ifdef __cplusplus
extern "C" {
#endif

int x3_vin_init(x3_vin_info_t *vin_info);
int x3_vin_start(x3_vin_info_t *vin_info);
void x3_vin_stop(x3_vin_info_t *vin_info);
void x3_vin_deinit(x3_vin_info_t *vin_info);
int x3_vin_feedback(int pipeId, hb_vio_buffer_t *feedback_buf);
int x3_vin_get_ouput(int pipeId, hb_vio_buffer_t *buffer);
int x3_vin_output_release(int pipeId, hb_vio_buffer_t *buffer);
int x3_vin_sif_raw_dump(int pipeId, char *file_name);
int x3_vin_isp_yuv_dump(int pipeId, char *file_name);
int x3_vin_isp_yuv_dump_to_jpeg(VENC_CHN, int pipeId, char *file_name);
int x3_vin_sif_get_data(const int pipe_id, hb_vio_buffer_t *sif_raw, int timeout);
int x3_vin_sif_release_data(const int pipe_id, hb_vio_buffer_t *sif_raw);
int x3_vin_isp_get_data(const int pipe_id, hb_vio_buffer_t *isp_yuv, int timeout);
int x3_vin_isp_release_data(const int pipe_id, hb_vio_buffer_t *isp_yuv);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // X3_VIO_VIN_H_
