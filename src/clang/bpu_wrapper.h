/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-04-11 11:22:11
 * @LastEditTime: 2023-04-11 15:10:28
 ***************************************************************************/
#ifndef BPU_WRAPPERH_
#define BPU_WRAPPERH_

#include "sp_bpu.h"
#include "dnn/hb_dnn.h"

#ifdef __cplusplus
extern "C"
{
#endif

bpu_module *x3_bpu_predict_init(const char *model_file_name);

int32_t x3_bpu_init_tensors(bpu_module *bpu_handle, hbDNNTensor *output_tensors);
int32_t x3_bpu_deinit_tensor(hbDNNTensor *tensor, int32_t len);
int32_t x3_bpu_start_predict(bpu_module *bpu_handle, char *frame_buffer);
int32_t x3_bpu_predict_unint(bpu_module *handle);
#ifdef __cplusplus
}
#endif

#endif // BPU_WRAPPERH_