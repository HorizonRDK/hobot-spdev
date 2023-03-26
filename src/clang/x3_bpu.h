/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_BPU_H_
#define X3_BPU_H_

#include "sp_bpu.h"
#include "vio/hb_vio_interface.h"
#include "dnn/hb_dnn.h"

#ifdef __cplusplus
extern "C"
{
#endif

bpu_module *x3_bpu_predict_init(const char *model_file_name);

int x3_bpu_init_tensors(bpu_module *bpu_handle, hbDNNTensor *output_tensors);
int x3_bpu_deinit_tensor(hbDNNTensor *tensor, int len);
int x3_bpu_start_predict(bpu_module *bpu_handle, char *frame_buffer);
int x3_bpu_predict_unint(bpu_module *handle);
#ifdef __cplusplus
}
#endif

#endif // X3_BPU_H_