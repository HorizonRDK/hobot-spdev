#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <stdbool.h>
#include <future>
#include "sp_bpu.h"

#include "bpu_wrapper.h"
#include "dnn/hb_dnn.h"
static void print_model_info(hbPackedDNNHandle_t packed_dnn_handle);

#define ALIGN_16(v) ((v + (16 - 1)) / 16 * 16)

void HB_CHECK_SUCCESS(int32_t value, char *errmsg)
{
    /*value can be call of function*/
    int32_t ret_code = value;
    if (ret_code != 0)
    {
        printf("[BPU MSG] %s, return code:%d\n", errmsg, ret_code);
    }
}

bpu_module *x3_bpu_predict_init(const char *model_file_name)
{

    bpu_module *bpu_handle = (bpu_module *)malloc(sizeof(bpu_module));
    memset(bpu_handle, 0, sizeof(bpu_module));
    //第一步加载模型
    hbPackedDNNHandle_t packed_dnn_handle;
    HB_CHECK_SUCCESS(hbDNNInitializeFromFiles(&packed_dnn_handle, static_cast<const char **>(&model_file_name), 1), "hbDNNInitializeFromFiles fail");

    // 第二步获取模型名称
    const char **model_name_list;
    int32_t model_count = 0;
    HB_CHECK_SUCCESS(hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle), "hbDNNGetModelNameList fail");

    // 第三步获取dnn_handle
    hbDNNHandle_t dnn_handle;
    HB_CHECK_SUCCESS(hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name_list[0]), "hbDNNGetModelHandle fail");

    bpu_handle->m_packed_dnn_handle = packed_dnn_handle;
    bpu_handle->m_dnn_handle = dnn_handle;

    //input size alloc
    hbDNNTensorProperties input_properties;
    hbDNNGetInputTensorProperties(&input_properties, bpu_handle->m_dnn_handle, 0);
    bpu_handle->input_tensor.properties = input_properties;
    hbSysAllocCachedMem(bpu_handle->input_tensor.sysMem, input_properties.validShape.dimensionSize[2] * input_properties.validShape.dimensionSize[3] * 3 / 2);

    print_model_info(bpu_handle->m_packed_dnn_handle);

    return bpu_handle;
}

int32_t x3_bpu_init_tensors(bpu_module *bpu_handle, hbDNNTensor *output_tensors)
{
    int32_t ret = -1;
    int32_t output_count;
    auto dnn_handle = bpu_handle->m_dnn_handle;
    ret = hbDNNGetOutputCount(&output_count, dnn_handle);
    if (ret)
    {
        printf("[BPU ERR] %s:hbDNNGetOutputCount failed!Error code:%d\n", __func__, ret);
        return ret;
    }
    // output_tensors = new hbDNNTensor[output_count];
    for (int32_t i = 0; i < output_count; i++)
    {
        hbDNNTensorProperties &output_properties = output_tensors[i].properties;
        ret = hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, i);
        if (ret)
        {
            printf("[BPU ERR] %s:hbDNNGetOutputTensorProperties failed!Error code:%d\n", __func__, ret);
            return ret;
        }
        // 获取模型输出尺寸
        int32_t out_aligned_size = 4;
        for (int32_t j = 0; j < output_properties.alignedShape.numDimensions; j++)
        {
            out_aligned_size =
                out_aligned_size * output_properties.alignedShape.dimensionSize[j];
        }
        hbSysMem &mem = output_tensors[i].sysMem[0];
        ret = hbSysAllocCachedMem(&mem, out_aligned_size);
        if (ret)
        {
            printf("[BPU ERR] %s:hbSysAllocCachedMem failed!Error code:%d\n", __func__, ret);
            return ret;
        }
    }
    return ret;
}

int32_t x3_bpu_deinit_tensor(hbDNNTensor *tensor, int32_t len)
{
    int32_t ret = -1;
    for (size_t i = 0; i < len; i++)
    {
        ret = hbSysFreeMem(&(tensor[i].sysMem[0]));
    }
    return ret;
}

int32_t x3_bpu_start_predict(bpu_module *bpu_handle, char *frame_buffer)
{
    // copy NV12data from frame_buffer to input tensor
    int32_t height = bpu_handle->input_tensor.properties.validShape.dimensionSize[2];
    int32_t width = bpu_handle->input_tensor.properties.validShape.dimensionSize[3];
    int32_t yuv_length = height * width * 3 / 2;
    memcpy(bpu_handle->input_tensor.sysMem[0].virAddr, frame_buffer, yuv_length);
    hbSysFlushMem(bpu_handle->input_tensor.sysMem, HB_SYS_MEM_CACHE_CLEAN);

    auto dnn_handle = bpu_handle->m_dnn_handle;
    auto input = bpu_handle->input_tensor;
    hbDNNTaskHandle_t task_handle = nullptr;
    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
    hbDNNInfer(&task_handle,
               &(bpu_handle->output_tensor),
               &(bpu_handle->input_tensor),
               bpu_handle->m_dnn_handle,
               &infer_ctrl_param);
    // 第七步等待任务结束
    hbDNNWaitTaskDone(task_handle, 0);
    hbSysFlushMem(&(bpu_handle->output_tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    // 释放task handle
    hbDNNReleaseTask(task_handle);
    return 0;
}

int32_t x3_bpu_predict_unint(bpu_module *handle)
{
    hbSysFreeMem(&(handle->input_tensor.sysMem[0]));
    hbDNNRelease(handle->m_packed_dnn_handle);
    free(handle);
    return 0;
}

static void print_model_info(hbPackedDNNHandle_t packed_dnn_handle)
{
    int32_t i = 0, j = 0;
    hbDNNHandle_t dnn_handle;
    const char **model_name_list;
    int32_t model_count = 0;
    hbDNNTensorProperties properties;

    hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle);
    if (model_count <= 0)
    {
        printf("Modle count <= 0\n");
        // return;
    }
    HB_CHECK_SUCCESS(
        hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name_list[0]),
        "hbDNNGetModelHandle failed");

    printf("Model info:\nmodel_name: %s", model_name_list[0]);

    int32_t input_count = 0;
    int32_t output_count = 0;
    HB_CHECK_SUCCESS(hbDNNGetInputCount(&input_count, dnn_handle),
                     "hbDNNGetInputCount failed");
    HB_CHECK_SUCCESS(hbDNNGetOutputCount(&output_count, dnn_handle),
                     "hbDNNGetInputCount failed");

    printf("Input count: %d", input_count);
    for (i = 0; i < input_count; i++)
    {
        HB_CHECK_SUCCESS(
            hbDNNGetInputTensorProperties(&properties, dnn_handle, i),
            "hbDNNGetInputTensorProperties failed");

        printf("input[%d]: tensorLayout: %d tensorType: %d validShape:(",
               i, properties.tensorLayout, properties.tensorType);
        for (j = 0; j < properties.validShape.numDimensions; j++)
            printf("%d, ", properties.validShape.dimensionSize[j]);
        printf("), ");
        printf("alignedShape:(");
        for (j = 0; j < properties.alignedShape.numDimensions; j++)
            printf("%d, ", properties.alignedShape.dimensionSize[j]);
        printf(")\n");
    }

    printf("Output count: %d", output_count);
    for (i = 0; i < output_count; i++)
    {
        HB_CHECK_SUCCESS(
            hbDNNGetOutputTensorProperties(&properties, dnn_handle, i),
            "hbDNNGetOutputTensorProperties failed");
        printf("Output[%d]: tensorLayout: %d tensorType: %d validShape:(",
               i, properties.tensorLayout, properties.tensorType);
        for (j = 0; j < properties.validShape.numDimensions; j++)
            printf("%d, ", properties.validShape.dimensionSize[j]);
        printf("), ");
        printf("alignedShape:(");
        for (j = 0; j < properties.alignedShape.numDimensions; j++)
            printf("%d, ", properties.alignedShape.dimensionSize[j]);
        printf(")\n");
    }
}