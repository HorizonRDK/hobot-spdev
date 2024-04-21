#ifndef SP_BPU
#define SP_BPU
#include <stdint.h>
#include "dnn/hb_dnn.h"
#define SP_PREDICT_TYPE_YOLOV5 1
#define SP_PREDICT_TYPE_MOBILENET 2
#define SP_PREDICT_TYPE_FCOS 3
#ifdef __cplusplus
extern "C"
{
#endif
  // 这个结构体中存储的数据用来后处理时进行坐标还原
  typedef struct
  {
    int32_t m_model_w;    // 模型输入宽
    int32_t m_model_h;    // 模型输入高
    int32_t m_ori_width;  // 用户看到的原始图像宽，比如web上显示的推流视频
    int32_t m_ori_height; // 用户看到的原始图像高，比如web上显示的推流视频
  } bpu_image_info_t;

  typedef struct
  {
    float xmin;
    float ymin;
    float xmax;
    float ymax;
    int32_t id;
    float score;
    //   const char* class_name;
  } classfiy_result;

  typedef struct
  {
    int32_t m_alog_type; // 1:mobilenetV2 2: yolo5 3:personMultitask
    hbPackedDNNHandle_t m_packed_dnn_handle;
    hbDNNHandle_t m_dnn_handle;
    hbDNNTensor input_tensor;
    hbDNNTensor *output_tensor;
  } bpu_module;

  bpu_module *sp_init_bpu_module(const char *model_file_name);

  int32_t sp_bpu_start_predict(bpu_module *bpu_handle, char *addr);

  int32_t sp_release_bpu_module(bpu_module *bpu_handle);
  int32_t sp_init_bpu_tensors(bpu_module *bpu_handle, hbDNNTensor *output_tensors);
  int32_t sp_deinit_bpu_tensor(hbDNNTensor *tensor, int32_t len);

#ifdef __cplusplus
}
#endif
#endif