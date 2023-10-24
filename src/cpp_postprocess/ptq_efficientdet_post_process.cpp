// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <queue>
#include <arm_neon.h>
#include <cassert>

#include "ptq_efficientdet_post_process.h"

/**
 * Config definition for EfficientDet
 */
struct EfficientDetConfig {
  std::vector<std::vector<double>> anchor_scales;
  std::vector<double> anchor_ratio;
  std::vector<int> feature_strides;
  int class_num;
  std::vector<std::string> class_names;
};

struct EDBaseAnchor {
  EDBaseAnchor(float x1, float y1, float x2, float y2)
      : x1_(x1), y1_(y1), x2_(x2), y2_(y2) {}
  float x1_;
  float y1_;
  float x2_;
  float y2_;
};

struct EDAnchor {
  EDAnchor(float c_x, float c_y, float w, float h)
      : c_x_(c_x), c_y_(c_y), w_(w), h_(h) {}
  float c_x_;
  float c_y_;
  float w_;
  float h_;
};


const int kEfficientDetClassNum = 80;

EfficientDetConfig default_efficient_det_config = {
    {{4.0, 5.039684199579493, 6.3496042078727974},
     {4.0, 5.039684199579493, 6.3496042078727974},
     {4.0, 5.039684199579493, 6.3496042078727974},
     {4.0, 5.039684199579493, 6.3496042078727974},
     {4.0, 5.039684199579493, 6.3496042078727974}},
    {0.5, 1, 2},
    {8, 16, 32, 64, 128},
    80,
    {"person",        "bicycle",      "car",
     "motorcycle",    "airplane",     "bus",
     "train",         "truck",        "boat",
     "traffic light", "fire hydrant", "stop sign",
     "parking meter", "bench",        "bird",
     "cat",           "dog",          "horse",
     "sheep",         "cow",          "elephant",
     "bear",          "zebra",        "giraffe",
     "backpack",      "umbrella",     "handbag",
     "tie",           "suitcase",     "frisbee",
     "skis",          "snowboard",    "sports ball",
     "kite",          "baseball bat", "baseball glove",
     "skateboard",    "surfboard",    "tennis racket",
     "bottle",        "wine glass",   "cup",
     "fork",          "knife",        "spoon",
     "bowl",          "banana",       "apple",
     "sandwich",      "orange",       "broccoli",
     "carrot",        "hot dog",      "pizza",
     "donut",         "cake",         "chair",
     "couch",         "potted plant", "bed",
     "dining table",  "toilet",       "tv",
     "laptop",        "mouse",        "remote",
     "keyboard",      "cell phone",   "microwave",
     "oven",          "toaster",      "sink",
     "refrigerator",  "book",         "clock",
     "vase",          "scissors",     "teddy bear",
     "hair drier",    "toothbrush"}};



/**
 * Finds the smallest element in the range [first, last).
 * @tparam[in] ForwardIterator
 * @para[in] first: first iterator
 * @param[in] last: last iterator
 * @return Iterator to the smallest element in the range [first, last)
 */
template <class ForwardIterator>
inline size_t argmin(ForwardIterator first, ForwardIterator last) {
  return std::distance(first, std::min_element(first, last));
}
 
/**
 * Finds the greatest element in the range [first, last)
 * @tparam[in] ForwardIterator: iterator type
 * @param[in] first: fist iterator
 * @param[in] last: last iterator
 * @return Iterator to the greatest element in the range [first, last)
 */
template <class ForwardIterator>
inline size_t argmax(ForwardIterator first, ForwardIterator last) {
  return std::distance(first, std::max_element(first, last));
}

/**
 * Bounding box definition
 */
typedef struct Bbox {
  float xmin;
  float ymin;
  float xmax;
  float ymax;
             
  Bbox() {}  
             
  Bbox(float xmin, float ymin, float xmax, float ymax)
      : xmin(xmin), ymin(ymin), xmax(xmax), ymax(ymax) {}
             
  friend std::ostream &operator<<(std::ostream &os, const Bbox &bbox) {
    os << "[" << std::fixed << std::setprecision(6) << bbox.xmin << ","
       << bbox.ymin << "," << bbox.xmax << "," << bbox.ymax << "]";
    return os; 
  }          
             
  ~Bbox() {} 
} Bbox;

typedef struct Detection {
  int id; 
  float score;
  Bbox bbox;
  const char *class_name;
  Detection() {}
 
  Detection(int id, float score, Bbox bbox)
      : id(id), score(score), bbox(bbox) {}
 
  Detection(int id, float score, Bbox bbox, const char *class_name)
      : id(id), score(score), bbox(bbox), class_name(class_name) {}
 
  friend bool operator>(const Detection &lhs, const Detection &rhs) {
    return (lhs.score > rhs.score);
  }
 
  friend std::ostream &operator<<(std::ostream &os, const Detection &det) {
    os << "{" 
       << R"("bbox")"
       << ":" << det.bbox << "," 
       << R"("score")"
       << ":" << det.score << "," 
       << R"("id")"
       << ":" << det.id << ","
       << R"("name")"
       << ":\"" << default_efficient_det_config.class_names[det.id] << "\"}";
    return os; 
  }
 
  ~Detection() {}
} Detection;

std::vector<Detection> efficient_det_dets;
std::vector<Detection> efficient_det_restuls;
static std::vector<std::vector<EDAnchor>> anchors_table;

static void efficient_det_nms(std::vector<Detection> &input,
               float iou_threshold,
               int top_k,
               std::vector<Detection> &result,
               bool suppress) {
  // sort order by score desc
  std::stable_sort(input.begin(), input.end(), std::greater<Detection>());

  std::vector<bool> skip(input.size(), false);

  // pre-calculate boxes area
  std::vector<float> areas;
  areas.reserve(input.size());
  for (size_t i = 0; i < input.size(); i++) {
    float width = input[i].bbox.xmax - input[i].bbox.xmin;
    float height = input[i].bbox.ymax - input[i].bbox.ymin;
    areas.push_back(width * height);
  }

  int count = 0;
  for (size_t i = 0; count < top_k && i < skip.size(); i++) {
    if (skip[i]) {
      continue;
    }
    skip[i] = true;
    ++count;

    for (size_t j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) {
        continue;
      }
      if (suppress == false) {
        if (input[i].id != input[j].id) {
          continue;
        }
      }

      // intersection area
      float xx1 = std::max(input[i].bbox.xmin, input[j].bbox.xmin);
      float yy1 = std::max(input[i].bbox.ymin, input[j].bbox.ymin);
      float xx2 = std::min(input[i].bbox.xmax, input[j].bbox.xmax);
      float yy2 = std::min(input[i].bbox.ymax, input[j].bbox.ymax);

      if (xx2 > xx1 && yy2 > yy1) {
        float area_intersection = (xx2 - xx1) * (yy2 - yy1);
        float iou_ratio =
            area_intersection / (areas[j] + areas[i] - area_intersection);
        if (iou_ratio > iou_threshold) {
          skip[j] = true;
        }
      }
    }
    result.push_back(input[i]);
  }
}

static inline uint32x4x4_t CalculateIndex(uint32_t idx,
                                          float32x4_t a,
                                          float32x4_t b,
                                          uint32x4x4_t c) {
  uint32x4_t mask{0};
  mask = vcltq_f32(b, a);
  uint32x4_t vec_idx = {idx, idx + 1, idx + 2, idx + 3};
  uint32x4x4_t res = {{vbslq_u32(mask, vec_idx, c.val[0]), 0, 0, 0}};
  return res;
}

static inline float32x2_t CalculateMax(float32x4_t in) {
  auto pmax = vpmax_f32(vget_high_f32(in), vget_low_f32(in));
  return vpmax_f32(pmax, pmax);
}

static inline uint32_t CalculateVectorIndex(uint32x4x4_t vec_res_idx,
                                            float32x4_t vec_res_value) {
  uint32x4_t res_idx_mask{0};
  uint32x4_t mask_ones = vdupq_n_u32(0xFFFFFFFF);

  auto pmax = CalculateMax(vec_res_value);
  auto mask = vceqq_f32(vec_res_value, vcombine_f32(pmax, pmax));
  res_idx_mask = vandq_u32(vec_res_idx.val[0], mask);
  res_idx_mask = vaddq_u32(res_idx_mask, mask_ones);
  auto pmin =
      vpmin_u32(vget_high_u32(res_idx_mask), vget_low_u32(res_idx_mask));
  pmin = vpmin_u32(pmin, pmin);
  uint32_t res = vget_lane_u32(pmin, 0);
  return (res - 0xFFFFFFFF);
}

static std::pair<float, int> MaxScoreID(int32_t *input,
                                        float *scale,
                                        int length) {
  float init_res_value = input[0] * scale[0];
  float32x4_t vec_res_value = vdupq_n_f32(init_res_value);
  uint32x4x4_t vec_res_idx{{0}};
  int i = 0;
  for (; i <= (length - 4); i += 4) {
    int32x4_t vec_input = vld1q_s32(input + i);
    float32x4_t vec_scale = vld1q_f32(scale + i);

    float32x4_t vec_elements = vmulq_f32(vcvtq_f32_s32(vec_input), vec_scale);
    float32x4_t temp_vec_res_value = vmaxq_f32(vec_elements, vec_res_value);
    vec_res_idx =
        CalculateIndex(i, temp_vec_res_value, vec_res_value, vec_res_idx);
    vec_res_value = temp_vec_res_value;
  }

  uint32_t idx = CalculateVectorIndex(vec_res_idx, vec_res_value);
  float res = vget_lane_f32(CalculateMax(vec_res_value), 0);

  // Compute left elements
  for (; i < length; ++i) {
    float score = input[i] * scale[i];
    if (score > res) {
      idx = i;
      res = score;
    }
  }
  std::pair<float, int> result_id_score = {res, idx};
  return result_id_score;
}

int GetAnchors(std::vector<EDAnchor> &anchors,
                                                 int layer,
                                                 int feat_height,
                                                 int feat_width) {
  int stride = default_efficient_det_config.feature_strides[layer];
  auto scales = default_efficient_det_config.anchor_scales[layer];
  const auto &ratios = default_efficient_det_config.anchor_ratio;
  int w = stride, h = stride;
  int size = w * h;
  float x_ctr = 0.5 * (w - 1.f);
  float y_ctr = 0.5 * (h - 1.f);
  // base anchor
  std::vector<EDBaseAnchor> base_anchors;
  for (const auto &ratio : ratios) {
    for (const auto &scale : scales) {
      double size_ratio = std::floor(size / ratio);
      double new_w = std::floor(std::sqrt(size_ratio) + 0.5) * scale;
      double new_h = std::floor(new_w / scale * ratio + 0.5) * scale;
      base_anchors.push_back(EDBaseAnchor(x_ctr - 0.5f * (new_w - 1.f),
                                          y_ctr - 0.5f * (new_h - 1.f),
                                          x_ctr + 0.5f * (new_w - 1.f),
                                          y_ctr + 0.5f * (new_h - 1.f)));
    }
  }

  for (int i = 0; i < feat_height; ++i) {
    for (int j = 0; j < feat_width; ++j) {
      auto ori_y = i * stride;
      auto ori_x = j * stride;
      for (const auto &base_anchor : base_anchors) {
        float x1 = base_anchor.x1_ + ori_x;
        float y1 = base_anchor.y1_ + ori_y;
        float x2 = base_anchor.x2_ + ori_x;
        float y2 = base_anchor.y2_ + ori_y;
        float width = x2 - x1 + 1.f;
        float height = y2 - y1 + 1.f;
        float ctr_x = x1 + 0.5f * (width - 1.f);
        float ctr_y = y1 + 0.5f * (height - 1.f);

        anchors.push_back(EDAnchor(ctr_x, ctr_y, width, height));
      }
    }
  }
  return 0;
}

int GetBboxAndScores(
    hbDNNTensor *c_tensor,
    hbDNNTensor *bbox_tensor,
    std::vector<EDAnchor> &anchors,
    int class_num,
    float img_h,
    float img_w,
    EfficientdetPostProcessInfo_t *post_info) {
  auto *shape = c_tensor->properties.validShape.dimensionSize;
  //int32_t c_batch_size = shape[0];

  //int32_t c_hnum = shape[1];
  //int32_t c_wnum = shape[2];
  int32_t c_cnum = shape[3];
  uint32_t anchor_num_per_pixel = c_cnum / class_num;

  shape = bbox_tensor->properties.validShape.dimensionSize;
  int32_t b_batch_size = shape[0];
  int32_t b_hnum = shape[1];
  int32_t b_wnum = shape[2];
  //int32_t b_cnum = shape[3];

  assert(anchor_num_per_pixel == b_cnum / 4);
  assert(c_batch_size == b_batch_size && c_hnum == b_hnum && c_wnum == b_wnum);
  auto box_num = b_batch_size * b_hnum * b_wnum * anchor_num_per_pixel;

  auto quanti_type = bbox_tensor->properties.quantiType;

  if (quanti_type == hbDNNQuantiType::NONE) {
    auto *raw_cls_data = reinterpret_cast<float *>(c_tensor->sysMem[0].virAddr);
    auto *raw_box_data =
        reinterpret_cast<float *>(bbox_tensor->sysMem[0].virAddr);
    for (int i = 0; i < box_num; i++) {
      uint32_t res_id_cur_anchor = i * class_num;
      int max_id = 0;
      float max_score = raw_cls_data[res_id_cur_anchor];
      if (class_num % 4 != 0) {
        for (int cls = 1; cls < class_num; ++cls) {
          float cls_value = raw_cls_data[res_id_cur_anchor + cls];
          if (cls_value > max_score) {
            max_score = cls_value;
            max_id = cls;
          }
        }
      } else {
        int max_id_0 = 0;
        int max_id_1 = 1;
        int max_id_2 = 2;
        int max_id_3 = 3;
        float max_score_0 = raw_cls_data[res_id_cur_anchor];
        float max_score_1 = raw_cls_data[res_id_cur_anchor + 1];
        float max_score_2 = raw_cls_data[res_id_cur_anchor + 2];
        float max_score_3 = raw_cls_data[res_id_cur_anchor + 3];
        for (int cls = 0; cls < class_num; cls += 4) {
          float cls_value_0 = raw_cls_data[res_id_cur_anchor + cls];
          float cls_value_1 = raw_cls_data[res_id_cur_anchor + cls + 1];
          float cls_value_2 = raw_cls_data[res_id_cur_anchor + cls + 2];
          float cls_value_3 = raw_cls_data[res_id_cur_anchor + cls + 3];
          if (cls_value_0 > max_score_0) {
            max_score_0 = cls_value_0;
            max_id_0 = cls;
          }

          if (cls_value_1 > max_score_1) {
            max_score_1 = cls_value_1;
            max_id_1 = cls + 1;
          }

          if (cls_value_2 > max_score_2) {
            max_score_2 = cls_value_2;
            max_id_2 = cls + 2;
          }

          if (cls_value_3 > max_score_3) {
            max_score_3 = cls_value_3;
            max_id_3 = cls + 3;
          }
        }

        if (max_score_0 > max_score) {
          max_score = max_score_0;
          max_id = max_id_0;
        }

        if (max_score_1 > max_score) {
          max_score = max_score_1;
          max_id = max_id_1;
        }

        if (max_score_2 > max_score) {
          max_score = max_score_2;
          max_id = max_id_2;
        }

        if (max_score_3 > max_score) {
          max_score = max_score_3;
          max_id = max_id_3;
        }
      }
      // box
      if (max_score <= post_info->score_threshold) continue;

      // aligned index
      int start = i * 4;
      float dx = raw_box_data[start];
      float dy = raw_box_data[start + 1];
      float dw = raw_box_data[start + 2];
      float dh = raw_box_data[start + 3];
      float width = anchors[i].w_;
      float height = anchors[i].h_;
      float ctr_x = anchors[i].c_x_;
      float ctr_y = anchors[i].c_y_;

      float pred_ctr_x = dx * width + ctr_x;
      float pred_ctr_y = dy * height + ctr_y;
      float pred_w = std::exp(dw) * width;
      float pred_h = std::exp(dh) * height;

      // python在这里需要对框做clip,  x >= 0 && x <= input_width...
      float xmin = (pred_ctr_x - 0.5f * (pred_w - 1.f));
      float ymin = (pred_ctr_y - 0.5f * (pred_h - 1.f));
      float xmax = (pred_ctr_x + 0.5f * (pred_w - 1.f));
      float ymax = (pred_ctr_y + 0.5f * (pred_h - 1.f));

      Bbox bbox;
      bbox.xmin = std::max(xmin, 0.f);
      bbox.ymin = std::max(ymin, 0.f);
      bbox.xmax = std::min(xmax, img_w);
      bbox.ymax = std::min(ymax, img_h);

      efficient_det_dets.push_back(Detection(max_id,
                                              max_score,
                                              bbox,
                                              default_efficient_det_config.class_names[max_id].c_str()));
    }
  } else {
    auto *raw_cls_data =
        reinterpret_cast<int32_t *>(c_tensor->sysMem[0].virAddr);
    auto *raw_box_data =
        reinterpret_cast<int32_t *>(bbox_tensor->sysMem[0].virAddr);
    for (int i = 0; i < box_num; i++) {
      // score and cls
      uint32_t res_id_cur_anchor = i * class_num;
      int cls_scale_index = res_id_cur_anchor % 720;
      // cls scales for every box
      float *cls_scale_data = c_tensor->properties.scale.scaleData;
      auto cls_scales = cls_scale_data + cls_scale_index;
      // 获取max_id and max_score;
      auto max_score_id =
          MaxScoreID(raw_cls_data + res_id_cur_anchor, cls_scales, class_num);
      float max_score = max_score_id.first;
      int max_id = max_score_id.second;

      // box
      if (max_score <= post_info->score_threshold) continue;

      // stride is 40, if do not have dequanti node
      int start = i * 4 + (4 * i) / 36 * 4;
      int scale_index = (i * 4) % 36;
      // box scales for every box
      float *box_scale_data = bbox_tensor->properties.scale.scaleData;
      auto box_scales = box_scale_data + scale_index;
      float dx = raw_box_data[start] * box_scales[0];
      float dy = raw_box_data[start + 1] * box_scales[1];
      float dw = raw_box_data[start + 2] * box_scales[2];
      float dh = raw_box_data[start + 3] * box_scales[3];
      float width = anchors[i].w_;
      float height = anchors[i].h_;
      float ctr_x = anchors[i].c_x_;
      float ctr_y = anchors[i].c_y_;

      float pred_ctr_x = dx * width + ctr_x;
      float pred_ctr_y = dy * height + ctr_y;
      float pred_w = std::exp(dw) * width;
      float pred_h = std::exp(dh) * height;

      // python在这里需要对框做clip,  x >= 0 && x <= input_width...
      float xmin = (pred_ctr_x - 0.5f * (pred_w - 1.f));
      float ymin = (pred_ctr_y - 0.5f * (pred_h - 1.f));
      float xmax = (pred_ctr_x + 0.5f * (pred_w - 1.f));
      float ymax = (pred_ctr_y + 0.5f * (pred_h - 1.f));

      Bbox bbox;
      bbox.xmin = std::max(xmin, 0.f);
      bbox.ymin = std::max(ymin, 0.f);
      bbox.xmax = std::min(xmax, img_w);
      bbox.ymax = std::min(ymax, img_h);

      efficient_det_dets.push_back(Detection(max_id,
                                    max_score,
                                    bbox,
                                    default_efficient_det_config.class_names[max_id].c_str()));
    }
  }
  return 0;
}

void EfficientdetdoProcess(hbDNNTensor *cls_tensor, hbDNNTensor *bbox_tensor, EfficientdetPostProcessInfo_t *post_info, int layer) {

  float origin_height = post_info->ori_height;
  float origin_width = post_info->ori_width;

  auto input_height = post_info->height;
  auto input_width = post_info->width;

  float w_ratio = input_width * 1.f / origin_width;
  float h_ratio = input_height * 1.f / origin_height;
  float scale = std::min(w_ratio, h_ratio);
  float new_h = input_height;
  float new_w = input_width;
  if (post_info->is_pad_resize) {
    new_h = scale * origin_height;
    new_w = scale * origin_width;
    new_h = std::ceil(new_h / 128.f) * 128.f;
    new_w = std::ceil(new_w / 128.f) * 128.f;
    w_ratio = scale;
    h_ratio = scale;
  }


  int height = bbox_tensor->properties.alignedShape.dimensionSize[1];
  int width = bbox_tensor->properties.alignedShape.dimensionSize[2];
  int layer_num = default_efficient_det_config.feature_strides.size();
  if (anchors_table.empty()) {
    // Note: note thread safe
    anchors_table.resize(layer_num);
    printf("init anchors_table.\n");
  }
  GetAnchors(anchors_table[layer], layer, height, width);

  std::vector<EDAnchor> &anchors = anchors_table[layer];
  GetBboxAndScores(cls_tensor, bbox_tensor, anchors, kEfficientDetClassNum, new_h, new_w, post_info);

}


char* EfficientdetPostProcess(EfficientdetPostProcessInfo_t *post_info) {

  int i = 0;
  char *str_dets;
  std::stringstream out_string;

  float origin_height = post_info->ori_height;
  float origin_width = post_info->ori_width;

  auto input_height = post_info->height;
  auto input_width = post_info->width;

  float w_ratio = input_width * 1.f / origin_width;
  float h_ratio = input_height * 1.f / origin_height;
  float scale = std::min(w_ratio, h_ratio);
  float new_h = input_height;
  float new_w = input_width;
  if (post_info->is_pad_resize) {
    new_h = scale * origin_height;
    new_w = scale * origin_width;
    new_h = std::ceil(new_h / 128.f) * 128.f;
    new_w = std::ceil(new_w / 128.f) * 128.f;
    w_ratio = scale;
    h_ratio = scale;
  }

  efficient_det_nms(efficient_det_dets, post_info->nms_threshold, post_info->nms_top_k, efficient_det_restuls, false);

  if (efficient_det_restuls.size() > post_info->nms_top_k) {
    efficient_det_restuls.resize(post_info->nms_top_k);
  }

  for (auto &box : efficient_det_restuls) {
    box.bbox.xmax /= w_ratio;
    box.bbox.xmin /= w_ratio;
    box.bbox.ymax /= h_ratio;
    box.bbox.ymin /= h_ratio;

    //    box.bbox.xmin = std::max(box.bbox.xmin, 0.f);
    //    box.bbox.ymin = std::max(box.bbox.ymin, 0.f);
    box.bbox.xmax = std::min(static_cast<float>(box.bbox.xmax), static_cast<float>(post_info->ori_width - 1)) + 1;
    box.bbox.ymax = std::min(static_cast<float>(box.bbox.ymax), static_cast<float>(post_info->ori_height - 1)) + 1;
  }

  // 算法结果转换成json格式
  out_string << "\"efficient_det_result\": [";
  for (i = 0; i < efficient_det_restuls.size(); i++) {
  	auto det_ret = efficient_det_restuls[i];
  	out_string << det_ret;
  	if (i < efficient_det_restuls.size() - 1)
		out_string << ",";
  }
  out_string << "]" << std::endl;

  str_dets = (char *)malloc(out_string.str().length() + 1);
  str_dets[out_string.str().length()] = '\0';
  snprintf(str_dets, out_string.str().length(), "%s", out_string.str().c_str());
  // printf("str_dets: %s\n", str_dets);
  efficient_det_dets.clear();
  efficient_det_restuls.clear();
  return str_dets;
}

