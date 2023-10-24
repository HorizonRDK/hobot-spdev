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

#include "ptq_ssd_post_process.h"

inline float fastExp(float x) {
  union {
    uint32_t i;
    float f;
  } v;
  v.i = (12102203.1616540672f * x + 1064807160.56887296f);
  return v.f;
}

#define BSWAP_32(x) static_cast<int32_t>(__builtin_bswap32(x))

#define r_int32(x, big_endian) \
  (big_endian) ? BSWAP_32((x)) : static_cast<int32_t>((x))

/**
 * Config definition for SSD
 */
struct SSDConfig {
  std::vector<float> std;
  std::vector<float> mean;
  std::vector<float> offset;
  std::vector<int> step;
  std::vector<std::pair<float, float>> anchor_size;
  std::vector<std::vector<float>> anchor_ratio;
  int background_index;
  int class_num;
  std::vector<std::string> class_names;
};

/**
 * Default ssd config
 * std: [0.1, 0.1, 0.2, 0.2]
 * mean: [0, 0, 0, 0]
 * offset: [0.5, 0.5]
 * step: [15, 30, 60, 100, 150, 300]
 * anchor_size: [[60, -1], [105, 150], [150, 195],
 *              [195, 240], [240, 285], [285,300]]
 * anchor_ratio: [[2, 0.5, 0, 0], [2, 0.5, 3, 1.0 / 3],
 *              [2, 0.5, 3, 1.0 / 3], [2, 0.5, 3, 1.0 / 3],
 *              [2, 0.5, 1.0 / 3], [2, 0.5, 1.0 / 3]]
 * background_index 0
 * class_num: 20
 * class_names: ["aeroplane",   "bicycle", "bird",  "boaupdate", "bottle",
     "bus",         "car",     "cat",   "chair",     "cow",
     "diningtable", "dog",     "horse", "motorbike", "person",
     "pottedplant", "sheep",   "sofa",  "train",     "tvmonitor"]
 */
SSDConfig default_ssd_config = {
    {0.1, 0.1, 0.2, 0.2},
    {0, 0, 0, 0},
    {0.5, 0.5},
    {15, 30, 60, 100, 150, 300},
    {{60, -1}, {105, 150}, {150, 195}, {195, 240}, {240, 285}, {285, 300}},
    {{2, 0.5, 0, 0},
     {2, 0.5, 3, 1.0 / 3},
     {2, 0.5, 3, 1.0 / 3},
     {2, 0.5, 3, 1.0 / 3},
     {2, 0.5, 3, 1.0 / 3},
     {2, 0.5, 3, 1.0 / 3}},
    0,
    20,
    {"aeroplane",   "bicycle", "bird",  "boaupdate", "bottle",
     "bus",         "car",     "cat",   "chair",     "cow",
     "diningtable", "dog",     "horse", "motorbike", "person",
     "pottedplant", "sheep",   "sofa",  "train",     "tvmonitor"}};

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


typedef struct Anchor {
  float cx{0.0};
  float cy{0.0};
  float w{0.0};
  float h{0.0};
  Anchor(float cx, float cy, float w, float h) : cx(cx), cy(cy), w(w), h(h) {}

  friend std::ostream &operator<<(std::ostream &os, const Anchor &anchor) {
    os << "[" << anchor.cx << "," << anchor.cy << "," << anchor.w << ","
       << anchor.h << "]";
    return os;
  }
} Anchor;

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
       << ":\"" << default_ssd_config.class_names[det.id] << "\"}";
    return os; 
  }
 
  ~Detection() {}
} Detection;

static float DequantiScale(int32_t data,
                                             bool big_endian,
                                             float &scale_value) {
  return static_cast<float>(r_int32(data, big_endian)) * scale_value;
}

std::vector<Detection> ssd_dets;
std::vector<Detection> ssd_det_restuls;
std::vector<std::vector<Anchor>> anchors_table;

#define NMS_MAX_INPUT (400)
void ssd_nms(std::vector<Detection> &input,
         float iou_threshold,
         int top_k,
         std::vector<Detection> &result,
         bool suppress) {
  // sort order by score desc
  std::stable_sort(input.begin(), input.end(), std::greater<Detection>());
  if (input.size() > NMS_MAX_INPUT) {
    input.resize(NMS_MAX_INPUT);
  }

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

int SsdAnchors(std::vector<Anchor> &anchors, int layer, int layer_height, int layer_width) {
  int step = default_ssd_config.step[layer];
  float min_size = default_ssd_config.anchor_size[layer].first;
  float max_size = default_ssd_config.anchor_size[layer].second;
  auto &anchor_ratio = default_ssd_config.anchor_ratio[layer];
  for (int i = 0; i < layer_height; i++) {
    for (int j = 0; j < layer_width; j++) {
      float cy = (i + default_ssd_config.offset[0]) * step;
      float cx = (j + default_ssd_config.offset[1]) * step;
      anchors.emplace_back(Anchor(cx, cy, min_size, min_size));
      if (max_size > 0) {
        anchors.emplace_back(Anchor(cx,
                                    cy,
                                    std::sqrt(max_size * min_size),
                                    std::sqrt(max_size * min_size)));
      }
      for (int k = 0; k < 4; k++) {
        if (anchor_ratio[k] == 0) continue;
        float sr = std::sqrt(anchor_ratio[k]);
        float w = min_size * sr;
        float h = min_size / sr;
        anchors.emplace_back(Anchor(cx, cy, w, h));
      }
    }
  }
  return 0;
}

int GetBboxAndScoresQuantiNONE(
    hbDNNTensor *bbox_tensor,
    hbDNNTensor *cls_tensor,
    std::vector<Anchor> &anchors,
    int class_num, SsdPostProcessInfo_t *post_info) {
  int *shape = cls_tensor->properties.validShape.dimensionSize;
  //uint32_t c_batch_size = shape[0];

  //uint32_t c_hnum = shape[1];
  //uint32_t c_wnum = shape[2];
  uint32_t c_cnum = shape[3];
  uint32_t anchor_num_per_pixel = c_cnum / class_num;

  shape = bbox_tensor->properties.validShape.dimensionSize;
  int32_t b_batch_size = shape[0];

  uint32_t b_hnum = shape[1];
  uint32_t b_wnum = shape[2];
  //uint32_t b_cnum = shape[3];

  assert(anchor_num_per_pixel == b_cnum / 4);
  assert(c_batch_size == b_batch_size && c_hnum == b_hnum && c_wnum == b_wnum);
  auto box_num = b_batch_size * b_hnum * b_wnum * anchor_num_per_pixel;

  auto *raw_cls_data = reinterpret_cast<float *>(cls_tensor->sysMem[0].virAddr);

  auto *raw_box_data =
      reinterpret_cast<float *>(bbox_tensor->sysMem[0].virAddr);

  for (int i = 0; i < box_num; i++) {
    uint32_t res_id_cur_anchor = i * class_num;
    // get softmax sum
    double sum = 0;
    int max_id = 0;
    // TODO(@horizon.ai): fastExp only affect the final score value
    // confirm whether it affects the accuracy
    bool is_performance_ = true;
    double background_score;
    if (is_performance_) {
      background_score = fastExp(
          raw_cls_data[res_id_cur_anchor + default_ssd_config.background_index]);
    } else {
      background_score = std::exp(
          raw_cls_data[res_id_cur_anchor + default_ssd_config.background_index]);
    }

    double max_score = 0;
    for (int cls = 0; cls < class_num; ++cls) {
      float cls_score;
      if (is_performance_) {
        cls_score = fastExp(raw_cls_data[res_id_cur_anchor + cls]);
      } else {
        cls_score = std::exp(raw_cls_data[res_id_cur_anchor + cls]);
      }
      /* 1. scores should be larger than background score, or else will not be
      selected
      2. For Location for class_name, to add background to class-names list when
      background is not lastest
      3. For ssd_mobilenetv1_300x300_nv12.bin, background_index is 0, the value
      can be modified according to different model*/
      if (cls != default_ssd_config.background_index && cls_score > max_score &&
          cls_score > background_score) {
        max_id = cls - 1;
        max_score = cls_score;
      }
      sum += cls_score;
    }
    // get softmax score
    max_score = max_score / sum;

    if (max_score <= post_info->score_threshold) {
      continue;
    }

    int start = i * 4;
    float dx = raw_box_data[start];
    float dy = raw_box_data[start + 1];
    float dw = raw_box_data[start + 2];
    float dh = raw_box_data[start + 3];

    auto x_min = (anchors[i].cx - anchors[i].w / 2) / post_info->width;
    auto y_min = (anchors[i].cy - anchors[i].h / 2) / post_info->height;
    auto x_max = (anchors[i].cx + anchors[i].w / 2) / post_info->width;
    auto y_max = (anchors[i].cy + anchors[i].h / 2) / post_info->height;

    auto prior_w = x_max - x_min;
    auto prior_h = y_max - y_min;
    auto prior_center_x = (x_max + x_min) / 2;
    auto prior_center_y = (y_max + y_min) / 2;
    auto decode_x = default_ssd_config.std[0] * dx * prior_w + prior_center_x;
    auto decode_y = default_ssd_config.std[1] * dy * prior_h + prior_center_y;
    auto decode_w = std::exp(default_ssd_config.std[2] * dw) * prior_w;
    auto decode_h = std::exp(default_ssd_config.std[3] * dh) * prior_h;

    auto xmin_org = (decode_x - decode_w * 0.5) * post_info->ori_width;
    auto ymin_org = (decode_y - decode_h * 0.5) * post_info->ori_height;
    auto xmax_org = (decode_x + decode_w * 0.5) * post_info->ori_width;
    auto ymax_org = (decode_y + decode_h * 0.5) * post_info->ori_height;

    xmin_org = std::max(xmin_org, 0.0);
    xmax_org = std::min(xmax_org, post_info->ori_width - 1.0);
    ymin_org = std::max(ymin_org, 0.0);
    ymax_org = std::min(ymax_org, post_info->ori_height - 1.0);

    if (xmax_org <= 0 || ymax_org <= 0) continue;
    if (xmin_org > xmax_org || ymin_org > ymax_org) continue;

    Bbox bbox(xmin_org, ymin_org, xmax_org, ymax_org);
    ssd_dets.emplace_back(Detection(
        (int)max_id, max_score, bbox, default_ssd_config.class_names[max_id].c_str()));
  }
  return 0;
}

int GetBboxAndScoresQuantiSCALE(
    hbDNNTensor *bbox_tensor,
    hbDNNTensor *cls_tensor,
    std::vector<Anchor> &anchors,
    int class_num, SsdPostProcessInfo_t *post_info) {
  int h_idx{1}, w_idx{2}, c_idx{3};

  // bbox
  int32_t bbox_n = bbox_tensor->properties.validShape.dimensionSize[0];
  int32_t bbox_h = bbox_tensor->properties.validShape.dimensionSize[h_idx];
  int32_t bbox_w = bbox_tensor->properties.validShape.dimensionSize[w_idx];
  int32_t bbox_c_valid =
      bbox_tensor->properties.validShape.dimensionSize[c_idx];
  int32_t bbox_c_aligned =
      bbox_tensor->properties.alignedShape.dimensionSize[c_idx];

  int32_t *bbox_data =
      reinterpret_cast<int32_t *>(bbox_tensor->sysMem[0].virAddr);
  float *bbox_scale = bbox_tensor->properties.scale.scaleData;

  // cls shape
  //int32_t cls_n = cls_tensor->properties.validShape.dimensionSize[0];
  //int32_t cls_h = cls_tensor->properties.validShape.dimensionSize[h_idx];
  //int32_t cls_w = cls_tensor->properties.validShape.dimensionSize[w_idx];
  int32_t cls_c_valid = cls_tensor->properties.validShape.dimensionSize[c_idx];
  int32_t cls_c_aligned =
      cls_tensor->properties.alignedShape.dimensionSize[c_idx];

  int32_t *cls_data =
      reinterpret_cast<int32_t *>(cls_tensor->sysMem[0].virAddr);
  float *cls_scale = cls_tensor->properties.scale.scaleData;

  auto stride = cls_c_valid / class_num;
  auto bbox_num_pred = bbox_c_valid / stride;

  bool is_performance_ = true;

  for (int h = 0; h < bbox_h; ++h) {
    for (int w = 0; w < bbox_w; ++w) {
      for (int k = 0; k < stride; ++k) {
        int32_t *cur_cls_data = cls_data + k * class_num;
        float *cur_cls_scale = cls_scale + k * class_num;
        float tmp = DequantiScale(cur_cls_data[0], false, cur_cls_scale[0]);

        double background_score =
            is_performance_ ? fastExp(tmp) : std::exp(tmp);
        double sum = 0;
        int max_id = 0;
        double max_score = 0;
        for (int index = 0; index < class_num; ++index) {
          tmp = DequantiScale(cur_cls_data[index], false, cur_cls_scale[index]);

          float cls_score = is_performance_ ? fastExp(tmp) : std::exp(tmp);

          sum += cls_score;
          if (index != 0 && cls_score > max_score &&
              cls_score > background_score) {
            max_id = index - 1;
            max_score = cls_score;
          }
        }
        max_score = max_score / sum;

        if (max_score <= post_info->score_threshold) {
          continue;
        }

        int32_t *cur_bbox_data = bbox_data + k * bbox_num_pred;
        float *cur_bbox_scale = bbox_scale + k * bbox_num_pred;
        float dx = DequantiScale(cur_bbox_data[0], false, cur_bbox_scale[0]);
        float dy = DequantiScale(cur_bbox_data[1], false, cur_bbox_scale[1]);
        float dw = DequantiScale(cur_bbox_data[2], false, cur_bbox_scale[2]);
        float dh = DequantiScale(cur_bbox_data[3], false, cur_bbox_scale[3]);

        int i = h * bbox_w * stride + w * stride + k;
        auto x_min = (anchors[i].cx - anchors[i].w / 2) / post_info->width;
        auto y_min = (anchors[i].cy - anchors[i].h / 2) / post_info->height;
        auto x_max = (anchors[i].cx + anchors[i].w / 2) / post_info->width;
        auto y_max = (anchors[i].cy + anchors[i].h / 2) / post_info->height;

        auto prior_w = x_max - x_min;
        auto prior_h = y_max - y_min;
        auto prior_center_x = (x_max + x_min) / 2;
        auto prior_center_y = (y_max + y_min) / 2;
        auto decode_x = default_ssd_config.std[0] * dx * prior_w + prior_center_x;
        auto decode_y = default_ssd_config.std[1] * dy * prior_h + prior_center_y;
        auto decode_w = std::exp(default_ssd_config.std[2] * dw) * prior_w;
        auto decode_h = std::exp(default_ssd_config.std[3] * dh) * prior_h;

        auto xmin_org = (decode_x - decode_w * 0.5) * post_info->ori_width;
        auto ymin_org = (decode_y - decode_h * 0.5) * post_info->ori_height;
        auto xmax_org = (decode_x + decode_w * 0.5) * post_info->ori_width;
        auto ymax_org = (decode_y + decode_h * 0.5) * post_info->ori_height;

        xmin_org = std::max(xmin_org, 0.0);
        xmax_org = std::min(xmax_org, post_info->ori_width - 1.0);
        ymin_org = std::max(ymin_org, 0.0);
        ymax_org = std::min(ymax_org, post_info->ori_height - 1.0);

        if (xmax_org <= 0 || ymax_org <= 0) continue;
        if (xmin_org > xmax_org || ymin_org > ymax_org) continue;

        Bbox bbox(xmin_org, ymin_org, xmax_org, ymax_org);
        ssd_dets.emplace_back(Detection((int)max_id,
                                    max_score,
                                    bbox,
                                    default_ssd_config.class_names[max_id].c_str()));
      }
      bbox_data = bbox_data + bbox_c_aligned;
      cls_data = cls_data + cls_c_aligned;
    }
  }
  return 0;
}


void SsddoProcess(hbDNNTensor *bbox_tensor, hbDNNTensor *cls_tensor, SsdPostProcessInfo_t *post_info, int layer) {

  int height = bbox_tensor->properties.alignedShape.dimensionSize[1];
  int width = bbox_tensor->properties.alignedShape.dimensionSize[2];
  int layer_num = default_ssd_config.step.size();
  if (anchors_table.empty()) {
    // Note: note thread safe
    anchors_table.resize(layer_num);
    printf("init anchors_table.\n");
  }
  SsdAnchors(anchors_table[layer], layer, height, width);

  auto quanti_type = bbox_tensor->properties.quantiType;
  if (quanti_type == hbDNNQuantiType::SCALE) {
    GetBboxAndScoresQuantiSCALE(bbox_tensor, cls_tensor, anchors_table[layer], default_ssd_config.class_num + 1, post_info);
  } else if (quanti_type == hbDNNQuantiType::NONE) {
    GetBboxAndScoresQuantiNONE(bbox_tensor, cls_tensor, anchors_table[layer], default_ssd_config.class_num + 1, post_info);
  } else {
    printf("error quanti_type: %d\n", quanti_type);
  }
}


char* SsdPostProcess(SsdPostProcessInfo_t *post_info) {

  int i = 0;
  char *str_dets;

  // 计算交并比来合并检测框，传入交并比阈值(0.65)和返回box数量(5000)
  ssd_nms(ssd_dets, post_info->nms_threshold, post_info->nms_top_k, ssd_det_restuls, false);
  std::stringstream out_string;

  // 算法结果转换成json格式
  int ssd_det_restuls_size = ssd_det_restuls.size();
  out_string << "\"ssd_result\": [";
  for (i = 0; i < ssd_det_restuls_size; i++) {
  	auto det_ret = ssd_det_restuls[i];
  	out_string << det_ret;
  	if (i < ssd_det_restuls_size - 1)
		out_string << ",";
  }
  out_string << "]" << std::endl;

  str_dets = (char *)malloc(out_string.str().length() + 1);
  str_dets[out_string.str().length()] = '\0';
  snprintf(str_dets, out_string.str().length(), "%s", out_string.str().c_str());
  // printf("str_dets: %s\n", str_dets);
  ssd_dets.clear();
  ssd_det_restuls.clear();
  return str_dets;
}

