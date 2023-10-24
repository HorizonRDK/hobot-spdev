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

#include "yolov3_post_process.h"

/**
 * Config definition for Yolov3
 */
struct Yolov3Config {
  std::vector<int> strides;
  std::vector<std::vector<std::pair<double, double>>> anchors_table;
  int class_num;
  std::vector<std::string> class_names;
};


Yolov3Config default_yolov3_config = {
    {32, 16, 8},
    {{{3.625, 2.8125}, {4.875, 6.1875}, {11.65625, 10.1875}},
     {{1.875, 3.8125}, {3.875, 2.8125}, {3.6875, 7.4375}},
     {{1.25, 1.625}, {2.0, 3.75}, {4.125, 2.875}}},
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
       << ":\"" << default_yolov3_config.class_names[det.id] << "\"}";
    return os; 
  }
 
  ~Detection() {}
} Detection;

std::vector<Detection> yolov3_dets;

#define NMS_MAX_INPUT (400)

static void yolov3_nms(std::vector<Detection> &input,
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

float DequantiScale(int32_t data,
                                                      bool big_endian,
                                                      float &scale_value) {
  return static_cast<float>(r_int32(data, big_endian)) * scale_value;
}

void PostProcessQuantiScaleNHWC(
    hbDNNTensor *tensor,
    Yolov3PostProcessInfo_t *post_info,
    int layer) {
  auto *data = reinterpret_cast<int32_t *>(tensor->sysMem[0].virAddr);
  float *scale = tensor->properties.scale.scaleData;
  int num_classes = default_yolov3_config.class_num;
  int stride = default_yolov3_config.strides[layer];
  int num_pred = default_yolov3_config.class_num + 4 + 1;

  std::vector<float> class_pred(default_yolov3_config.class_num, 0.0);
  std::vector<std::pair<double, double>> &anchors =
      default_yolov3_config.anchors_table[layer];

  double h_ratio = post_info->height * 1.0 / post_info->ori_height;
  double w_ratio = post_info->width * 1.0 / post_info->ori_width;
  double resize_ratio = std::min(w_ratio, h_ratio);
  if (post_info->is_pad_resize) {
    w_ratio = resize_ratio;
    h_ratio = resize_ratio;
  }

  int height = tensor->properties.validShape.dimensionSize[1];
  int width = tensor->properties.validShape.dimensionSize[2];

  int channel_valid = tensor->properties.validShape.dimensionSize[3];
  int channel_aligned = tensor->properties.alignedShape.dimensionSize[3];
  printf("channel_valid: %d\n", channel_valid);
  printf("channel_aligned: %d\n", channel_aligned);

  int anchors_size = anchors.size();
  for (int32_t h = 0; h < height; h++) {
    for (int32_t w = 0; w < width; w++) {
      for (int k = 0; k < anchors_size; k++) {
        double anchor_x = anchors[k].first;
        double anchor_y = anchors[k].second;

        int32_t *cur_data = data + k * num_pred;
        float *cur_scale = scale + k * num_pred;
        float objness = DequantiScale(cur_data[4], false, cur_scale[4]);

        for (int index = 0; index < num_classes; ++index) {
          class_pred[index] =
              DequantiScale(cur_data[5 + index], false, cur_scale[5 + index]);
        }

        float id = argmax(class_pred.begin(), class_pred.end());
        double x1 = 1 / (1 + std::exp(-objness)) * 1;
        double x2 = 1 / (1 + std::exp(-class_pred[id]));
        double confidence = x1 * x2;

        if (confidence < post_info->score_threshold) {
          continue;
        }

        float center_x = DequantiScale(cur_data[0], false, cur_scale[0]);
        float center_y = DequantiScale(cur_data[1], false, cur_scale[1]);
        float scale_x = DequantiScale(cur_data[2], false, cur_scale[2]);
        float scale_y = DequantiScale(cur_data[3], false, cur_scale[3]);

        double box_center_x =
            ((1.0 / (1.0 + std::exp(-center_x))) + w) * stride;
        double box_center_y =
            ((1.0 / (1.0 + std::exp(-center_y))) + h) * stride;

        double box_scale_x = std::exp(scale_x) * anchor_x * stride;
        double box_scale_y = std::exp(scale_y) * anchor_y * stride;

        double xmin = (box_center_x - box_scale_x / 2.0);
        double ymin = (box_center_y - box_scale_y / 2.0);
        double xmax = (box_center_x + box_scale_x / 2.0);
        double ymax = (box_center_y + box_scale_y / 2.0);

        double w_padding =
            (post_info->width - w_ratio * post_info->ori_width) / 2.0;
        double h_padding =
            (post_info->height - h_ratio * post_info->ori_height) / 2.0;

        double xmin_org = (xmin - w_padding) / w_ratio;
        double xmax_org = (xmax - w_padding) / w_ratio;
        double ymin_org = (ymin - h_padding) / h_ratio;
        double ymax_org = (ymax - h_padding) / h_ratio;

        if (xmin_org > xmax_org || ymin_org > ymax_org) {
          continue;
        }

        xmin_org = std::max(xmin_org, 0.0);
        xmax_org = std::min(xmax_org, post_info->ori_width - 1.0);
        ymin_org = std::max(ymin_org, 0.0);
        ymax_org = std::min(ymax_org, post_info->ori_height - 1.0);

        Bbox bbox(xmin_org, ymin_org, xmax_org, ymax_org);
        yolov3_dets.push_back(Detection((int)id,
                                 confidence,
                                 bbox,
                                 default_yolov3_config.class_names[(int)id].c_str()));
      }
      data = data + channel_aligned;
    }
  }
}

void PostProcessQuantiNoneNHWC(
    hbDNNTensor *tensor,
    Yolov3PostProcessInfo_t *post_info,
    int layer) {
  auto *data = reinterpret_cast<float *>(tensor->sysMem[0].virAddr);
  int num_classes = default_yolov3_config.class_num;
  int stride = default_yolov3_config.strides[layer];
  int num_pred = default_yolov3_config.class_num + 4 + 1;

  std::vector<float> class_pred(default_yolov3_config.class_num, 0.0);
  std::vector<std::pair<double, double>> &anchors =
      default_yolov3_config.anchors_table[layer];

  double h_ratio = post_info->height * 1.0 / post_info->ori_height;
  double w_ratio = post_info->width * 1.0 / post_info->ori_width;
  double resize_ratio = std::min(w_ratio, h_ratio);
  if (post_info->is_pad_resize) {
    w_ratio = resize_ratio;
    h_ratio = resize_ratio;
  }

  int height = tensor->properties.validShape.dimensionSize[1];
  int width = tensor->properties.validShape.dimensionSize[2];

  int anchors_size = anchors.size();
  for (int32_t h = 0; h < height; h++) {
    for (int32_t w = 0; w < width; w++) {
      for (int k = 0; k < anchors_size; k++) {
        double anchor_x = anchors[k].first;
        double anchor_y = anchors[k].second;
        float *cur_data = data + k * num_pred;
        float objness = cur_data[4];
        for (int index = 0; index < num_classes; ++index) {
          class_pred[index] = cur_data[5 + index];
        }

        float id = argmax(class_pred.begin(), class_pred.end());
        double x1 = 1 / (1 + std::exp(-objness)) * 1;
        double x2 = 1 / (1 + std::exp(-class_pred[id]));
        double confidence = x1 * x2;

        if (confidence < post_info->score_threshold) {
          continue;
        }

        float center_x = cur_data[0];
        float center_y = cur_data[1];
        float scale_x = cur_data[2];
        float scale_y = cur_data[3];

        double box_center_x =
            ((1.0 / (1.0 + std::exp(-center_x))) + w) * stride;
        double box_center_y =
            ((1.0 / (1.0 + std::exp(-center_y))) + h) * stride;

        double box_scale_x = std::exp(scale_x) * anchor_x * stride;
        double box_scale_y = std::exp(scale_y) * anchor_y * stride;

        double xmin = (box_center_x - box_scale_x / 2.0);
        double ymin = (box_center_y - box_scale_y / 2.0);
        double xmax = (box_center_x + box_scale_x / 2.0);
        double ymax = (box_center_y + box_scale_y / 2.0);

        double w_padding =
            (post_info->width - w_ratio * post_info->ori_width) / 2.0;
        double h_padding =
            (post_info->height - h_ratio * post_info->ori_height) / 2.0;

        double xmin_org = (xmin - w_padding) / w_ratio;
        double xmax_org = (xmax - w_padding) / w_ratio;
        double ymin_org = (ymin - h_padding) / h_ratio;
        double ymax_org = (ymax - h_padding) / h_ratio;

        if (xmin_org > xmax_org || ymin_org > ymax_org) {
          continue;
        }

        xmin_org = std::max(xmin_org, 0.0);
        xmax_org = std::min(xmax_org, post_info->ori_width - 1.0);
        ymin_org = std::max(ymin_org, 0.0);
        ymax_org = std::min(ymax_org, post_info->ori_height - 1.0);

        Bbox bbox(xmin_org, ymin_org, xmax_org, ymax_org);
        yolov3_dets.push_back(Detection((int)id,
                                 confidence,
                                 bbox,
                                 default_yolov3_config.class_names[(int)id].c_str()));
      }
      data = data + num_pred * anchors.size();
    }
  }
}

void PostProcessNCHW(
    hbDNNTensor *tensor,
    Yolov3PostProcessInfo_t *post_info,
    int layer) {
  auto *data = reinterpret_cast<float *>(tensor->sysMem[0].virAddr);
  int num_classes = default_yolov3_config.class_num;
  int stride = default_yolov3_config.strides[layer];
  int num_pred = default_yolov3_config.class_num + 4 + 1;

  std::vector<float> class_pred(default_yolov3_config.class_num, 0.0);
  std::vector<std::pair<double, double>> &anchors =
      default_yolov3_config.anchors_table[layer];

  double h_ratio = post_info->height * 1.0 / post_info->ori_height;
  double w_ratio = post_info->width * 1.0 / post_info->ori_width;
  double resize_ratio = std::min(w_ratio, h_ratio);
  if (post_info->is_pad_resize) {
    w_ratio = resize_ratio;
    h_ratio = resize_ratio;
  }

  int height = tensor->properties.validShape.dimensionSize[2];
  int width = tensor->properties.validShape.dimensionSize[3];

  int aligned_h = tensor->properties.validShape.dimensionSize[2];
  int aligned_w = tensor->properties.validShape.dimensionSize[3];
  int aligned_hw = aligned_h * aligned_w;

  int anchors_size = anchors.size();
  for (int k = 0; k < anchors_size; k++) {
    for (int32_t h = 0; h < height; h++) {
      for (int32_t w = 0; w < width; w++) {
        double anchor_x = anchors[k].first;
        double anchor_y = anchors[k].second;
        int stride_hw = h * aligned_w + w;

        float objness = data[(k * num_pred + 4) * aligned_hw + stride_hw];
        for (int index = 0; index < num_classes; ++index) {
          class_pred[index] =
              data[(k * num_pred + index + 5) * aligned_hw + stride_hw];
        }

        float id = argmax(class_pred.begin(), class_pred.end());
        double x1 = 1 / (1 + std::exp(-objness)) * 1;
        double x2 = 1 / (1 + std::exp(-class_pred[id]));
        double confidence = x1 * x2;

        if (confidence < post_info->score_threshold) {
          continue;
        }

        float center_x = data[(k * num_pred) * aligned_hw + stride_hw];
        float center_y = data[(k * num_pred + 1) * aligned_hw + stride_hw];
        float scale_x = data[(k * num_pred + 2) * aligned_hw + stride_hw];
        float scale_y = data[(k * num_pred + 3) * aligned_hw + stride_hw];

        double box_center_x =
            ((1.0 / (1.0 + std::exp(-center_x))) + w) * stride;
        double box_center_y =
            ((1.0 / (1.0 + std::exp(-center_y))) + h) * stride;

	double box_scale_x = std::exp(scale_x) * anchor_x * stride;
        double box_scale_y = std::exp(scale_y) * anchor_y * stride;

        double xmin = (box_center_x - box_scale_x / 2.0);
        double ymin = (box_center_y - box_scale_y / 2.0);
        double xmax = (box_center_x + box_scale_x / 2.0);
        double ymax = (box_center_y + box_scale_y / 2.0);

        double w_padding =
            (post_info->width - w_ratio * post_info->ori_width) / 2.0;
        double h_padding =
            (post_info->height - h_ratio * post_info->ori_height) / 2.0;

        double xmin_org = (xmin - w_padding) / w_ratio;
        double xmax_org = (xmax - w_padding) / w_ratio;
        double ymin_org = (ymin - h_padding) / h_ratio;
        double ymax_org = (ymax - h_padding) / h_ratio;

        if (xmin_org > xmax_org || ymin_org > ymax_org) {
          continue;
        }

        xmin_org = std::max(xmin_org, 0.0);
        xmax_org = std::min(xmax_org, post_info->ori_width - 1.0);
        ymin_org = std::max(ymin_org, 0.0);
        ymax_org = std::min(ymax_org, post_info->ori_height - 1.0);

        Bbox bbox(xmin_org, ymin_org, xmax_org, ymax_org);
        yolov3_dets.push_back(Detection((int)id,
                                 confidence,
                                 bbox,
                                 default_yolov3_config.class_names[(int)id].c_str()));
      }
    }
  }
}


void PostProcessNHWC(
    hbDNNTensor *tensor,
    Yolov3PostProcessInfo_t *post_info,
    int layer) {
  auto quanti_type = tensor->properties.quantiType;
  if (quanti_type == hbDNNQuantiType::SCALE) {
    PostProcessQuantiScaleNHWC(tensor, post_info, layer);
  } else if (quanti_type == hbDNNQuantiType::NONE) {
    PostProcessQuantiNoneNHWC(tensor, post_info, layer);
  } else {
    printf("PostProcessNHWC quanti_type : %d\n",  quanti_type);
  }
}


void Yolov3doProcess(hbDNNTensor *tensor, Yolov3PostProcessInfo_t *post_info, int layer) {

  if (tensor->properties.quantizeAxis == 3) {
    PostProcessNHWC(tensor, post_info, layer);
  } else if (tensor->properties.quantizeAxis == 1) {
    PostProcessNCHW(tensor, post_info, layer);
  } else {
    printf("tensor layout error.\n");
  }
}


// Yolov3 输出tensor格式
// 3次下采样得到三组缩小后的gred，然后对每个gred进行三次预测，最后输出结果
char* Yolov3PostProcess(Yolov3PostProcessInfo_t *post_info) {

  int i = 0;
  char *str_yolov3_dets;
  std::vector<Detection> det_restuls;

  // 计算交并比来合并检测框，传入交并比阈值(0.65)和返回box数量(5000)
  yolov3_nms(yolov3_dets, post_info->nms_threshold, post_info->nms_top_k, det_restuls, false);
  std::stringstream out_string;

  // 算法结果转换成json格式
  int det_restuls_size = det_restuls.size();
  out_string << "\"yolov3_result\": [";
  for (i = 0; i < det_restuls_size; i++) {
  	auto det_ret = det_restuls[i];
  	out_string << det_ret;
  	if (i < det_restuls_size - 1)
		out_string << ",";
  }
  out_string << "]" << std::endl;

  str_yolov3_dets = (char *)malloc(out_string.str().length() + 1);
  str_yolov3_dets[out_string.str().length()] = '\0';
  snprintf(str_yolov3_dets, out_string.str().length(), "%s", out_string.str().c_str());
  // printf("str_yolov3_dets: %s\n", str_yolov3_dets);
  yolov3_dets.clear();
  det_restuls.clear();
  return str_yolov3_dets;
}

