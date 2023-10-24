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

// #include "utils/utils_log.h"

#include "yolov5_post_process.h"

#define BSWAP_32(x) static_cast<int32_t>(__builtin_bswap32(x))

#define r_int32(x, big_endian) \
  (big_endian) ? BSWAP_32((x)) : static_cast<int32_t>((x))

/**
 * Config definition for Yolov5
 */
struct Yolov5Config {
  std::vector<int> strides;
  std::vector<std::vector<std::pair<double, double>>> anchors_table;
  int class_num;
  std::vector<std::string> class_names;
};

/**
 * Default yolo config
 * strides: [8, 16, 32]
 * anchors_table: [[[10, 13], [16,30], [33,23]],
 *                  [[30,61], [62,45], [59,119]],
 *                  [[116,90], [156,198], [373,326]]]
 * class_num: 80
 * class_names: ["person",      "bicycle",      "car",
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
     "hair drier",    "toothbrush"]
 */


Yolov5Config default_yolov5_config = {
    {8, 16, 32},
    {{{10, 13}, {16, 30}, {33, 23}},
     {{30, 61}, {62, 45}, {59, 119}},
     {{116, 90}, {156, 198}, {373, 326}}},
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
       << ":\"" << default_yolov5_config.class_names[det.id] << "\"}";
    return os; 
  }
 
  ~Detection() {}
} Detection;

std::vector<Detection> dets;
std::vector<Detection> det_restuls;


static float DequantiScale(int32_t data, bool big_endian, float &scale_value) {
  return static_cast<float>(r_int32(data, big_endian)) * scale_value;
}

static int get_tensor_hw(hbDNNTensor &tensor, int *height, int *width) {
  int h_index = 0;
  int w_index = 0;
  // printf("tensor.properties.tensorLayout: %d\n", tensor.properties.tensorLayout);
  if (tensor.properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
    h_index = 1;
    w_index = 2;
  } else if (tensor.properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    h_index = 2;
    w_index = 3;
  } else {
    return -1;
  }
  *height = tensor.properties.validShape.dimensionSize[h_index];
  *width = tensor.properties.validShape.dimensionSize[w_index];
  return 0;
}

static void yolov5_nms(std::vector<Detection> &input,
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

void Yolov5doProcess(hbDNNTensor *tensor, Yolov5PostProcessInfo_t *post_info, int layer) {

  // 80个分类
  int num_classes = default_yolov5_config.class_num;
  // 下采样值 8 16 32
  int stride = default_yolov5_config.strides[layer];
  // 每个预测值占用多少空间
  // 一组条件类别概率，都是区间在[0,1]之间的值，代表概率。
  // box参数即box的中心点坐标（x,y）和box的宽和高（w,h）
  // 一个是置信度，这是个区间在[0,1]之间的值
  /* 假如一个图片被分割成 width * height 个grid cell，我们有B个anchor box，
   * 也就是说每个grid cell有B个bounding box, 每个bounding box内有4个位置参数，
   * 1个置信度，classes个类别概率，那么最终的输出维数是：width * height * [B*(4 + 1 + classes)]
   */
  int num_pred = default_yolov5_config.class_num + 4 + 1;

  std::vector<float> class_pred(default_yolov5_config.class_num, 0.0);
  // 3组 预设检测框类型
  std::vector<std::pair<double, double>> &anchors = default_yolov5_config.anchors_table[layer];

  // 计算原始图像与算法推理实际使用图像的缩放比
  double h_ratio = post_info->height * 1.0 / post_info->ori_height;
  double w_ratio = post_info->width * 1.0 / post_info->ori_width;
  double resize_ratio = std::min(w_ratio, h_ratio);
  if (post_info->is_pad_resize) {
    w_ratio = resize_ratio;
    h_ratio = resize_ratio;
  }

  // int height, width;
  // auto ret = get_tensor_hw(*tensor, &height, &width);
  // if (ret != 0) {
  //   printf("get_tensor_hw failed\n");
  // }

  int height = tensor->properties.validShape.dimensionSize[1];
  int width = tensor->properties.validShape.dimensionSize[2];

  int anchor_num = anchors.size();
  auto quanti_type = tensor->properties.quantiType;

  if (quanti_type == hbDNNQuantiType::NONE) {
    auto *data = reinterpret_cast<float *>(tensor->sysMem[0].virAddr);
    for (int32_t h = 0; h < height; h++) {
      for (int32_t w = 0; w < width; w++) {
        for (int k = 0; k < anchor_num; k++) {
          double anchor_x = anchors[k].first;
          double anchor_y = anchors[k].second;
          // 取出一个预测结果
          float *cur_data = data + k * num_pred;
          // 置信度
          float objness = cur_data[4];

          // 获得概率值最大的分类对应的编号，作为id
          int id = argmax(cur_data + 5, cur_data + 5 + num_classes);
          // 计算置信度
          double x1 = 1 / (1 + std::exp(-objness)) * 1;
          double x2 = 1 / (1 + std::exp(-cur_data[id + 5]));
          double confidence = x1 * x2;

          // 过滤执行度不足的检测框
          if (confidence < post_info->score_threshold) {
            continue;
          }

          // box参数即box的中心点坐标（x,y）和box的宽和高（w,h）
          float center_x = cur_data[0];
          float center_y = cur_data[1];
          float scale_x = cur_data[2];
          float scale_y = cur_data[3];

          double box_center_x =
              ((1.0 / (1.0 + std::exp(-center_x))) * 2 - 0.5 + w) * stride;
          double box_center_y =
              ((1.0 / (1.0 + std::exp(-center_y))) * 2 - 0.5 + h) * stride;

          double box_scale_x =
              std::pow((1.0 / (1.0 + std::exp(-scale_x))) * 2, 2) * anchor_x;
          double box_scale_y =
              std::pow((1.0 / (1.0 + std::exp(-scale_y))) * 2, 2) * anchor_y;

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

          if (xmax_org <= 0 || ymax_org <= 0) {
            continue;
          }

          if (xmin_org > xmax_org || ymin_org > ymax_org) {
            continue;
          }

          // 把box的坐标限制在图像大小范围内
          xmin_org = std::max(xmin_org, 0.0);
          xmax_org = std::min(xmax_org, post_info->ori_width - 1.0);
          ymin_org = std::max(ymin_org, 0.0);
          ymax_org = std::min(ymax_org, post_info->ori_height - 1.0);

          // 实际在原图上的box，添加到检测结果中
          Bbox bbox(xmin_org, ymin_org, xmax_org, ymax_org);
          dets.emplace_back((int)id,
                    confidence,
                    bbox,
                    default_yolov5_config.class_names[(int)id].c_str());

        }
        data = data + num_pred * anchors.size();
      }
    }
  }else if (quanti_type == hbDNNQuantiType::SCALE) {
    auto *data = reinterpret_cast<int32_t *>(tensor->sysMem[0].virAddr);
    auto dequantize_scale_ptr = reinterpret_cast<float *>(tensor->properties.scale.scaleData);
    bool big_endian = false;
    for (int32_t h = 0; h < height; h++) {
      for (int32_t w = 0; w < width; w++) {
        for (int k = 0; k < anchor_num; k++) {
          double anchor_x = anchors[k].first;
          double anchor_y = anchors[k].second;
          int32_t *cur_data = data + k * num_pred;
          int offset = num_pred * k;

          float objness = DequantiScale(
              cur_data[4], big_endian, *(dequantize_scale_ptr + offset + 4));

          double max_cls_data = std::numeric_limits<double>::lowest();
          int id{0};
          float32x4_t max;
          for (int cls = 0; cls < num_classes; cls += 4) {
            for (int j = 0; j < 4; j++) {
              float score = cur_data[cls + 5 + j] *
                            dequantize_scale_ptr[offset + cls + 5 + j];
              if (score > max_cls_data) {
                max_cls_data = score;
                id = cls + j;
              }
            }
          }

          double x1 = 1 / (1 + std::exp(-objness)) * 1;
          double x2 = 1 / (1 + std::exp(-max_cls_data));
          double confidence = x1 * x2;

          if (confidence < post_info->score_threshold) {
            continue;
          }

          float center_x = DequantiScale(
              cur_data[0], big_endian, *(dequantize_scale_ptr + offset));
          float center_y = DequantiScale(
              cur_data[1], big_endian, *(dequantize_scale_ptr + offset + 1));
          float scale_x = DequantiScale(
              cur_data[2], big_endian, *(dequantize_scale_ptr + offset + 2));
          float scale_y = DequantiScale(
              cur_data[3], big_endian, *(dequantize_scale_ptr + offset + 3));

          double box_center_x =
              ((1.0 / (1.0 + std::exp(-center_x))) * 2 - 0.5 + w) * stride;
          double box_center_y =
              ((1.0 / (1.0 + std::exp(-center_y))) * 2 - 0.5 + h) * stride;

          double box_scale_x =
              std::pow((1.0 / (1.0 + std::exp(-scale_x))) * 2, 2) * anchor_x;
          double box_scale_y =
              std::pow((1.0 / (1.0 + std::exp(-scale_y))) * 2, 2) * anchor_y;

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

          if (xmax_org <= 0 || ymax_org <= 0) {
            continue;
          }

          if (xmin_org > xmax_org || ymin_org > ymax_org) {
            continue;
          }

          xmin_org = std::max(xmin_org, 0.0);
          xmax_org = std::min(xmax_org, post_info->ori_width - 1.0);
          ymin_org = std::max(ymin_org, 0.0);
          ymax_org = std::min(ymax_org, post_info->ori_height - 1.0);
          xmax_org = std::max(xmax_org, 0.0);
          ymax_org = std::max(ymax_org, 0.0);

          Bbox bbox(xmin_org, ymin_org, xmax_org, ymax_org);
          dets.emplace_back((int)id,
                            confidence,
                            bbox,
                            default_yolov5_config.class_names[(int)id].c_str());
        }
        data = data + num_pred * anchors.size();
      }
    }
  } else {
    printf("yolov5x unsupport shift dequantzie now!\n");
    return;
  }
}



// Yolov5 输出tensor格式
// 3次下采样得到三组缩小后的gred，然后对每个gred进行三次预测，最后输出结果
char* Yolov5PostProcess(Yolov5PostProcessInfo_t *post_info) {

  int i = 0;
  char *str_dets;

  // 计算交并比来合并检测框，传入交并比阈值(0.65)和返回box数量(5000)
  yolov5_nms(dets, post_info->nms_threshold, post_info->nms_top_k, det_restuls, false);
  std::stringstream out_string;

  // 算法结果转换成json格式
  int det_restuls_size = det_restuls.size();
  out_string << "\"yolov5_result\": [";
  for (i = 0; i < det_restuls_size; i++) {
  	auto det_ret = det_restuls[i];
  	out_string << det_ret;
  	if (i < det_restuls_size - 1)
		out_string << ",";
  }
  out_string << "]" << std::endl;

  str_dets = (char *)malloc(out_string.str().length() + 1);
  str_dets[out_string.str().length()] = '\0';
  snprintf(str_dets, out_string.str().length(), "%s", out_string.str().c_str());
  // printf("str_dets: %s\n", str_dets);
  dets.clear();
  det_restuls.clear();
  return str_dets;
}

