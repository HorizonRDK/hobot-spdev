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


#include "centernet_post_process.h"

#define BSWAP_32(x) static_cast<int32_t>(__builtin_bswap32(x))

#define r_int32(x, big_endian) \
  (big_endian) ? BSWAP_32((x)) : static_cast<int32_t>((x))

/**
 * Config definition for Centernet
 */
struct PTQCenternetConfig {
  int class_num;
  std::vector<std::string> class_names;
};

PTQCenternetConfig default_ptq_centernet_config = {
    80, {"person",        "bicycle",      "car",
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

struct DecodeData {
  float topk_score;
  int topk_inds;
  int topk_clses;
  float topk_ys;
  float topk_xs;

  friend bool operator>(const DecodeData &ldt, const DecodeData &rdt) {
    return (ldt.topk_clses > rdt.topk_clses);
  }
};

struct DataNode {
  float value;
  int indx;

  friend bool operator>(const DataNode &ldt, const DataNode &rdt) {
    return (ldt.value > rdt.value);
  }
};

static float fastExp(float x) {
  union {
    uint32_t i;
    float f;
  } v;
  v.i = (12102203.1616540672f * x + 1064807160.56887296f);
  return v.f;
}

// order topK data in node
static void top_k_helper(DataNode *node, int topk, int len) {
  std::priority_queue<int, std::vector<DataNode>, std::greater<DataNode>> heap;
  int i = 0;
  while (i < len) {
    if (i < topk) {
      heap.push(node[i]);
    } else {
      if (heap.top().value < node[i].value) {
        heap.pop();
        heap.push(node[i]);
      }
    }
    i++;
  }
  for (int j = 0; j < topk; j++) {
    node[j] = heap.top();
    heap.pop();
  }
}

#define BSWAP_32(x) static_cast<int32_t>(__builtin_bswap32(x))

#define r_int32(x, big_endian) \
  (big_endian) ? BSWAP_32((x)) : static_cast<int32_t>((x))


template <typename DType>
float quanti_scale_function(DType data, float scale) {
  return static_cast<float>(data) * scale;
}

int filter_func(hbDNNTensor *tensor,
                std::vector<DataNode> &node,
                float &t_value,
                float *scale) {
  int h_index{2}, w_index{3}, c_index{1};
  int *shape = tensor->properties.validShape.dimensionSize;
  int input_c = shape[c_index];
  int input_h = shape[h_index];
  int input_w = shape[w_index];

  int num_elements = input_c * input_h * input_w;
  int16_t *raw_heat_map_data =
      reinterpret_cast<int16_t *>(tensor->sysMem[0].virAddr);

  // per-tensor way
  float threshold = t_value / scale[0];
  DataNode tmp_node;
  for (int i = 0; i < num_elements; ++i) {
    // compare with int16 data
    if (raw_heat_map_data[i] > static_cast<int16_t>(threshold)) {
      // dequantize
      tmp_node.value = quanti_scale_function(raw_heat_map_data[i], scale[0]);
      tmp_node.indx = i;
      node.emplace_back(tmp_node);
    }
  }
  return 0;
}

static float DequantiScale(int32_t data, bool big_endian, float &scale_value) {
  return static_cast<float>(r_int32(data, big_endian)) * scale_value;
}

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
       << ":\"" << default_ptq_centernet_config.class_names[det.id] << "\"}";
    return os; 
  }
 
  ~Detection() {}
} Detection;

std::vector<Detection> centernet_dets;


void Centernet_resnet101_doProcess(hbDNNTensor *nms_tensor, hbDNNTensor *wh_tensor, hbDNNTensor *reg_tensor, CenternetPostProcessInfo_t *post_info, int layer){

  int h_index{2}, w_index{3}, c_index{1};
  int *shape = nms_tensor->properties.validShape.dimensionSize;
  int area = shape[h_index] * shape[w_index];

  int origin_height = post_info->ori_height;
  int origin_width = post_info->ori_width;
  float scale_x = 1.0;
  float scale_y = 1.0;
  float offset_x = 0.0;
  float offset_y = 0.0;
  if (post_info->is_pad_resize) {
    float pad_len = origin_height > origin_width ? origin_height : origin_width;
    scale_x = pad_len / static_cast<float>(shape[w_index]);
    scale_y = pad_len / static_cast<float>(shape[h_index]);
    offset_x = (pad_len - origin_width) / 2;
    offset_y = (pad_len - origin_height) / 2;
  } else {
    scale_x = origin_width / 128.0;
    scale_y = origin_height / 128.0;
  }

  // Determine whether the model contains a dequnatize node by the first tensor
  auto quanti_type = nms_tensor->properties.quantiType;

  std::vector<DataNode> node;

  if (quanti_type == hbDNNQuantiType::SCALE) {
    auto &scales0 = nms_tensor->properties.scale.scaleData;
    // filter score with dequantize
    filter_func(nms_tensor, node, post_info->score_threshold, scales0);
  } else {
    printf("centernet unsupport shift dequantzie now!\n");
    return;
  }

  // topk sort
  int topk = node.size() > post_info->nms_top_k ? post_info->nms_top_k : node.size();
  if (topk != 0) top_k_helper(node.data(), topk, node.size());

  std::vector<float> reg_x(topk);
  std::vector<float> reg_y(topk);
  std::vector<Detection> tmp_box(topk);

  if (wh_tensor->properties.quantiType == hbDNNQuantiType::SCALE) {
    int32_t *wh = reinterpret_cast<int32_t *>(wh_tensor->sysMem[0].virAddr);
    int32_t *reg = reinterpret_cast<int32_t *>(reg_tensor->sysMem[0].virAddr);
    for (int i = 0; i < topk; i++) {
      float topk_score = node[i].value;

      if (topk_score <= post_info->score_threshold) {
        continue;
      }

      // bbox decode with dequantize
      int topk_clses = node[i].indx / area;
      int topk_inds = node[i].indx % area;
      float topk_ys = static_cast<float>(topk_inds / shape[h_index]);
      float topk_xs = static_cast<float>(topk_inds % shape[w_index]);

      auto &wh_scale = wh_tensor->properties.scale.scaleData;
      auto &reg_scale = reg_tensor->properties.scale.scaleData;
      // per-channel way
      topk_xs += quanti_scale_function(reg[topk_inds], *reg_scale);
      topk_ys += quanti_scale_function(reg[area + topk_inds], *(reg_scale + 1));
      // per-channel way
      float wh_0 = quanti_scale_function(wh[topk_inds], *wh_scale);
      float wh_1 = quanti_scale_function(wh[area + topk_inds], *(wh_scale + 1));

      tmp_box[i].bbox.xmin = topk_xs - wh_0 / 2;
      tmp_box[i].bbox.xmax = topk_xs + wh_0 / 2;
      tmp_box[i].bbox.ymin = topk_ys - wh_1 / 2;
      tmp_box[i].bbox.ymax = topk_ys + wh_1 / 2;

      tmp_box[i].score = topk_score;
      tmp_box[i].id = topk_clses;
      tmp_box[i].class_name = default_ptq_centernet_config.class_names[topk_clses].c_str();
      centernet_dets.push_back(tmp_box[i]);
    }
  } else {
    printf("centernet unsupport now!\n");
    return;
  }

  auto &detections = centernet_dets;
  int det_num = centernet_dets.size();
  printf("det.size(): %d", det_num);
  for (int i = 0; i < det_num; i++) {
    detections[i].bbox.xmin = detections[i].bbox.xmin * scale_x - offset_x;
    detections[i].bbox.xmax = detections[i].bbox.xmax * scale_x - offset_x;
    detections[i].bbox.ymin = detections[i].bbox.ymin * scale_y - offset_y;
    detections[i].bbox.ymax = detections[i].bbox.ymax * scale_y - offset_y;
  }

  return;

}


int NMSMaxPool2dDequanti(hbDNNTensor *tensor,
                         std::vector<DataNode> &node,
                         float &t_value,
                         float *scale) {
  int h_index{2}, w_index{3}, c_index{1};
  int *shape = tensor->properties.validShape.dimensionSize;
  int input_c = shape[c_index];
  int input_h = shape[h_index];
  int input_w = shape[w_index];

  auto *raw_heat_map_data =
      reinterpret_cast<int32_t *>(tensor->sysMem[0].virAddr);

  std::vector<int> edge1 = {-1, +1, +input_w - 1, +input_w, +input_w + 1};

  std::vector<int> edge2 = {
      -1,
      +1,
      -input_w - 1,
      -input_w,
      -input_w + 1,
  };

  std::vector<int> edge3 = {
      +1,
      +input_w,
      +input_w + 1,
      -input_w,
      -input_w + 1,
  };

  std::vector<int> edge4 = {
      -1,
      +input_w - 1,
      +input_w,
      -input_w - 1,
      -input_w,
  };

  int pos[8] = {+1,
                -1,
                +input_w - 1,
                +input_w,
                +input_w + 1,
                -input_w - 1,
                -input_w,
                -input_w + 1};

  bool big_endian = false;

  for (int c = 0; c < input_c; c++) {
    int channel_offset = c * input_h * input_w;
    int32_t *iptr = raw_heat_map_data + channel_offset;
    float scale_value = scale[c];
    // VLOG(EXAMPLE_REPORT) << "c: " << c << "; scale1:" << scale[c];
    DataNode tmp_node;

    std::vector<int32_t> point1(4, 0.f);
    point1[0] = iptr[0];
    point1[1] = iptr[1];
    point1[2] = iptr[input_w + 0];
    point1[3] = iptr[input_w + 1];
    int i = 0;
    for (; i < 4; ++i) {
      if (point1[0] < point1[i]) break;
    }
    if (i == 4) {
      tmp_node.value = DequantiScale(point1[0], big_endian, scale_value);
      if (tmp_node.value > t_value) {
        tmp_node.indx = channel_offset;
        node.emplace_back(tmp_node);
      }
    }

    std::vector<int32_t> point2(4, 0.f);
    point2[0] = iptr[input_w - 2];
    point2[1] = iptr[input_w - 1];  //
    point2[2] = iptr[input_w + input_w - 2];
    point2[3] = iptr[input_w + input_w - 1];
    i = 0;
    for (; i < 4; ++i) {
      if (point2[1] < point2[i]) break;
    }
    if (i == 4) {
      tmp_node.value = DequantiScale(point2[1], big_endian, scale_value);  //
      if (tmp_node.value > t_value) {
        tmp_node.indx = channel_offset + input_w - 1;
        node.emplace_back(tmp_node);
      }
    }

    std::vector<int32_t> point3(4, 0.f);
    point3[0] = iptr[(input_h - 2) * input_w + 0];
    point3[1] = iptr[(input_h - 2) * input_w + 1];
    point3[2] = iptr[(input_h - 1) * input_w + 0];  //
    point3[3] = iptr[(input_h - 1) * input_w + 1];
    i = 0;
    for (; i < 4; ++i) {
      if (point3[2] < point3[i]) break;
    }
    if (i == 4) {
      tmp_node.value = DequantiScale(point3[2], big_endian, scale_value);
      if (tmp_node.value > t_value) {
        tmp_node.indx = channel_offset + (input_h - 1) * input_w;
        node.emplace_back(tmp_node);
      }
    }

    std::vector<int32_t> point4(4, 0.f);
    point4[0] = iptr[(input_h - 2) * input_w + (input_w - 2)];
    point4[1] = iptr[(input_h - 2) * input_w + (input_w - 1)];
    point4[2] = iptr[(input_h - 1) * input_w + (input_w - 2)];
    point4[3] = iptr[(input_h - 1) * input_w + (input_w - 1)];  //
    i = 0;
    for (; i < 4; ++i) {
      if (point4[3] < point4[i]) break;
    }
    if (i == 4) {
      tmp_node.value = DequantiScale(point4[3], big_endian, scale_value);
      if (tmp_node.value > t_value) {
        tmp_node.indx = channel_offset + (input_h - 1) * input_w + input_w - 1;
        node.emplace_back(tmp_node);
      }
    }

    // top + bottom
    for (int w = 1; w < input_w - 1; ++w) {
      i = 0;
      int32_t cur_value = iptr[w];
      for (; i < 5; ++i) {
        if (cur_value < iptr[w + edge1[i]]) break;
      }
      if (i == 5) {
        tmp_node.value = DequantiScale(cur_value, big_endian, scale_value);
        if (tmp_node.value > t_value) {
          tmp_node.indx = channel_offset + w;
          node.emplace_back(tmp_node);
        }
      }

      i = 0;
      int cur_pos = (input_h - 1) * input_w + w;
      cur_value = iptr[cur_pos];
      for (; i < 5; ++i) {
        if (cur_value < iptr[cur_pos + edge2[i]]) break;
      }
      if (i == 5) {
        tmp_node.value = DequantiScale(cur_value, big_endian, scale_value);
        if (tmp_node.value > t_value) {
          tmp_node.indx = channel_offset + cur_pos;
          node.emplace_back(tmp_node);
        }
      }
    }

    // left + right
    for (int h = 1; h < input_h - 1; ++h) {
      i = 0;
      int cur_pos = h * input_w;
      int32_t cur_value = iptr[cur_pos];
      for (; i < 5; ++i) {
        if (cur_value < iptr[cur_pos + edge3[i]]) break;
      }
      if (i == 5) {
        tmp_node.value = DequantiScale(cur_value, big_endian, scale_value);
        if (tmp_node.value > t_value) {
          tmp_node.indx = channel_offset + cur_pos;
          node.emplace_back(tmp_node);
        }
      }

      i = 0;
      cur_pos = h * input_w + input_w - 1;
      cur_value = iptr[cur_pos];
      for (; i < 5; ++i) {
        if (cur_value < iptr[cur_pos + edge3[i]]) break;
      }
      if (i == 5) {
        tmp_node.value = DequantiScale(cur_value, big_endian, scale_value);
        if (tmp_node.value > t_value) {
          tmp_node.indx = channel_offset + cur_pos;
          node.emplace_back(tmp_node);
        }
      }
    }

    // center
    for (int h = 1; h < input_h - 1; h++) {
      int offset = h * input_w;
      for (int w = 1; w < input_w - 1; w++) {
        int cur_pos = offset + w;
        int32_t cur_value = iptr[cur_pos];
        i = 0;
        for (; i < 8; ++i) {
          if (cur_value < iptr[cur_pos + pos[i]]) break;
        }
        if (i == 8) {
          tmp_node.value = DequantiScale(cur_value, big_endian, scale_value);
          if (tmp_node.value > t_value) {
            tmp_node.indx = channel_offset + cur_pos;
            node.emplace_back(tmp_node);
          }
        }
      }
    }
  }
  return 0;
}

int NMSMaxPool2d(hbDNNTensor *tensor,
                 std::vector<DataNode> &node,
                 float &t_value) {
  int h_index{2}, w_index{3}, c_index{1};
  int *shape = tensor->properties.validShape.dimensionSize;
  int input_c = shape[c_index];
  int input_h = shape[h_index];
  int input_w = shape[w_index];

  float *raw_heat_map_data =
      reinterpret_cast<float *>(tensor->sysMem[0].virAddr);

  std::vector<int> edge1 = {-1, +1, +input_w - 1, +input_w, +input_w + 1};

  std::vector<int> edge2 = {
      -1,
      +1,
      -input_w - 1,
      -input_w,
      -input_w + 1,
  };

  std::vector<int> edge3 = {
      +1,
      +input_w,
      +input_w + 1,
      -input_w,
      -input_w + 1,
  };

  std::vector<int> edge4 = {
      -1,
      +input_w - 1,
      +input_w,
      -input_w - 1,
      -input_w,
  };

  int pos[8] = {+1,
                -1,
                +input_w - 1,
                +input_w,
                +input_w + 1,
                -input_w - 1,
                -input_w,
                -input_w + 1};

  for (int c = 0; c < input_c; c++) {
    int channel_offset = c * input_h * input_w;
    float *iptr = raw_heat_map_data + channel_offset;
    DataNode tmp_node;

    std::vector<float> point1(4, 0.f);
    point1[0] = iptr[0];  //
    point1[1] = iptr[1];
    point1[2] = iptr[input_w + 0];
    point1[3] = iptr[input_w + 1];
    int i = 0;
    for (; i < 4; ++i) {
      if (point1[0] < point1[i]) break;
    }
    if (i == 4 && point1[0] > t_value) {
      tmp_node.indx = channel_offset;
      tmp_node.value = point1[0];
      node.emplace_back(tmp_node);
    }

    std::vector<float> point2(4, 0.f);
    point2[0] = iptr[input_w - 2];
    point2[1] = iptr[input_w - 1];  //
    point2[2] = iptr[input_w + input_w - 2];
    point2[3] = iptr[input_w + input_w - 1];
    i = 0;
    for (; i < 4; ++i) {
      if (point2[1] < point2[i]) break;
    }
    if (i == 4 && point2[1] > t_value) {
      tmp_node.indx = channel_offset + input_w - 1;
      tmp_node.value = point2[1];  //
      node.emplace_back(tmp_node);
    }

    std::vector<float> point3(4, 0.f);
    point3[0] = iptr[(input_h - 2) * input_w + 0];
    point3[1] = iptr[(input_h - 2) * input_w + 1];
    point3[2] = iptr[(input_h - 1) * input_w + 0];  //
    point3[3] = iptr[(input_h - 1) * input_w + 1];
    i = 0;
    for (; i < 4; ++i) {
      if (point3[2] < point3[i]) break;
    }
    if (i == 4 && point3[2] > t_value) {
      tmp_node.indx = channel_offset + (input_h - 1) * input_w;
      tmp_node.value = point3[2];
      node.emplace_back(tmp_node);
    }

    std::vector<float> point4(4, 0.f);
    point4[0] = iptr[(input_h - 2) * input_w + (input_w - 2)];
    point4[1] = iptr[(input_h - 2) * input_w + (input_w - 1)];
    point4[2] = iptr[(input_h - 1) * input_w + (input_w - 2)];
    point4[3] = iptr[(input_h - 1) * input_w + (input_w - 1)];  //
    i = 0;
    for (; i < 4; ++i) {
      if (point4[3] < point4[i]) break;
    }
    if (i == 4 && point4[3] > t_value) {
      tmp_node.indx = channel_offset + (input_h - 1) * input_w + input_w - 1;
      tmp_node.value = point4[3];
      node.emplace_back(tmp_node);
    }

    // top + bottom
    for (int w = 1; w < input_w - 1; ++w) {
      i = 0;
      float cur_value = iptr[w];
      for (; i < 5; ++i) {
        if (cur_value < iptr[w + edge1[i]]) break;
      }
      if (i == 5 && cur_value > t_value) {
        tmp_node.indx = channel_offset + w;
        tmp_node.value = cur_value;
        node.emplace_back(tmp_node);
      }

      i = 0;
      int cur_pos = (input_h - 1) * input_w + w;
      cur_value = iptr[cur_pos];
      for (; i < 5; ++i) {
        if (cur_value < iptr[cur_pos + edge2[i]]) break;
      }
      if (i == 5 && cur_value > t_value) {
        tmp_node.indx = channel_offset + cur_pos;
        tmp_node.value = cur_value;
        node.emplace_back(tmp_node);
      }
    }

    // left + right
    for (int h = 1; h < input_h - 1; ++h) {
      i = 0;
      int cur_pos = h * input_w;
      float cur_value = iptr[cur_pos];
      for (; i < 5; ++i) {
        if (cur_value < iptr[cur_pos + edge3[i]]) break;
      }
      if (i == 5 && cur_value > t_value) {
        tmp_node.indx = channel_offset + cur_pos;
        tmp_node.value = cur_value;
        node.emplace_back(tmp_node);
      }

      i = 0;
      cur_pos = h * input_w + input_w - 1;
      cur_value = iptr[cur_pos];
      for (; i < 5; ++i) {
        if (iptr[cur_pos] < iptr[cur_pos + edge4[i]]) break;
      }
      if (i == 5 && cur_value > t_value) {
        tmp_node.indx = channel_offset + cur_pos;
        tmp_node.value = cur_value;
        node.emplace_back(tmp_node);
      }
    }

    // center
    for (int h = 1; h < input_h - 1; h++) {
      int offset = h * input_w;
      for (int w = 1; w < input_w - 1; w++) {
        int cur_pos = offset + w;
        float cur_value = iptr[cur_pos];
        i = 0;
        for (; i < 8; ++i) {
          if (cur_value < iptr[cur_pos + pos[i]]) break;
        }
        if (i == 8 && cur_value > t_value) {
          tmp_node.indx = channel_offset + cur_pos;
          tmp_node.value = cur_value;
          node.emplace_back(tmp_node);
        }
      }
    }
  }
  return 0;
}

void CenternetdoProcess(hbDNNTensor *nms_tensor, hbDNNTensor *wh_tensor, hbDNNTensor *reg_tensor, CenternetPostProcessInfo_t *post_info, int layer) {

  int h_index{2}, w_index{3}, c_index{1};
  int *shape = nms_tensor->properties.validShape.dimensionSize;
  int area = shape[w_index] * shape[w_index];

  int origin_height = post_info->ori_height;
  int origin_width = post_info->ori_width;
  float scale_x = 1.0;
  float scale_y = 1.0;
  float offset_x = 0.0;
  float offset_y = 0.0;
  if (post_info->is_pad_resize) {
    float pad_len = origin_height > origin_width ? origin_height : origin_width;
    scale_x = pad_len / static_cast<float>(shape[w_index]);
    scale_y = pad_len / static_cast<float>(shape[h_index]);
    offset_x = (pad_len - origin_width) / 2;
    offset_y = (pad_len - origin_height) / 2;
  } else {
    scale_x = origin_width / 128.0;
    scale_y = origin_height / 128.0;
  }

  // Determine whether the model contains a dequnatize node by the first tensor
  auto quanti_type = nms_tensor->properties.quantiType;

  std::vector<DataNode> node;
  float t_value =
      log(post_info->score_threshold / (1.f - post_info->score_threshold));  // ln (2.f/3.f)
  if (quanti_type == hbDNNQuantiType::NONE) {
    NMSMaxPool2d(nms_tensor, node, t_value);
  } else if (quanti_type == hbDNNQuantiType::SCALE) {
    auto &scales0 = nms_tensor->properties.scale.scaleData;
    NMSMaxPool2dDequanti(nms_tensor, node, t_value, scales0);
  } else {
    printf("centernet unsupport shift dequantzie now!\n");
    return;
  }

  int topk = node.size() > post_info->nms_top_k ? post_info->nms_top_k : node.size();
  if (topk != 0) top_k_helper(node.data(), topk, node.size());

  std::vector<float> reg_x(topk);
  std::vector<float> reg_y(topk);
  std::vector<Detection> tmp_box(topk);

  if (quanti_type == hbDNNQuantiType::NONE) {
    float *wh = reinterpret_cast<float *>(wh_tensor->sysMem[0].virAddr);
    float *reg = reinterpret_cast<float *>(reg_tensor->sysMem[0].virAddr);

    for (int i = 0; i < topk; i++) {
      // float topk_score = 1.0 / (1.0 + exp(-node[i].value));
      float topk_score = 1.0 / (1.0 + fastExp(-node[i].value));
      if (topk_score <= post_info->score_threshold) {
        continue;
      }

      int topk_clses = node[i].indx / area;
      int topk_inds = node[i].indx % area;
      float topk_ys = static_cast<float>(topk_inds / shape[w_index]);
      float topk_xs = static_cast<float>(topk_inds % shape[w_index]);

      topk_xs += reg[topk_inds];
      topk_ys += reg[area + topk_inds];

      tmp_box[i].bbox.xmin = topk_xs - wh[topk_inds] / 2;
      tmp_box[i].bbox.xmax = topk_xs + wh[topk_inds] / 2;
      tmp_box[i].bbox.ymin = topk_ys - wh[area + topk_inds] / 2;
      tmp_box[i].bbox.ymax = topk_ys + wh[area + topk_inds] / 2;

      tmp_box[i].score = topk_score;
      tmp_box[i].id = topk_clses;
      tmp_box[i].class_name = default_ptq_centernet_config.class_names[topk_clses].c_str();
      centernet_dets.push_back(tmp_box[i]);
    }

  } else if (quanti_type == hbDNNQuantiType::SCALE) {
    bool big_endian = false;
    int32_t *reg = reinterpret_cast<int32_t *>(reg_tensor->sysMem[0].virAddr);
    int32_t *wh = reinterpret_cast<int32_t *>(wh_tensor->sysMem[0].virAddr);

    for (int i = 0; i < topk; i++) {
      // float topk_score = 1.0 / (1.0 + exp(-node[i].value));
      float topk_score = 1.0 / (1.0 + fastExp(-node[i].value));
      if (topk_score <= post_info->score_threshold) {
        continue;
      }

      int topk_clses = node[i].indx / area;
      int topk_inds = node[i].indx % area;
      float topk_ys = static_cast<float>(topk_inds / shape[w_index]);
      float topk_xs = static_cast<float>(topk_inds % shape[w_index]);

      auto &scales1 = wh_tensor->properties.scale.scaleData;
      auto &scales2 = reg_tensor->properties.scale.scaleData;
      topk_xs += DequantiScale(reg[topk_inds], big_endian, *scales2);
      topk_ys +=
          DequantiScale(reg[area + topk_inds], big_endian, *(scales2 + 1));

      float wh_0 = DequantiScale(wh[topk_inds], big_endian, *scales1);

      float wh_1 =
          DequantiScale(wh[area + topk_inds], big_endian, *(scales1 + 1));

      tmp_box[i].bbox.xmin = topk_xs - wh_0 / 2;
      tmp_box[i].bbox.xmax = topk_xs + wh_0 / 2;
      tmp_box[i].bbox.ymin = topk_ys - wh_1 / 2;
      tmp_box[i].bbox.ymax = topk_ys + wh_1 / 2;

      tmp_box[i].score = topk_score;
      tmp_box[i].id = topk_clses;
      tmp_box[i].class_name = default_ptq_centernet_config.class_names[topk_clses].c_str();
      centernet_dets.push_back(tmp_box[i]);
    }
  } else {
    printf("centernet unsupport shift dequantzie now!\n");
    return;
  }

  auto &detections = centernet_dets;
  int det_num = centernet_dets.size();
  printf("det.size(): %d\n", det_num);
  for (int i = 0; i < det_num; i++) {
    detections[i].bbox.xmin = detections[i].bbox.xmin * scale_x - offset_x;
    detections[i].bbox.xmax = detections[i].bbox.xmax * scale_x - offset_x;
    detections[i].bbox.ymin = detections[i].bbox.ymin * scale_y - offset_y;
    detections[i].bbox.ymax = detections[i].bbox.ymax * scale_y - offset_y;
  }

  return;

}

char* CenternetPostProcess(CenternetPostProcessInfo_t *post_info) {

  int i = 0;
  char *str_dets;

  std::stringstream out_string;
  std::vector<Detection> centernet_det_restuls;

  centernet_det_restuls = centernet_dets;
  // 算法结果转换成json格式
  int centernet_det_restuls_size = centernet_det_restuls.size();
  out_string << "\"centernet_result\": [";
  for (i = 0; i < centernet_det_restuls_size; i++) {
  	auto det_ret = centernet_det_restuls[i];
  	out_string << det_ret;
  	if (i < centernet_det_restuls_size - 1)
		out_string << ",";
  }
  out_string << "]" << std::endl;

  str_dets = (char *)malloc(out_string.str().length() + 1);
  str_dets[out_string.str().length()] = '\0';
  snprintf(str_dets, out_string.str().length(), "%s", out_string.str().c_str());
  // printf("str_dets: %s\n", str_dets);
  centernet_dets.clear();
  centernet_det_restuls.clear();
  return str_dets;
}

