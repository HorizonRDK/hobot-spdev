/*
 * Copyright (C) 2024 D-Robotics Inc.
 * All rights reserved.
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <atomic>
#include <cstdbool>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <map>

#include <functional>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

#include "dnn_python.h"

using namespace std;

#ifdef __cplusplus
extern "C" {
#endif

std::string TensorType[] = {"Y",
                            "NV12",
                            "NV12_SEPARATE",
                            "YUV444",
                            "RGB",
                            "BGR",
                            "int4",
                            "uint4",
                            "int8",
                            "uint8",
                            "float16",
                            "int16",
                            "uint16",
                            "float32",
                            "int32",
                            "uint32",
                            "float64",
                            "int64",
                            "uint64"};

std::map<int, std::string> TensorLayout = {{0, "NHWC"}, {2, "NCHW"}, {255, "NONE"}};

/**
 * C++ => Python
 * Convert buffer to numpy array
 * @param arr
 * @param properties
 * @return PyArrayObject*
 */
PyObject* buffer_2_numpy(void *arr, hbDNNTensorProperties &properties, int npy_type, int32_t type_size) {
    hbDNNTensorShape buffer_shape = properties.validShape;
    int size = 1;
    for (int i = 0; i < 4; i++) {
        size *= buffer_shape.dimensionSize[i];
    }

    // 创建 numpy 数组对象
    npy_intp dims[4];
    for (int i = 0; i < 4; ++i) {
        dims[i] = buffer_shape.dimensionSize[i];
    }
    PyObject *array = PyArray_SimpleNew(4, dims, npy_type);
    if (!array) {
        PyErr_SetString(PyExc_RuntimeError, "Failed to create numpy array.");
        return NULL;
    }

    // 从缓冲区复制数据到 numpy 数组中
    void *data_ptr = PyArray_DATA((PyArrayObject*)array);
    memcpy(data_ptr, arr, size * type_size);

    return array;
}

PyObject* buffer_2_pyarray(void *arr, hbDNNTensorProperties &properties) {
    hbDNNDataType dataType = static_cast<hbDNNDataType>(properties.tensorType);

    switch (dataType) {
        case HB_DNN_TENSOR_TYPE_S8:
            return buffer_2_numpy(reinterpret_cast<int8_t *>(arr), properties, NPY_INT8, sizeof(int8_t));
        case HB_DNN_TENSOR_TYPE_S32:
            return buffer_2_numpy(reinterpret_cast<int32_t *>(arr), properties, NPY_INT32, sizeof(int32_t));
        case HB_DNN_TENSOR_TYPE_F32:
            return buffer_2_numpy(reinterpret_cast<float *>(arr), properties, NPY_FLOAT, sizeof(float));
        default:
            // Do not support data type, using default int8 for output.
            return buffer_2_numpy(reinterpret_cast<int8_t *>(arr), properties, NPY_INT8, sizeof(int8_t));
    }
}


int32_t NumpyCopyHelper(hbDNNTensor *input_tensor,
                        unsigned char *src,
                        uint32_t size) {
  void *dst = input_tensor->sysMem[0].virAddr;
  switch (input_tensor->properties.tensorType) {
    case HB_DNN_IMG_TYPE_YUV444:
    case HB_DNN_IMG_TYPE_RGB:
    case HB_DNN_IMG_TYPE_BGR:
    case HB_DNN_TENSOR_TYPE_U8:
      memcpy(dst, src, size);
      break;
    case HB_DNN_TENSOR_TYPE_S8:
      memcpy(dst, src, size);
      break;
    case HB_DNN_TENSOR_TYPE_S16:
      memcpy(dst, src, size);
      break;
    case HB_DNN_TENSOR_TYPE_U16:
      memcpy(dst, src, size);
      break;
    case HB_DNN_TENSOR_TYPE_F32:
      memcpy(dst, src, size);
      break;
    case HB_DNN_TENSOR_TYPE_S32:
      memcpy(dst, src, size);
      break;
    case HB_DNN_TENSOR_TYPE_U32:
      memcpy(dst, src, size);
      break;
    case HB_DNN_TENSOR_TYPE_F64:
      memcpy(dst, src, size);
      break;
    case HB_DNN_TENSOR_TYPE_S64:
      memcpy(dst, src, size);
      break;
    case HB_DNN_TENSOR_TYPE_U64:
      memcpy(dst, src, size);
      break;
    default:
      std::cout << "Transpose data type error: "
                << input_tensor->properties.tensorType << std::endl;
      return -1;
  }
  return 0;
}

static PyObject *TensorProperties_new(PyTypeObject *type, PyObject *args, PyObject *kwargs)
{
    TensorProperties *self = (TensorProperties *)type->tp_alloc(type, 0);
    return (PyObject *)self;
}

// 构造函数：初始化类的属性
static int TensorProperties_init(PyObject *self, PyObject *args, PyObject *kwargs) {
    // 初始化类的属性
    ((TensorProperties*)self)->tensor_type = 2;
    ((TensorProperties*)self)->dtype = 1;
    ((TensorProperties*)self)->layout = 3;
    ((TensorProperties*)self)->shape_size = 3; // 假设初始大小为 3
    ((TensorProperties*)self)->shape = (int*)malloc(sizeof(int) * ((TensorProperties*)self)->shape_size);
    // 为 shape 数组分配内存并赋初值
    for (int i = 0; i < ((TensorProperties*)self)->shape_size; ++i) {
        ((TensorProperties*)self)->shape[i] = i; // 假设初始值为 0
    }
    ((TensorProperties*)self)->scale_len = 16;
    ((TensorProperties*)self)->scale_data = (float*)malloc(sizeof(float) * ((TensorProperties*)self)->scale_len);
    // 为 scale_data 数组分配内存并赋初值
    for (int i = 0; i < ((TensorProperties*)self)->scale_len; ++i) {
        ((TensorProperties*)self)->scale_data[i] = 0.0 + i; // 假设初始值为 0.0
    }
    return 0;
}

// 获取 tensor_type 成员属性的 getter 函数
static PyObject* tensorproperties_get_tensor_type(TensorProperties *self, void *closure) {
    // 检查索引是否超出范围
    if (self->tensor_type < 0 || self->tensor_type >= static_cast<int32_t>(sizeof(TensorType) / sizeof(TensorType[0]))) {
        PyErr_SetString(PyExc_IndexError, "Invalid tensor type index");
        return NULL;
    }
    // 从 TensorType 表中获取对应的字符串
    const char* tensor_type_str = TensorType[self->tensor_type].c_str();
    // 将字符串转换为 Python 字符串对象并返回
    return PyUnicode_FromString(tensor_type_str);
}

// 获取 dtype 成员属性的 getter 函数
static PyObject* tensorproperties_get_dtype(TensorProperties *self, void *closure) {
    // 检查索引是否超出范围
    if (self->dtype < 0 || self->dtype >= static_cast<int32_t>(sizeof(TensorType) / sizeof(TensorType[0]))) {
        PyErr_SetString(PyExc_IndexError, "Invalid tensor dtype index");
        return NULL;
    }
    // 从 TensorType 表中获取对应的字符串
    const char* dtype_str;
    if (self->dtype < 6) {
        dtype_str = TensorType[9].c_str();
    } else {
        dtype_str = TensorType[self->dtype].c_str();
    }

    // 将字符串转换为 Python 字符串对象并返回
    return PyUnicode_FromString(dtype_str);
}

// 获取 layout 成员属性的 getter 函数
static PyObject* tensorproperties_get_layout(TensorProperties *self, void *closure) {
    // 查找布局值对应的字符串
    std::string layout_str = "UNKNOWN";
    auto it = TensorLayout.find(self->layout);
    if (it != TensorLayout.end()) {
        layout_str = it->second;
    }
    // 将字符串转换为 Python 字符串对象并返回
    return PyUnicode_FromString(layout_str.c_str());
}

// 获取 shape 成员属性的 getter 函数
static PyObject* tensorproperties_get_shape(TensorProperties *self, void *closure) {
    // 创建一个元组来存储 shape
    PyObject *shape_tuple = PyTuple_New(self->shape_size);
    if (!shape_tuple) {
        // 错误处理，如果创建元组失败
        return NULL;
    }

    for (int i = 0; i < self->shape_size; ++i) {
        PyObject *dim = PyLong_FromLong(self->shape[i]);
        if (!dim) {
            // 错误处理，如果创建 Python 对象失败
            Py_DECREF(shape_tuple); // 释放之前创建的元组
            return NULL;
        }
        // 将 dim 添加到元组中
        PyTuple_SET_ITEM(shape_tuple, i, dim);
    }

    // 返回元组
    return shape_tuple;
}

// 获取 scale_data 成员属性的 getter 函数
static PyObject* tensorproperties_get_scale_data(TensorProperties *self, void *closure) {
    // 创建一个 NumPy 数组来存储 scale_data
    npy_intp dims[] = {self->scale_len}; // 设置数组的维度
    PyObject *scale_array = PyArray_SimpleNewFromData(1, dims, NPY_FLOAT, self->scale_data);

    // 返回 NumPy 数组
    return scale_array;
}


// PyGetSetDef 定义成员属性，使用 getter 函数获取属性值
static PyGetSetDef TensorPropertiesGetSet[] = {
    {"tensor_type", (getter)tensorproperties_get_tensor_type, NULL, "tensor type", NULL},
    {"dtype", (getter)tensorproperties_get_dtype, NULL, "data type", NULL},
    {"layout", (getter)tensorproperties_get_layout, NULL, "layout", NULL},
    {"shape", (getter)tensorproperties_get_shape, NULL, "shape", NULL},
    {"scale_data", (getter)tensorproperties_get_scale_data, NULL, "scale data", NULL},
    {NULL} /* Sentinel */
};

static PyTypeObject TensorPropertiesType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "dnnpy.TensorProperties",                   /* tp_name */
    sizeof(TensorProperties),                      /* tp_basicsize */
    0,                                             /* tp_itemsize */
    (destructor)PyObject_Free,                     /* tp_dealloc */
    0,                                             /* tp_print */
    0,                                             /* tp_getattr */
    0,                                             /* tp_setattr */
    0,                                             /* tp_reserved */
    0,                                             /* tp_repr */
    0,                                             /* tp_as_number */
    0,                                             /* tp_as_sequence */
    0,                                             /* tp_as_mapping */
    0,                                             /* tp_hash */
    0,                                             /* tp_call */
    0,                                             /* tp_str */
    0,                                             /* tp_getattro */
    0,                                             /* tp_setattro */
    0,                                             /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT,                            /* tp_flags */
    "TensorProperties object",                     /* tp_doc */
    0,                                             /* tp_traverse */
    0,                                             /* tp_clear */
    0,                                             /* tp_richcompare */
    0,                                             /* tp_weaklistoffset */
    0,                                             /* tp_iter */
    0,                                             /* tp_iternext */
    0,                                             /* tp_methods */
    0,                                             /* tp_members */
    TensorPropertiesGetSet,                        /* tp_getset */
    0,                                             /* tp_base */
    0,                                             /* tp_dict */
    0,                                             /* tp_descr_get */
    0,                                             /* tp_descr_set */
    0,                                             /* tp_dictoffset */
    (initproc)TensorProperties_init,               /* tp_init */
    0,                                             /* tp_alloc */
    (newfunc)PyType_GenericNew,                    /* tp_new */
    0,                                             /* tp_free */
};

static PyObject *PyDNNTensor_new(PyTypeObject *type, PyObject *args, PyObject *kwargs)
{
    PyDNNTensor *self = (PyDNNTensor *)type->tp_alloc(type, 0);
    return (PyObject *)self;
}

// 构造函数：初始化类的属性
static int PyDNNTensor_init(PyObject *self, PyObject *args, PyObject *kwargs) {
    // 初始化类的属性
    return 0;
}

// 获取 properties 成员属性的 getter 函数
static PyObject* PyDNNTensor_get_properties(PyDNNTensor *self, void *closure) {
    // 创建一个新的 TensorPropertiesType 对象
    TensorProperties *properties_obj = (TensorProperties *)TensorProperties_new(&TensorPropertiesType, NULL, NULL);
    if (properties_obj == NULL) {
        PyErr_SetString(PyExc_RuntimeError, "Failed to create TensorPropertiesType object");
        return NULL;
    }

    // 设置属性
    // shape
    Py_INCREF(Py_None);
    Py_XDECREF(properties_obj->shape);
    properties_obj->shape_size = self->properties.validShape.numDimensions;
    properties_obj->shape = (int32_t*)malloc(sizeof(int32_t) * properties_obj->shape_size);
    // 为 shape 数组分配内存并赋初值
    for (int i = 0; i < properties_obj->shape_size; ++i) {
        properties_obj->shape[i] = self->properties.validShape.dimensionSize[i];
    }

    // dtype
    properties_obj->dtype = self->properties.tensorType;

    // layout
    properties_obj->layout = self->properties.tensorLayout;
    // tensor_type
    properties_obj->tensor_type = self->properties.tensorType;

    properties_obj->scale_len = self->properties.scale.scaleLen;
    properties_obj->scale_data = self->properties.scale.scaleData;

    return Py_BuildValue("O", properties_obj);
}

// 获取 buffer 成员属性的 getter 函数
static PyObject* PyDNNTensor_get_buffer(PyDNNTensor *self, void *closure) {
    // 将 self->buffer 转换为 Python 对象并返回
    // 假设你有一个函数可以将 C 结构体转换为 PyObject

    return buffer_2_pyarray(self->buffer, self->properties);
}

static int32_t GetInputName(hbDNNHandle_t dnn_handle, int32_t input_index,
                       char *input_name) {
    const char *name = NULL;

    RETURN_IF_FAILED(hbDNNGetInputName(&name, dnn_handle, input_index));
    if (name != NULL && input_name != NULL) {
        strcpy(input_name, name);
    }
    return 0;
}

static int32_t GetOutputName(hbDNNHandle_t dnn_handle, int32_t output_index,
                        char *output_name) {
    const char *name = NULL;
    RETURN_IF_FAILED(hbDNNGetOutputName(&name, dnn_handle, output_index));
    if (name != NULL && output_name != NULL) {
        strcpy(output_name, name);
    }
    return 0;
}


// 获取 name 成员属性的 getter 函数
static PyObject* PyDNNTensor_get_name(PyDNNTensor *self, void *closure) {
    // 将 self->name 转换为 Python 对象并返回

    return PyUnicode_FromString(self->name);
}

// PyGetSetDef 定义成员属性，使用 getter 函数获取属性值
static PyGetSetDef PyDNNTensorGetSet[] = {
    {"properties", (getter)PyDNNTensor_get_properties, NULL, "Tensor properties", NULL},
    {"buffer", (getter)PyDNNTensor_get_buffer, NULL, "Tensor buffer", NULL},
    {"name", (getter)PyDNNTensor_get_name, NULL, "Tensor name", NULL},
    {NULL}  /* Sentinel */
};

static PyTypeObject PyDNNTensorType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "dnnpy.PyDNNTensor",                        /* tp_name */
    sizeof(PyDNNTensor),                           /* tp_basicsize */
    0,                                             /* tp_itemsize */
    0,                                             /* tp_dealloc */
    0,                                             /* tp_print */
    0,                                             /* tp_getattr */
    0,                                             /* tp_setattr */
    0,                                             /* tp_reserved */
    0,                                             /* tp_repr */
    0,                                             /* tp_as_number */
    0,                                             /* tp_as_sequence */
    0,                                             /* tp_as_mapping */
    0,                                             /* tp_hash */
    0,                                             /* tp_call */
    0,                                             /* tp_str */
    0,                                             /* tp_getattro */
    0,                                             /* tp_setattro */
    0,                                             /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT,                            /* tp_flags */
    "PyDNNTensor object",                          /* tp_doc */
    0,                                             /* tp_traverse */
    0,                                             /* tp_clear */
    0,                                             /* tp_richcompare */
    0,                                             /* tp_weaklistoffset */
    0,                                             /* tp_iter */
    0,                                             /* tp_iternext */
    0,                                             /* tp_methods */
    0,                                             /* tp_members */
    PyDNNTensorGetSet,                             /* tp_getset */
    0,                                             /* tp_base */
    0,                                             /* tp_dict */
    0,                                             /* tp_descr_get */
    0,                                             /* tp_descr_set */
    0,                                             /* tp_dictoffset */
    (initproc)PyDNNTensor_init,                    /* tp_init */
    0,                                             /* tp_alloc */
    (newfunc)PyType_GenericNew,                    /* tp_new */
    0,                                             /* tp_free */
};

static PyObject *Model_new(PyTypeObject *type, PyObject *args, PyObject *kwargs)
{
    Model_Object *self = (Model_Object *)type->tp_alloc(type, 0);
    return (PyObject *)self;
}

static void Model_dealloc(Model_Object *self)
{
    // release_model_tensor(self);
    self->ob_base.ob_type->tp_free(self);
}

static int Model_init(Model_Object *self, PyObject *args, PyObject *kwargs)
{
    return 0;
}

static PyObject *model_get_model_name(Model_Object *self, void *closure) {
    // 将 Python 对象转换为 Model_Object 指针
    Model_Object *model = (Model_Object *)self;

    // 检查 model 是否为 NULL
    if (!model) {
        PyErr_SetString(PyExc_RuntimeError, "Invalid Model_Object pointer");
        return NULL;
    }

    // 使用 UTF-8 编码解析模型名称，并将其作为 Python Unicode 对象返回
    return PyUnicode_DecodeUTF8(model->name, strlen(model->name), "replace");
}


static PyObject* model_get_tensor_inputs(Model_Object *self, void *closure) {
    // 获取模型的输入张量列表，这里假设 inputs 是一个列表，存储了 PyDNNTensor 对象的引用
    PyObject *inputs_list = PyList_New(0);  // 创建一个空列表
    if (!inputs_list) {
        PyErr_SetString(PyExc_RuntimeError, "Failed to create inputs list.");
        return NULL;
    }

    for (int i = 0; i < self->m_input_count; i++) {
        // 创建一个 Model 对象
        PyDNNTensor *dnn_tensor = PyObject_New(PyDNNTensor, &PyDNNTensorType);
        if (dnn_tensor == NULL) {
            PyErr_SetString(PyExc_RuntimeError, "Failed to create dnn_tensor object");
            Py_DECREF(inputs_list);
            return NULL;
        }
        dnn_tensor->properties = self->m_inputs[i].properties;
        dnn_tensor->buffer = self->m_inputs[i].sysMem[0].virAddr;
        GetInputName(self->m_dnn_handle, i, dnn_tensor->name);

        // 将张量对象添加到列表中
        PyList_Append(inputs_list, (PyObject *)dnn_tensor);
    }

    return inputs_list;
}

static PyObject* model_get_tensor_outputs(Model_Object *self, void *closure) {
    // 获取模型的输出张量列表，这里假设 outputs 是一个列表，存储了 PyDNNTensor 对象的引用
    PyObject *outputs_list = PyList_New(0);  // 创建一个空列表
    if (!outputs_list) {
        PyErr_SetString(PyExc_RuntimeError, "Failed to create outputs list.");
        return NULL;
    }


    for (int i = 0; i < self->m_output_count; i++) {
        // 创建一个 PyDNNTensor 对象
        PyDNNTensor *dnn_tensor = (PyDNNTensor *)PyDNNTensor_new(&PyDNNTensorType, NULL, NULL);
        // PyDNNTensor *dnn_tensor = PyObject_New(PyDNNTensor, &PyDNNTensorType);
        if (dnn_tensor == NULL) {
            PyErr_SetString(PyExc_RuntimeError, "Failed to create dnn_tensor object");
            Py_DECREF(outputs_list);
            return NULL;
        }

        // 初始化 PyDNNTensor 对象的属性
        dnn_tensor->properties = self->m_outputs[i].properties;
        dnn_tensor->buffer = self->m_outputs[i].sysMem[0].virAddr;
        GetOutputName(self->m_dnn_handle, i, dnn_tensor->name);

        // 将张量对象添加到列表中
        if (PyList_Append(outputs_list, (PyObject *)dnn_tensor) == -1) {
            PyErr_SetString(PyExc_RuntimeError, "Failed to append tensor to outputs list");
            Py_DECREF(dnn_tensor);
            Py_DECREF(outputs_list);
            return NULL;
        }
    }

    return outputs_list;
}

static PyObject* model_get_estimate_latency(Model_Object *self, void *closure) {
    // 将延迟时间转换为 Python 整数对象并返回
    return PyLong_FromLong(self->m_estimate_latency);
}

static int32_t forward(Model_Object *model_obj, unsigned char *data_ptr, int32_t data_size, int32_t core_id, int32_t priority) {
    int32_t ret = 0;
    uint32_t input_count = model_obj->m_input_count;

    for (uint32_t index = 0; index < input_count; index++) {
        hbDNNTensor *input_tensor = &model_obj->m_inputs[index];
        auto tensor_type = static_cast<hbDNNDataType>(input_tensor->properties.tensorType);

        // 将数据拷贝到输入张量的系统内存中
        if (tensor_type == HB_DNN_IMG_TYPE_NV12_SEPARATE) {
            memcpy(input_tensor->sysMem[0].virAddr, data_ptr, data_size / 3 * 2);
            memcpy(input_tensor->sysMem[1].virAddr, data_ptr + data_size / 3 * 2, data_size / 3);
            hbSysFlushMem(&input_tensor->sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
            hbSysFlushMem(&input_tensor->sysMem[1], HB_SYS_MEM_CACHE_CLEAN);
        } else if (tensor_type == HB_DNN_IMG_TYPE_Y || tensor_type == HB_DNN_IMG_TYPE_NV12) {
            memcpy(input_tensor->sysMem[0].virAddr, data_ptr, data_size);
            hbSysFlushMem(&input_tensor->sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
        } else {
            NumpyCopyHelper(input_tensor, data_ptr, data_size);
            hbSysFlushMem(&input_tensor->sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
        }
    }

    hbDNNTaskHandle_t task_handle = NULL;
    hbDNNInferCtrlParam ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&ctrl_param);

    ctrl_param.bpuCoreId = core_id; // 设置核心 ID
    ctrl_param.priority = priority; // 设置优先级

    hbDNNTensor *output = model_obj->m_outputs;

    // 执行推理
    ret = hbDNNInfer(&task_handle, &output, model_obj->m_inputs, model_obj->m_dnn_handle, &ctrl_param);
    if (ret) {
        std::cout << "hbDNNInfer failed" << std::endl;
        return -1;
    }
    // 等待任务完成
    ret = hbDNNWaitTaskDone(task_handle, 0);
    if (ret) {
        std::cout << "hbDNNWaitTaskDone failed" << std::endl;
        return -1;
    }

    // 确保 CPU 从 DDR 中读取数据之后再使用输出张量数据
    for (int32_t i = 0; i < model_obj->m_output_count; i++) {
        hbSysFlushMem(&output[i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
    }

    // 释放任务句柄
    ret = hbDNNReleaseTask(task_handle);
    if (ret) {
        std::cout << "hbDNNReleaseTask failed" << std::endl;
        return -1;
    }
    task_handle = NULL;


    return 0;
}

static PyObject *Model_forward(Model_Object *self, PyObject *args, PyObject *kwargs)
{
    PyObject *arg_obj = NULL;
    int core_id = 0;
    int priority = 0;

    // 定义参数的关键字
    static const char *keywords[] = {"arg", "core_id", "priority", NULL};

    // 解析参数
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "O|ii", const_cast<char **>(keywords), &arg_obj, &core_id, &priority)) {
        Py_RETURN_NONE;
    }

    // 将 Python 对象转换为 NumPy 数组
    PyArrayObject* arg_array = (PyArrayObject*)PyArray_FROM_OTF(arg_obj, NPY_UBYTE, NPY_ARRAY_IN_ARRAY);
    if (arg_array == NULL) {
        PyErr_SetString(PyExc_TypeError, "Input must be a NumPy array of type uint8.");
        Py_RETURN_NONE;
    }

    // 获取图像数据的指针
    unsigned char* arg_data_ptr = (unsigned char*)PyArray_DATA(arg_array);

    // 获取图像数据的形状信息
    npy_intp* arg_dims = PyArray_DIMS(arg_array);
    int arg_height = (int)arg_dims[0];
    int arg_width = (int)arg_dims[1];
    int arg_channels = (int)arg_dims[2];

    // 创建 NV12 数据数组
    int nv12_size = arg_height * arg_width; // NV12 格式的数据大小
    // unsigned char* nv12_data = (unsigned char*)malloc(nv12_size * sizeof(unsigned char));

    // 调用 forward 函数并传递处理后的参数
    // 这里假设 forward 函数接受一个 PyArrayObject* 类型的参数以及两个整数参数
    int32_t result = forward(self, arg_data_ptr, nv12_size, core_id, priority);

    // 处理 forward 函数的返回值并返回相应的结果
    if (result == -1) {
        // 处理 forward 函数执行失败的情况
        printf("arg_height=%d arg_width=%d, arg_channels=%d\n", arg_height, arg_width, arg_channels);
        Py_RETURN_NONE;
    }

    // 返回 forward 函数执行成功的情况
    return model_get_tensor_outputs(self, NULL);
}

// PyGetSetDef 定义成员属性，使用 getter 函数获取属性值
static PyGetSetDef ModelGetSet[] = {
    {"name", (getter)model_get_model_name, NULL, "Model Name", NULL},
    {"inputs", (getter)model_get_tensor_inputs, NULL, "Model Inputs", NULL},
    {"outputs", (getter)model_get_tensor_outputs, NULL, "Model Outputs", NULL},
    {"estimate_latency", (getter)model_get_estimate_latency, NULL, "Estimate latency", NULL},
    {NULL} /* Sentinel */
};

static struct PyMethodDef Model_Methods[] = {
    {"forward", (PyCFunction)Model_forward, METH_VARARGS | METH_KEYWORDS, "Run Model"},
    {NULL, NULL, 0, NULL},
};

static PyTypeObject ModelType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "dnnpy.Model",                              /* tp_name */
    sizeof(Model_Object),                          /* tp_basicsize */
    0,                                             /* tp_itemsize */
    (destructor)Model_dealloc,                     /* tp_dealloc */
    0,                                             /* tp_print */
    0,                                             /* tp_getattr */
    0,                                             /* tp_setattr */
    0,                                             /* tp_reserved */
    0,                                             /* tp_repr */
    0,                                             /* tp_as_number */
    0,                                             /* tp_as_sequence */
    0,                                             /* tp_as_mapping */
    0,                                             /* tp_hash */
    0,                                             /* tp_call */
    0,                                             /* tp_str */
    0,                                             /* tp_getattro */
    0,                                             /* tp_setattro */
    0,                                             /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT,                            /* tp_flags */
    "Model object",                                /* tp_doc */
    0,                                             /* tp_traverse */
    0,                                             /* tp_clear */
    0,                                             /* tp_richcompare */
    0,                                             /* tp_weaklistoffset */
    0,                                             /* tp_iter */
    0,                                             /* tp_iternext */
    Model_Methods,                                 /* tp_methods */
    0,                                             /* tp_members */
    ModelGetSet,                                   /* tp_getset */
    0,                                             /* tp_base */
    0,                                             /* tp_dict */
    0,                                             /* tp_descr_get */
    0,                                             /* tp_descr_set */
    0,                                             /* tp_dictoffset */
    (initproc)Model_init,                          /* tp_init */
    0,                                             /* tp_alloc */
    (newfunc)Model_new,                            /* tp_new */
    0,                                             /* tp_free */
};

// 释放模型张量资源
static void release_model_tensor(Model_Object *model_obj)
{
    if (model_obj->m_inputs != NULL) {
        // 释放输入张量数组的内存
        for (int i = 0; i < model_obj->m_input_count; ++i) {
            if (model_obj->m_inputs[i].sysMem == NULL) {
                hbSysFreeMem(model_obj->m_inputs[i].sysMem);
            }
        }
        free(model_obj->m_inputs);
        model_obj->m_inputs = NULL;
    }

    if (model_obj->m_outputs != NULL) {
        // 释放输出张量数组的内存
        for (int i = 0; i < model_obj->m_output_count; ++i) {
            if (model_obj->m_outputs[i].sysMem == NULL) {
                hbSysFreeMem(model_obj->m_outputs[i].sysMem);
            }
        }
        free(model_obj->m_outputs);
        model_obj->m_outputs = NULL;
    }
}

static int32_t prepare_model_tensor(Model_Object *model_obj)
{
    int32_t ret = 0;
    int32_t i = 0;
    hbDNNHandle_t dnn_handle = model_obj->m_dnn_handle;
    hbDNNTensorProperties properties;

    hbDNNGetInputCount(&model_obj->m_input_count, dnn_handle);
    hbDNNGetOutputCount(&model_obj->m_output_count, dnn_handle);

    // 为输入张量数组分配空间
    model_obj->m_inputs = (hbDNNTensor *)malloc(sizeof(hbDNNTensor) * model_obj->m_input_count);
    if (model_obj->m_inputs == NULL) {
        // 内存分配失败
        return -1;
    }

    for (i = 0; i < model_obj->m_input_count; ++i) {
        // alloc input tensor
        hbDNNGetInputTensorProperties(&properties, dnn_handle, i);
        model_obj->m_inputs[i].properties = properties;
        hbSysAllocCachedMem(model_obj->m_inputs[i].sysMem,
            properties.validShape.dimensionSize[0] * ALIGN_16(properties.validShape.dimensionSize[1]) *
            properties.validShape.dimensionSize[2] * ALIGN_16(properties.validShape.dimensionSize[3]));
        if (model_obj->m_inputs[i].sysMem == NULL) {
            // 内存分配失败，释放之前已分配的内存
            release_model_tensor(model_obj);
            return -1;
        }
    }

    // 为输出张量数组分配空间
    model_obj->m_outputs = (hbDNNTensor *)malloc(sizeof(hbDNNTensor) * model_obj->m_output_count);
    if (model_obj->m_outputs == NULL) {
        // 内存分配失败，释放之前已分配的内存
        release_model_tensor(model_obj);
        return -1;
    }

    for (i = 0; i < model_obj->m_output_count; ++i) {
        // alloc output tensor
        hbDNNGetOutputTensorProperties(&properties, dnn_handle, i);
        model_obj->m_outputs[i].properties = properties;
        hbSysAllocCachedMem(model_obj->m_outputs[i].sysMem, properties.alignedByteSize);
        if (model_obj->m_outputs[i].sysMem == NULL) {
            // 内存分配失败，释放之前已分配的内存
            release_model_tensor(model_obj);
            return -1;
        }
    }

    return ret;
}

Model_Object* create_and_load_model(const char* model_file) {
    // 创建一个 Model 对象
    Model_Object* model = PyObject_New(Model_Object, &ModelType);
    if (model == NULL) {
        return NULL;
    }

    // 第一步加载模型
    hbPackedDNNHandle_t packed_dnn_handle;
    if (hbDNNInitializeFromFiles(&packed_dnn_handle, &model_file, 1) != 0) {
        PyErr_SetString(PyExc_RuntimeError, "hbDNNInitializeFromFiles failed");
        Py_DECREF(model);
        return NULL;
    }

    // 第二步获取模型名称
    const char **model_name_list;
    int32_t model_count = 0;
    if (hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle) != 0) {
        PyErr_SetString(PyExc_RuntimeError, "hbDNNGetModelNameList failed");
        Py_DECREF(model);
        return NULL;
    }

    // 第三步获取 dnn_handle
    hbDNNHandle_t dnn_handle;
    if (hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name_list[0]) != 0) {
        PyErr_SetString(PyExc_RuntimeError, "hbDNNGetModelHandle failed");
        Py_DECREF(model);
        return NULL;
    }

    // 初始化 Model 对象的属性
    size_t len = strlen(model_name_list[0]);
    if (len < sizeof(model->name)) {
        memcpy(model->name, model_name_list[0], len);
        model->name[len] = '\0'; // 手动添加 null 结尾
    }
    model->m_estimate_latency = 100;  // 设置估计延迟为 100
    model->m_packed_dnn_handle = packed_dnn_handle;
    model->m_dnn_handle = dnn_handle;

    // 准备模型张量
    if (prepare_model_tensor(model) != 0) {
        PyErr_SetString(PyExc_RuntimeError, "prepare_model_tensor failed");
        Py_DECREF(model);
        return NULL;
    }

    return model;
}

static PyObject *Dnnpy_load(PyObject *self, PyObject *args, PyObject *kwargs)
{
    PyObject *model_file_arg;

    // 解析参数
    static const char *keywords[] = {"model_file", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "O", const_cast<char **>(keywords), &model_file_arg)) {
        return NULL;
    }

    // 检查传入参数的类型
    if (PyUnicode_Check(model_file_arg)) {
        // 如果参数是字符串类型，则将其转换为一个包含单个元素的列表
        PyObject *model_file_list = PyList_New(1);
        if (model_file_list == NULL) {
            PyErr_SetString(PyExc_RuntimeError, "Failed to create model file list");
            return NULL;
        }
        PyList_SET_ITEM(model_file_list, 0, model_file_arg);
        model_file_arg = model_file_list;
    } else if (!PyList_Check(model_file_arg)) {
        // 参数既不是字符串也不是列表，返回错误
        PyErr_SetString(PyExc_TypeError, "model_file must be a string or a list");
        return NULL;
    }

    // 创建一个空的模型列表
    PyObject *model_list = PyList_New(0);
    if (model_list == NULL) {
        PyErr_SetString(PyExc_RuntimeError, "Failed to create model list");
        return NULL;
    }

    // 遍历加载模型文件并创建模型对象
    Py_ssize_t num_files = PyList_Size(model_file_arg);
    for (Py_ssize_t i = 0; i < num_files; ++i) {
        PyObject *model_file_obj = PyList_GetItem(model_file_arg, i);
        if (!PyUnicode_Check(model_file_obj)) {
            PyErr_SetString(PyExc_TypeError, "model_file must be a string or a list of strings");
            Py_DECREF(model_list);
            return NULL;
        }
        const char *model_file = PyUnicode_AsUTF8(model_file_obj);
        if (model_file == NULL) {
            PyErr_SetString(PyExc_RuntimeError, "Failed to convert model file path to UTF-8");
            Py_DECREF(model_list);
            return NULL;
        }

        // 创建一个 Model 对象
        Model_Object *model = PyObject_New(Model_Object, &ModelType);
        if (model == NULL) {
            PyErr_SetString(PyExc_RuntimeError, "Failed to create Model object");
            Py_DECREF(model_list);
            return NULL;
        }

        // 创建并加载模型，省略部分代码
        model = create_and_load_model(model_file);
        if (model == NULL) {
            Py_DECREF(model_list);
            return NULL;
        }

        // 将 Model 对象添加到模型列表中
        if (PyList_Append(model_list, (PyObject *)model) != 0) {
            PyErr_SetString(PyExc_RuntimeError, "Failed to append Model object to model list");
            Py_DECREF(model);
            Py_DECREF(model_list);
            return NULL;
        }
    }

    // 返回模型列表
    return model_list;
}

static PyMethodDef dnnpy_methods[] = {
    {"load", (PyCFunction)Dnnpy_load, METH_VARARGS | METH_KEYWORDS, "Load model"},
    {NULL, NULL, 0, NULL},
};

/// wrap a module structure
static struct PyModuleDef dnnpy = {
    PyModuleDef_HEAD_INIT,
    "dnnpy",  /* name of module */
    "Deep Neural Network inference engine", /* module documentation, may be NULL */
    -1,            /* size of per-interpreter state of the module, or -1 if the module keeps state in global variables. */
    dnnpy_methods,
};

static PyObject* init_module(PyObject* m) {
    // 初始化 NumPy 库
    import_array();

    if (m == NULL) {
        return NULL;
    }

    PyVarObject ob_base = {1, &PyType_Type, 0};
    ModelType.ob_base = ob_base;
    PyDNNTensorType.ob_base = ob_base;
    TensorPropertiesType.ob_base = ob_base;

    if (PyType_Ready(&ModelType) < 0) {
        Py_INCREF(&ModelType);
        return NULL;
    }

    if (PyType_Ready(&PyDNNTensorType) < 0) {
        Py_INCREF(&PyDNNTensorType);
        return NULL;
    }

    if (PyType_Ready(&TensorPropertiesType) < 0) {
        Py_INCREF(&TensorPropertiesType);
        return NULL;
    }

    PyModule_AddObject(m, "Model", (PyObject*)&ModelType);
    PyModule_AddObject(m, "pyDNNTensor", (PyObject*)&PyDNNTensorType);
    PyModule_AddObject(m, "TensorProperties", (PyObject*)&TensorPropertiesType);

    return m;
}

/// init the module is a py module
PyMODINIT_FUNC PyInit_dnnpy(void)
{
    PyObject *m;
    m = PyModule_Create(&dnnpy);
    return init_module(m);
}

/// init the module is a py module
PyMODINIT_FUNC PyInit_pyeasy_dnn(void)
{
    PyObject *m;
    m = PyModule_Create(&dnnpy);
    return init_module(m);
}

#ifdef __cplusplus
}
#endif /* extern "C" */
