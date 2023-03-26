/*
 * Horizon Robotics
 *
 * Copyright (C) 2022 Horizon Robotics Inc.
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

#include <functional>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Python.h>

#include "x3_sdk_python.h"
#include "x3_sdk_display.h"
#include "x3_sdk_camera.h"
#include "x3_sdk_codec.h"

using namespace srpy_cam;
using namespace std;

#ifdef __cplusplus
extern "C" {
#endif

static void dump_frame_info(ImageFrame &image_frame)
{
    printf("image_id: %ld, lost_image_num: %ld, image_timestamp: %ld, data_size: %u, exp_time: %ld\n",
           image_frame.image_id, image_frame.lost_image_num, image_frame.image_timestamp,
           image_frame.data_size[0] + image_frame.data_size[1], image_frame.exp_time);
}

static int __save_fame(ImageFrame &image_frame, string tail)
{
    dump_frame_info(image_frame);
    string pic_name{
        "/userdata/image_" + to_string(image_frame.image_id) + "_" +
        to_string(image_frame.image_timestamp) + "_" +
        to_string(image_frame.data_size[0] + image_frame.data_size[1]) + tail};

    ofstream ofs;
    ofs.open(pic_name, ofstream::binary);
    if (ofs.is_open()) {
        ofs.write(reinterpret_cast<char *>(image_frame.data[0]), image_frame.data_size[0]);
        if (image_frame.plane_count > 1) {
            ofs.write(reinterpret_cast<char *>(image_frame.data[1]), image_frame.data_size[1]);
        }
        ofs.close();
    }

    return 0;
}

int save_raw_frame_data(ImageFrame &image_frame)
{
    return __save_fame(image_frame, ".raw");
}

int save_yuv_frame_data(ImageFrame &image_frame)
{
    return __save_fame(image_frame, ".yuv");
}

static int py_obj_to_array(PyObject *obj, int *array)
{
    int num = 0;

    if (obj == NULL) {
        num = 0;
    } else if (PyLong_Check(obj)) {
        num = 1;
        array[0] = PyLong_AsLong(obj);
    } else if (PyList_Check(obj)) {
        num = PyList_Size(obj);
        for (int i = 0; i < num; i++) {
            array[i] = PyLong_AsLong(PyList_GetItem(obj, i));
        }
    }

    return num;
}

static int py_obj_to_size(PyObject *obj, int *width, int *height)
{
    int num = 0;

    if (obj == NULL) {
        num = 0;
    } else if (PyLong_Check(obj)) {
        num = 0;
    } else if (PyList_Check(obj)) {
        num = PyList_Size(obj);
        if (PyLong_Check(PyList_GetItem(obj, 0))) {
            if (PyList_Size(obj) < 2) {
                return -1;
            }
            num = 1;
            width[0] = PyLong_AsLong(PyList_GetItem(obj, 0));
            height[0] = PyLong_AsLong(PyList_GetItem(obj, 1));
        } else if (PyList_Check(PyList_GetItem(obj, 0))) {
            for (int i = 0; i < num; i++) {
                if (PyList_Size(PyList_GetItem(obj, i)) < 2) {
                    return -1;
                }
                width[i] = PyLong_AsLong(PyList_GetItem(PyList_GetItem(obj, i), 0));
                height[i] = PyLong_AsLong(PyList_GetItem(PyList_GetItem(obj, i), 1));
            }
        }
    }

    return num;
}

static int py_obj_to_rect(PyObject *obj, int *x, int *y, int *width, int *height)
{
    int num = 0;

    if (obj == NULL) {
        num = 0;
    } else if (PyList_Check(obj)) {
        num = PyList_Size(obj);
        if (PyLong_Check(PyList_GetItem(obj, 0))) {
            if (PyList_Size(obj) < 4) {
                return -1;
            }
            num = 1;
            x[0] = PyLong_AsLong(PyList_GetItem(obj, 0));
            y[0] = PyLong_AsLong(PyList_GetItem(obj, 1));
            width[0] = PyLong_AsLong(PyList_GetItem(obj, 2));
            height[0] = PyLong_AsLong(PyList_GetItem(obj, 3));
        } else if (PyList_Check(PyList_GetItem(obj, 0))) {
            for (int i = 0; i < num; i++) {
                if (PyList_Size(PyList_GetItem(obj, i)) < 4) {
                    return -1;
                }
                x[i] = PyLong_AsLong(PyList_GetItem(PyList_GetItem(obj, i), 0));
                y[i] = PyLong_AsLong(PyList_GetItem(PyList_GetItem(obj, i), 1));
                width[i] = PyLong_AsLong(PyList_GetItem(PyList_GetItem(obj, i), 2));
                height[i] = PyLong_AsLong(PyList_GetItem(PyList_GetItem(obj, i), 3));
            }
        }
    }

    return num;
}

static PyObject *Camera_new(PyTypeObject *type, PyObject *args, PyObject *kw)
{
    libsppydev_Object *self = (libsppydev_Object *)type->tp_alloc(type, 0);
    self->pobj = nullptr;
    return (PyObject *)self;
}

static void Camera_dealloc(libsppydev_Object *self)
{
    if (self->pobj) {
        delete (SrPyCamera *)self->pobj;
        self->pobj = nullptr;
    }

    if (self->pframe) {
        delete (ImageFrame *)self->pframe;
        self->pframe = nullptr;
    }
    self->ob_base.ob_type->tp_free(self);
}

static int Camera_init(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    if (self->pobj) {
        PyErr_SetString(PyExc_Exception, "__init__ already called");
        return -1;
    }
    self->pobj = new SrPyCamera();
    self->pframe = new ImageFrame();
    self->object = SrPy_Camera;

    return 0;
}

PyObject *Camera_open_cam(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    if (!(self->pobj && self->pframe)) {
        PyErr_SetString(PyExc_Exception, "camera not inited");
        return Py_BuildValue("i", -1);
    }

    int pipe_id, video_index, fps = 30, chn_num = 0;
    int width[CAMERA_CHN_NUM], height[CAMERA_CHN_NUM];
    PyObject *width_obj = NULL, *height_obj = NULL, *size_obj = NULL;
    SrPyCamera *cam = (SrPyCamera *)self->pobj;
    static char *kwlist[] = {(char *)"pipe_id", (char *)"video_index",
        (char *)"fps", (char *)"width", (char *)"height", (char *)"size", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kw, "ii|iOOO", kwlist, &pipe_id, &video_index, &fps,
        &width_obj, &height_obj, &size_obj))
        return Py_BuildValue("i", -1);

    if (size_obj != NULL) {
        chn_num = py_obj_to_size(size_obj, width, height);
    } else {
        chn_num = py_obj_to_array(width_obj, width);
        chn_num = py_obj_to_array(height_obj, height);
    }
    if (chn_num < 0) {
        PRINT("Invalid param\n");
        return Py_BuildValue("i", -1);
    }
    if (chn_num < (CAMERA_CHN_NUM - 1)) {
        // set 0 means default size
        width[chn_num] = 0;
        height[chn_num] = 0;
        chn_num++;
    }

    return Py_BuildValue("i", cam->OpenCamera(pipe_id, video_index, fps, chn_num, width, height));
}

PyObject *Camera_open_vps(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    if (!(self->pobj && self->pframe)) {
        PyErr_SetString(PyExc_Exception, "camera not inited");
        return Py_BuildValue("i", -1);
    }

    int pipe_id = 1, proc_mode = 0, src_width = 1920, src_height = 1080, chn_num = 0;
    int rotate[CAMERA_CHN_NUM] = {0};
    int dst_width[CAMERA_CHN_NUM] = {0}, dst_height[CAMERA_CHN_NUM] = {0};
    int crop_x[CAMERA_CHN_NUM] = {0}, crop_y[CAMERA_CHN_NUM] = {0};
    int crop_width[CAMERA_CHN_NUM] = {0}, crop_height[CAMERA_CHN_NUM] = {0};
    PyObject *dst_width_obj = NULL, *dst_height_obj = NULL, *rotate_obj = NULL;
    PyObject *crop_rect_obj = NULL, *src_size_obj = NULL, *dst_size_obj = NULL;
    SrPyCamera *cam = (SrPyCamera *)self->pobj;
    static char *kwlist[] = {(char *)"pipe_id", (char *)"process_mode",
        (char *)"src_width", (char *)"src_height",
        (char *)"dst_width", (char *)"dst_height",
        (char *)"crop_rect", (char *)"rotate",
        (char *)"src_size", (char *)"dst_size", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kw, "|iiiiOOOOOO", kwlist, &pipe_id, &proc_mode,
        &src_width, &src_height, &dst_width_obj, &dst_height_obj,
        &crop_rect_obj, &rotate_obj, &src_size_obj, &dst_size_obj))
        return Py_BuildValue("i", -1);

    if (proc_mode < 0) {
        PRINT("Invalid param\n");
        return Py_BuildValue("i", -1);
    }

    if (src_size_obj != NULL) {
        py_obj_to_size(src_size_obj, &src_width, &src_height);
    }
    if (dst_size_obj != NULL) {
        chn_num = py_obj_to_size(dst_size_obj, dst_width, dst_height);
    } else {
        chn_num = py_obj_to_array(dst_width_obj, dst_width);
        chn_num = py_obj_to_array(dst_height_obj, dst_height);
    }
    if (chn_num < (CAMERA_CHN_NUM - 1)) {
        // set 0 means default size
        dst_width[chn_num] = 0;
        dst_height[chn_num] = 0;
        chn_num++;
    }
    if ((proc_mode == VPS_SCALE_CROP) || (proc_mode == VPS_SCALE_ROTATE_CROP)) {
        py_obj_to_rect(crop_rect_obj, crop_x, crop_y, crop_width, crop_height);
    }
    if ((proc_mode == VPS_SCALE_ROTATE) || (proc_mode == VPS_SCALE_ROTATE_CROP)) {
        py_obj_to_array(rotate_obj, rotate);
    }

    if (chn_num < 0) {
        PRINT("Invalid param\n");
        return Py_BuildValue("i", -1);
    }

    return Py_BuildValue("i", cam->OpenVPS(pipe_id, chn_num, proc_mode, src_width, src_height,
        dst_width, dst_height, crop_x, crop_y, crop_width, crop_height, rotate));
}

PyObject *Camera_close_cam(libsppydev_Object *self)
{
    if (!(self->pobj && self->pframe)) {
        PyErr_SetString(PyExc_Exception, "camera not inited");
        Py_RETURN_NONE;
    }

    SrPyCamera *cam = (SrPyCamera *)self->pobj;

    cam->CloseCamera();

    Py_RETURN_NONE;
}

PyObject *Camera_get_frame(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    if (!(self->pobj && self->pframe)) {
        PyErr_SetString(PyExc_Exception, "camera not inited");
        Py_RETURN_NONE;
    }

    DevModule module = Dev_IPU;
    int width = 0, height = 0;
    SrPyCamera *cam = (SrPyCamera *)self->pobj;
    PyObject *img_obj = nullptr, *uv_obj = nullptr;
    static char *kwlist[] = {(char *)"module", (char *)"width", (char *)"height", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kw, "|iii", kwlist, &module, &width, &height))
        Py_RETURN_NONE;

    if (!cam->GetImageFrame(self->pframe, module, width, height, 2000)) {
        img_obj = PyBytes_FromStringAndSize((const char *)self->pframe->data[0],
            self->pframe->data_size[0]);

        if (self->pframe->plane_count > 1) {
            uv_obj = PyBytes_FromStringAndSize((const char *)self->pframe->data[1],
                self->pframe->data_size[1]);
            PyBytes_ConcatAndDel(&img_obj, uv_obj);
        }

        cam->ReturnImageFrame(self->pframe, module, width, height);

        return img_obj;
    }

    Py_RETURN_NONE;
}

PyObject *Camera_set_frame(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    if (!(self->pobj && self->pframe)) {
        PyErr_SetString(PyExc_Exception, "camera not inited");
        Py_RETURN_NONE;
    }

    DevModule module = Dev_IPU;
    SrPyCamera *cam = (SrPyCamera *)self->pobj;
    PyObject *img_obj = nullptr;
    static char *kwlist[] = {(char *)"img_obj", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kw, "|O", kwlist, &img_obj))
        Py_RETURN_NONE;

    self->pframe->data[0] = (uint8_t *)PyBytes_AsString(img_obj);
    self->pframe->data_size[0] = PyBytes_Size(img_obj);

    return Py_BuildValue("i", cam->SetImageFrame(self->pframe, module));
}

/// encode related

static PyObject *Encoder_new(PyTypeObject *type, PyObject *args, PyObject *kw)
{
    libsppydev_Object *self = (libsppydev_Object *)type->tp_alloc(type, 0);
    self->pobj = nullptr;
    return (PyObject *)self;
}

static void Encoder_dealloc(libsppydev_Object *self)
{
    if (self->pobj) {
        delete static_cast<SrPyEncode *>(self->pobj);
        self->pobj = nullptr;
    }

    self->ob_base.ob_type->tp_free(self);
}

static int Encoder_init(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    if (self->pobj) {
        PyErr_SetString(PyExc_Exception, "__init__ already called");
        return -1;
    }
    self->pobj = static_cast<void *>(new SrPyEncode());
    self->object = SrPy_Encode;

    return 0;
}

static PyObject *Encoder_encode(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    if (!self->pobj) {
        PyErr_SetString(PyExc_Exception, "encoder not inited");
        return Py_BuildValue("i", -1);
    }

    int venc_chn = 0, type = 0, width = 0, height = 0, bits = 8000;
    static char *kwlist[] = {(char *)"video_chn", (char *)"type",
        (char *)"width", (char *)"height", (char *)"bits", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kw, "iiii|i", kwlist, &venc_chn, &type, &width, &height, &bits)) {
        return Py_BuildValue("i", -1);
    }

    SrPyEncode *pobj = static_cast<SrPyEncode *>(self->pobj);

    return Py_BuildValue("i", pobj->do_encoding(venc_chn, type, width, height, bits));
}

static PyObject *Encoder_send_frame(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    if (!self->pobj) {
        PyErr_SetString(PyExc_Exception, "encoder not inited");
        return Py_BuildValue("i", -1);
    }

    char *addr = nullptr;
    int32_t size = 0;
    PyObject *img_obj = nullptr;
    SrPyEncode *pobj = static_cast<SrPyEncode *>(self->pobj);
    static char *kwlist[] = {(char *)"img", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kw, "O", kwlist, &img_obj)) {
        return Py_BuildValue("i", -1);
    }

    addr = PyBytes_AsString(img_obj);
    size = PyBytes_Size(img_obj);

    return Py_BuildValue("i", pobj->send_frame(addr, size));
}

static PyObject *Encoder_get_frame(libsppydev_Object *self)
{
    if (!self->pobj) {
        PyErr_SetString(PyExc_Exception, "encoder not inited");
        Py_RETURN_NONE;
    }

    SrPyEncode *pobj = static_cast<SrPyEncode *>(self->pobj);
    PyObject *img_obj = nullptr;

    self->pframe = pobj->get_frame();
    if (self->pframe) {
        img_obj = PyBytes_FromStringAndSize((char *)self->pframe->data[0],
            self->pframe->data_size[0]);

        pobj->put_frame(self->pframe);

        return img_obj;
    }

    Py_RETURN_NONE;
}

static PyObject *Encoder_close(libsppydev_Object *self)
{
    if (!self->pobj) {
        PyErr_SetString(PyExc_Exception, "encoder not inited");
        return Py_BuildValue("i", -1);
    }

    SrPyEncode *pobj = static_cast<SrPyEncode *>(self->pobj);

    return Py_BuildValue("i", pobj->undo_encoding());
}

/// Decode related

static PyObject *Decoder_new(PyTypeObject *type, PyObject *args, PyObject *kw)
{
    libsppydev_Object *self = (libsppydev_Object *)type->tp_alloc(type, 0);
    self->pobj = nullptr;
    return (PyObject *)self;
}

static void Decoder_dealloc(libsppydev_Object *self)
{
    if (self->pobj) {
        delete static_cast<SrPyDecode *>(self->pobj);
        self->pobj = nullptr;
    }

    self->ob_base.ob_type->tp_free(self);
}

static int Decoder_init(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    if (self->pobj) {
        PyErr_SetString(PyExc_Exception, "__init__ already called");
        return -1;
    }

    self->pobj = static_cast<void *>(new SrPyDecode());
    self->object = SrPy_Decode;

    return 0;
}

static PyObject *Decoder_decode(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    if (!self->pobj) {
        PyErr_SetString(PyExc_Exception, "decoder not inited");
        return Py_BuildValue("i", -1);
    }

    /// Use the static args, for optional args using
    static char *string = nullptr;
    static int video_chn = 0, type = 1, width = 1920, height = 1080, dec_mode = 1;
    int frame_count = 0;
    int ret = 0;
    PyObject *list = nullptr;
    static char *kwlist[] = {(char *)"file", (char *)"video_chn", (char *)"type",
        (char *)"width", (char *)"height", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kw, "s|iiii", kwlist,
                          &string, &video_chn, &type, &width, &height)) {
        return Py_BuildValue("i", -1);
    }

    SrPyDecode *pobj = static_cast<SrPyDecode *>(self->pobj);

    ret = pobj->do_decoding(string, video_chn, type, width, height, &frame_count, dec_mode);

    list = PyList_New(0);
    PyList_Append(list, Py_BuildValue("i", ret));
    PyList_Append(list, Py_BuildValue("i", frame_count));

    return list;
}

static PyObject *Decoder_get_frame(libsppydev_Object *self)
{
    if (!self->pobj) {
        PyErr_SetString(PyExc_Exception, "decoder not inited");
        return Py_BuildValue("i", -1);
    }

    SrPyDecode *pobj = static_cast<SrPyDecode *>(self->pobj);
    PyObject *img_obj = nullptr, *uv_obj = nullptr;

    self->pframe = pobj->get_frame();
    if (self->pframe) {
        img_obj = PyBytes_FromStringAndSize((const char *)self->pframe->data[0],
            self->pframe->data_size[0]);
        uv_obj = PyBytes_FromStringAndSize((const char *)self->pframe->data[1],
            self->pframe->data_size[1]);
        PyBytes_ConcatAndDel(&img_obj, uv_obj);

        pobj->put_frame(self->pframe);

        return img_obj;
    }

    Py_RETURN_NONE;
}

static PyObject *Decoder_send_frame(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    PyObject *img_obj = nullptr;
    libsppydev_Object *pobj = nullptr;
    char *addr = nullptr;
    int size = 0;
    int chn = -1;
    int eos = 0;
    static char *kwlist[] = {(char *)"img", (char *)"chn", (char *)"eos", NULL};

    if (!self->pobj) {
        PyErr_SetString(PyExc_Exception, "decode not inited");
        return Py_BuildValue("i", -1);
    }
    if (!PyArg_ParseTupleAndKeywords(args, kw, "O|ii", kwlist, &img_obj, &chn, &eos)) {
        return Py_BuildValue("i", -1);
    }

    pobj = (libsppydev_Object *)self->pobj;
    addr = PyBytes_AsString(img_obj);
    size = PyBytes_Size(img_obj);

    if (chn < 0) {
        chn = ((SrPyDecode *)pobj)->m_chn;
    }

    return Py_BuildValue("i", ((SrPyDecode *)pobj)->send_frame(chn, addr, size, eos));
}

static PyObject *Decoder_close(libsppydev_Object *self)
{
    if (!self->pobj) {
        PyErr_SetString(PyExc_Exception, "decoder not inited");
        return Py_BuildValue("i", -1);
    }

    SrPyDecode *pobj = static_cast<SrPyDecode *>(self->pobj);

    return Py_BuildValue("i", pobj->undo_decoding());
}


/// Display related

static PyObject *Display_new(PyTypeObject *type, PyObject *args, PyObject *kw)
{
    libsppydev_Object *self = (libsppydev_Object *)type->tp_alloc(type, 0);
    self->pobj = nullptr;
    return (PyObject *)self;
}

static void Display_dealloc(libsppydev_Object *self)
{
    if (self->pobj) {
        delete static_cast<SrPyDisplay *>(self->pobj);
        self->pobj = nullptr;
    }

    self->ob_base.ob_type->tp_free(self);
}

static int Display_init(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    if (self->pobj) {
        PyErr_SetString(PyExc_Exception, "__init__ already called");
        return -1;
    }

    self->pobj = static_cast<void *>(new SrPyDisplay());

    self->object = SrPy_Display;

    return 0;
}

static PyObject *Display_display(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    int width = 1920, height = 1080, vot_chn = 0, chn_width = 0, chn_height = 0;
    int vot_intf = 0, vot_out_mode = 1;
    static char *kwlist[] = {(char *)"chn", (char *)"width", (char *)"height",
        (char *)"vot_intf", (char *)"vot_out_mode",
        (char *)"chn_width", (char *)"chn_height", NULL};

    if (!self->pobj) {
        PyErr_SetString(PyExc_Exception, "display not inited");
        return Py_BuildValue("i", -1);
    }
    if (!PyArg_ParseTupleAndKeywords(args, kw, "|iiiiiii", kwlist, &vot_chn, &width, &height, &vot_intf, &vot_out_mode, &chn_width, &chn_height)) {
        return Py_BuildValue("i", -1);
    }

    if ((chn_width == 0) || (chn_height == 0)) {
        chn_width = width;
        chn_height = height;
    }

    return Py_BuildValue("i", ((SrPyDisplay *)(self->pobj))->x3_vot_init(vot_chn, width, height, vot_intf, vot_out_mode, chn_width, chn_height));
}

static PyObject *Display_set_img(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    PyObject *img_obj = nullptr;
    libsppydev_Object *pobj = nullptr;
    char *addr = nullptr;
    int size = 0;
    int chn = 0;
    static char *kwlist[] = {(char *)"img", (char *)"chn", NULL};

    if (!self->pobj) {
        PyErr_SetString(PyExc_Exception, "display not inited");
        return Py_BuildValue("i", -1);
    }
    if (!PyArg_ParseTupleAndKeywords(args, kw, "O|i", kwlist, &img_obj, &chn)) {
        return Py_BuildValue("i", -1);
    }

    pobj = (libsppydev_Object *)self->pobj;
    addr = PyBytes_AsString(img_obj);
    size = PyBytes_Size(img_obj);

    return Py_BuildValue("i", ((SrPyDisplay *)pobj)->set_img(addr, size, chn));
}

static PyObject *Display_set_graph_rect(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    libsppydev_Object *pobj = nullptr;
    int x0, y0, x1, y1, chn = 2, flush = 0, line_width = 4;
    uint64_t color = 0xffff0000;
    static char *kwlist[] = {(char *)"x0", (char *)"y0", (char *)"x1", (char *)"y1",
        (char *)"chn", (char *)"flush", (char *)"color", (char *)"line_width", NULL};

    if (!self->pobj) {
        PyErr_SetString(PyExc_Exception, "display not inited");
        return Py_BuildValue("i", -1);
    }
    if (!PyArg_ParseTupleAndKeywords(args, kw, "iiii|iili", kwlist, &x0, &y0, &x1, &y1, &chn, &flush, &color, &line_width)) {
        return Py_BuildValue("i", -1);
    }

    pobj = (libsppydev_Object *)self->pobj;

    return Py_BuildValue("i", ((SrPyDisplay *)pobj)->set_graph_rect(x0, y0, x1, y1, chn, flush, (uint32_t)color, line_width));
}

static PyObject *Display_set_graph_word(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    libsppydev_Object *pobj = nullptr;
    int x, y, chn = 2, flush = 0, line_width = 1;
    uint64_t color = 0xffff0000;
    PyObject *str_obj = nullptr;
    static char *kwlist[] = {(char *)"x", (char *)"y", (char *)"str",
        (char *)"chn", (char *)"flush", (char *)"color", (char *)"line_width", NULL};

    if (!self->pobj) {
        PyErr_SetString(PyExc_Exception, "display not inited");
        return Py_BuildValue("i", -1);
    }
    if (!PyArg_ParseTupleAndKeywords(args, kw, "iiO|iili", kwlist, &x, &y, &str_obj, &chn, &flush, &color, &line_width)) {
        return Py_BuildValue("i", -1);
    }

    pobj = (libsppydev_Object *)self->pobj;

    return Py_BuildValue("i", ((SrPyDisplay *)pobj)->set_graph_word(x, y, PyBytes_AsString(str_obj), chn, flush, (uint32_t)color, line_width));
}

static PyObject *Display_close(libsppydev_Object *self)
{
    if (!self->pobj) {
        PyErr_SetString(PyExc_Exception, "display not inited");
        return Py_BuildValue("i", -1);
    }

    return Py_BuildValue("i", ((SrPyDisplay *)self->pobj)->x3_vot_deinit());
}

static PyObject *Module_bind(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    int ret = 0;

    return Py_BuildValue("i", ret);
}

static PyObject *Module_unbind(libsppydev_Object *self, PyObject *args, PyObject *kw)
{
    int ret = 0;

    return Py_BuildValue("i", ret);
}

#define M_DOC_STRING            \
    "bind(module, module)\n"                  \
    "unbind(module, module)\n"

static const char *__g_m_doc_str = M_DOC_STRING;

static struct PyMethodDef Camera_Methods[] = {
    {"open_cam", (PyCFunction)Camera_open_cam, METH_VARARGS | METH_KEYWORDS, "Open camera and start video stream"},
    {"open_vps", (PyCFunction)Camera_open_vps, METH_VARARGS | METH_KEYWORDS, "Open vps process"},
    {"close_cam", (PyCFunction)Camera_close_cam, METH_NOARGS, "Stop video stream and close camera"},
    {"get_frame", (PyCFunction)Camera_get_frame, METH_VARARGS | METH_KEYWORDS, "Get image from the channel"},
    {"set_frame", (PyCFunction)Camera_set_frame, METH_VARARGS | METH_KEYWORDS, "Set image to the vps"},
    {nullptr, nullptr, 0, nullptr},
};

static PyTypeObject libsppydev_CameraType = {
    PyVarObject_HEAD_INIT(&libsppydev_CameraType, 0) /* ob_size */
    "libsppydev.Camera",                             /* tp_name */
    sizeof(libsppydev_Object),                       /* tp_basicsize */
    0,                                               /* tp_itemsize */
    (destructor)Camera_dealloc,                      /* tp_dealloc */
    0,                                               /* tp_print */
    0,                                               /* tp_getattr */
    0,                                               /* tp_setattr */
    0,                                               /* tp_compare */
    0,                                               /* tp_repr */
    0,                                               /* tp_as_number */
    0,                                               /* tp_as_sequence */
    0,                                               /* tp_as_mapping */
    0,                                               /* tp_hash */
    0,                                               /* tp_call */
    0,                                               /* tp_str */
    0,                                               /* tp_getattro */
    0,                                               /* tp_setattro */
    0,                                               /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT,                              /* tp_flags */
    "My first camera object.",                       /* tp_doc */
    0,                                               /* tp_traverse */
    0,                                               /* tp_clear */
    0,                                               /* tp_richcompare */
    0,                                               /* tp_weaklistoffset */
    0,                                               /* tp_iter */
    0,                                               /* tp_iternext */
    Camera_Methods,                                  /* tp_methods */
    0,                                               /* tp_members */
    0,                                               /* tp_getset */
    0,                                               /* tp_base */
    0,                                               /* tp_dict */
    0,                                               /* tp_descr_get */
    0,                                               /* tp_descr_set */
    0,                                               /* tp_dictoffset */
    (initproc)Camera_init,                           /* tp_init */
    0,                                               /* tp_alloc */
    (newfunc)Camera_new,                             /* tp_new */
    0,                                               /* tp_free */
};

static PyMethodDef Encoder_methods[] = {
    {"encode", (PyCFunction)Encoder_encode, METH_VARARGS | METH_KEYWORDS, "Start encoder"},
    {"close", (PyCFunction)Encoder_close, METH_NOARGS, "Closes encoder."},
    {"send_frame", (PyCFunction)Encoder_send_frame, METH_VARARGS | METH_KEYWORDS, "Send frame to encoder"},
    {"get_frame", (PyCFunction)Encoder_get_frame, METH_NOARGS, "Get stream from encoder."},
    {nullptr, nullptr, 0, nullptr},
};

static PyTypeObject libsppydev_EncoderType = {
    PyVarObject_HEAD_INIT(&libsppydev_EncoderType, 0) /* ob_size */
    "libsppydev.Encoder",                             /* tp_name */
    sizeof(libsppydev_Object),                        /* tp_basicsize */
    0,                                                /* tp_itemsize */
    (destructor)Encoder_dealloc,                      /* tp_dealloc */
    0,                                                /* tp_print */
    0,                                                /* tp_getattr */
    0,                                                /* tp_setattr */
    0,                                                /* tp_compare */
    0,                                                /* tp_repr */
    0,                                                /* tp_as_number */
    0,                                                /* tp_as_sequence */
    0,                                                /* tp_as_mapping */
    0,                                                /* tp_hash */
    0,                                                /* tp_call */
    0,                                                /* tp_str */
    0,                                                /* tp_getattro */
    0,                                                /* tp_setattro */
    0,                                                /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT,                               /* tp_flags */
    "My first encoder object.",                       /* tp_doc */
    0,                                                /* tp_traverse */
    0,                                                /* tp_clear */
    0,                                                /* tp_richcompare */
    0,                                                /* tp_weaklistoffset */
    0,                                                /* tp_iter */
    0,                                                /* tp_iternext */
    Encoder_methods,                                  /* tp_methods */
    0,                                                /* tp_members */
    0,                                                /* tp_getset */
    0,                                                /* tp_base */
    0,                                                /* tp_dict */
    0,                                                /* tp_descr_get */
    0,                                                /* tp_descr_set */
    0,                                                /* tp_dictoffset */
    (initproc)Encoder_init,                           /* tp_init */
    0,                                                /* tp_alloc */
    (newfunc)Encoder_new,                             /* tp_new */
    0,                                                /* tp_free */
};

static PyMethodDef Decoder_methods[] = {
    {"decode", (PyCFunction)Decoder_decode, METH_VARARGS | METH_KEYWORDS, "Start decoder"},
    {"close", (PyCFunction)Decoder_close, METH_NOARGS, "Closes decoder."},
    {"get_frame", (PyCFunction)Decoder_get_frame, METH_NOARGS, "Get image from decoder."},
    {"send_frame", (PyCFunction)Decoder_send_frame, METH_VARARGS | METH_KEYWORDS, "Set buffer to decoder."},
    {nullptr, nullptr, 0, nullptr},
};

static PyTypeObject libsppydev_DecoderType = {
    PyVarObject_HEAD_INIT(&libsppydev_DecoderType, 0) /* ob_size */
    "libsppydev.Decoder",                             /* tp_name */
    sizeof(libsppydev_Object),                        /* tp_basicsize */
    0,                                                /* tp_itemsize */
    (destructor)Decoder_dealloc,                      /* tp_dealloc */
    0,                                                /* tp_print */
    0,                                                /* tp_getattr */
    0,                                                /* tp_setattr */
    0,                                                /* tp_compare */
    0,                                                /* tp_repr */
    0,                                                /* tp_as_number */
    0,                                                /* tp_as_sequence */
    0,                                                /* tp_as_mapping */
    0,                                                /* tp_hash */
    0,                                                /* tp_call */
    0,                                                /* tp_str */
    0,                                                /* tp_getattro */
    0,                                                /* tp_setattro */
    0,                                                /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT,                               /* tp_flags */
    "My first decoder object.",                       /* tp_doc */
    0,                                                /* tp_traverse */
    0,                                                /* tp_clear */
    0,                                                /* tp_richcompare */
    0,                                                /* tp_weaklistoffset */
    0,                                                /* tp_iter */
    0,                                                /* tp_iternext */
    Decoder_methods,                                  /* tp_methods */
    0,                                                /* tp_members */
    0,                                                /* tp_getset */
    0,                                                /* tp_base */
    0,                                                /* tp_dict */
    0,                                                /* tp_descr_get */
    0,                                                /* tp_descr_set */
    0,                                                /* tp_dictoffset */
    (initproc)Decoder_init,                           /* tp_init */
    0,                                                /* tp_alloc */
    (newfunc)Decoder_new,                             /* tp_new */
    0,                                                /* tp_free */
};

static PyMethodDef Display_methods[] = {
    {"display", (PyCFunction)Display_display, METH_VARARGS | METH_KEYWORDS, "Init display"},
    {"set_img", (PyCFunction)Display_set_img, METH_VARARGS | METH_KEYWORDS, "Set display image"},
    {"set_graph_rect", (PyCFunction)Display_set_graph_rect, METH_VARARGS | METH_KEYWORDS, "Set display grapth rect"},
    {"set_graph_word", (PyCFunction)Display_set_graph_word, METH_VARARGS | METH_KEYWORDS, "Set display grapth word"},
    {"close", (PyCFunction)Display_close, METH_NOARGS, "Closes Display."},
    {nullptr, nullptr, 0, nullptr},
};

static PyTypeObject libsppydev_DisplayType = {
    PyVarObject_HEAD_INIT(&libsppydev_DisplayType, 0) /* ob_size */
    "libsppydev.Display",                             /* tp_name */
    sizeof(libsppydev_Object),                        /* tp_basicsize */
    0,                                                /* tp_itemsize */
    (destructor)Display_dealloc,                      /* tp_dealloc */
    0,                                                /* tp_print */
    0,                                                /* tp_getattr */
    0,                                                /* tp_setattr */
    0,                                                /* tp_compare */
    0,                                                /* tp_repr */
    0,                                                /* tp_as_number */
    0,                                                /* tp_as_sequence */
    0,                                                /* tp_as_mapping */
    0,                                                /* tp_hash */
    0,                                                /* tp_call */
    0,                                                /* tp_str */
    0,                                                /* tp_getattro */
    0,                                                /* tp_setattro */
    0,                                                /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT,                               /* tp_flags */
    "My first display object.",                       /* tp_doc */
    0,                                                /* tp_traverse */
    0,                                                /* tp_clear */
    0,                                                /* tp_richcompare */
    0,                                                /* tp_weaklistoffset */
    0,                                                /* tp_iter */
    0,                                                /* tp_iternext */
    Display_methods,                                  /* tp_methods */
    0,                                                /* tp_members */
    0,                                                /* tp_getset */
    0,                                                /* tp_base */
    0,                                                /* tp_dict */
    0,                                                /* tp_descr_get */
    0,                                                /* tp_descr_set */
    0,                                                /* tp_dictoffset */
    (initproc)Display_init,                           /* tp_init */
    0,                                                /* tp_alloc */
    (newfunc)Display_new,                             /* tp_new */
    0,                                                /* tp_free */
};

static PyMethodDef libsppydev_methods[] = {
    {"bind", (PyCFunction)Module_bind, METH_VARARGS | METH_KEYWORDS, "Bind two module."},
    {"unbind", (PyCFunction)Module_unbind, METH_VARARGS | METH_KEYWORDS, "Unbind two module."},
    {nullptr, nullptr, 0, nullptr},
};

/// wrap a module structure
static struct PyModuleDef libsppydev = {
    PyModuleDef_HEAD_INIT,
    "libsppydev",  /* name of module */
    __g_m_doc_str, /* module documentation, may be nullptr */
    -1,            /* size of per-interpreter state of the module, or -1 if the module keeps state in global variables. */
    libsppydev_methods,
};

/// init the module is a py module
PyMODINIT_FUNC PyInit_libsppydev(void)
{
    PyObject *m;

    m = PyModule_Create(&libsppydev);

    PyVarObject ob_base = {1, &PyType_Type, 0};
    libsppydev_CameraType.ob_base = ob_base;
    libsppydev_EncoderType.ob_base = ob_base;
    libsppydev_DecoderType.ob_base = ob_base;
    libsppydev_DisplayType.ob_base = ob_base;

    if (PyType_Ready(&libsppydev_CameraType) < 0) {
        return nullptr;
    }

    if (PyType_Ready(&libsppydev_EncoderType) < 0) {
        return nullptr;
    }

    if (PyType_Ready(&libsppydev_DecoderType) < 0) {
        return nullptr;
    }

    if (PyType_Ready(&libsppydev_DisplayType) < 0) {
        return nullptr;
    }

    Py_INCREF(&libsppydev_CameraType);
    Py_INCREF(&libsppydev_EncoderType);
    Py_INCREF(&libsppydev_DecoderType);
    Py_INCREF(&libsppydev_DisplayType);

    PyModule_AddObject(m, "Camera", (PyObject *)&libsppydev_CameraType);
    PyModule_AddObject(m, "Encoder", (PyObject *)&libsppydev_EncoderType);
    PyModule_AddObject(m, "Decoder", (PyObject *)&libsppydev_DecoderType);
    PyModule_AddObject(m, "Display", (PyObject *)&libsppydev_DisplayType);

    return m;
}

#ifdef __cplusplus
}
#endif /* extern "C" */
