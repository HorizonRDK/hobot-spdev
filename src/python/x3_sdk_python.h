/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-03-19 10:29:31
 * @LastEditTime: 2023-03-19 14:29:09
 ***************************************************************************/
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

#ifndef _X3_SDK_PYTHON_H_
#define _X3_SDK_PYTHON_H_

#include <stdio.h>
#include <unistd.h>

#include <atomic>
#include <mutex>
#include <Python.h>

#include "x3_sdk_wrap.h"

#define __X3_API__ __attribute__((visibility("default")))

#define AUTO_GUARD_MTX_LOCK(mtxlock) \
    std::lock_guard<std::mutex> __guard_lock__##mtxlock(mtxlock)

#define AUTO_GUARD_RECURSIVE_LOCK(mtxlock) \
    std::lock_guard<std::recursive_mutex> __guard_lock__##mtxlock(mtxlock)

#define AUTO_UNIQUE_MTX_LOCK(mtxlock) \
    std::unique_lock<std::mutex> __unique_lock__##mtxlock(mtxlock)

#define AUTO_UNIQUE_RECURSIVE_MTX_LOCK(mtxlock) \
    std::unique_lock<std::recursive_mutex> __unique_lock__##mtxlock(mtxlock)

#define EN_DBG 0

#define PRINT(fmt, ...)                                              \
    do {                                                             \
        printf("[%s]:[%d]:" fmt, __func__, __LINE__, ##__VA_ARGS__); \
    } while (0)

#define DBG(fmt, ...)                  \
    do {                               \
        if (EN_DBG) {                  \
            PRINT(fmt, ##__VA_ARGS__); \
        }                              \
    } while (0)

#define ATOMIC_READ_HEAD(fd, buf, count)  pread(fd, buf, count, SEEK_SET)
#define ATOMIC_WRITE_HEAD(fd, buf, count) pwrite(fd, buf, count, SEEK_SET)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    PyObject_HEAD;
    void *pobj;
    ImageFrame *pframe;
    SrPy_Object_e object;
} libsppydev_Object;

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
