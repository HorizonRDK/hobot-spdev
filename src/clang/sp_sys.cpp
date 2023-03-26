/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-03-05 16:55:56
 * @LastEditTime: 2023-03-05 16:59:38
 ***************************************************************************/
#include <sys/stat.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>

#include "sp_sys.h"

using namespace std;

int sp_module_bind(void *src, int src_type, void *dst, int dst_type)
{
    int ret = 0;

    return ret;
}

int sp_module_unbind(void *src, int src_type, void *dst, int dst_type)
{
    int ret = 0;

    return ret;
}