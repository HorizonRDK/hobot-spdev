/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2022 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "utils_log.h"

#include "sensor_f37_config.h"
#include "sensor_gc4663_config.h"
#include "sensor_os8a10_config.h"
#include "sensor_imx415_config.h"
#include "sensor_ov8856_config.h"
#include "sensor_sc031gs_config.h"
#include "sensor_ofilm0092_config.h"
#include "sensor_ov10635_config.h"
#include "sensor_ov5647_config.h"
#include "sensor_imx219_config.h"
#include "sensor_imx477_config.h"
// sunrise camare 封装的头文件
#include "x3_vio_venc.h"
#include "x3_vio_vin.h"
#include "x3_vio_vps.h"
#include "x3_vio_bind.h"
#include "x3_sdk_wrap.h"

#include "common_utils.h"

typedef struct sensor_id {
    int i2c_bus;      // sensor挂在哪条总线上
    int i2c_dev_addr; // sensor i2c设备地址
    int i2c_addr_width;
    int det_reg;
    char sensor_name[10];
    int (*sensor_vin_param)(x3_vin_info_t *vin_info);
} sensor_id_t;

#define I2C_ADDR_8  1
#define I2C_ADDR_16 2

/******************************* F37 方案 **********************************/
int f37_linear_vin_param_init(x3_vin_info_t *vin_info)
{
    vin_info->snsinfo = SENSOR_1LANE_F37_30FPS_10BIT_LINEAR_INFO;
    vin_info->mipi_attr = MIPI_1LANE_SENSOR_F37_30FPS_10BIT_LINEAR_ATTR;
    vin_info->devinfo = DEV_ATTR_F37_LINEAR_BASE;
    vin_info->pipeinfo = PIPE_ATTR_F37_LINEAR_BASE;
    vin_info->disinfo = DIS_ATTR_F37_LINEAR_BASE;
    vin_info->ldcinfo = LDC_ATTR_F37_LINEAR_BASE;
    vin_info->vin_vps_mode = VIN_OFFLINE_VPS_OFFINE;

    vin_info->enable_dev_attr_ex = 0;

    return 0;
}

/******************************** GC4663 方案 ******************************/
int gc4663_linear_vin_param_init(x3_vin_info_t* vin_info)
{
	vin_info->snsinfo = SENSOR_2LANE_GC4663_30FPS_10BIT_LINEAR_INFO;
	vin_info->mipi_attr = MIPI_2LANE_SENSOR_GC4663_30FPS_10BIT_LINEAR_ATTR;
	vin_info->devinfo = DEV_ATTR_GC4663_LINEAR_BASE;
	vin_info->pipeinfo = PIPE_ATTR_GC4663_LINEAR_BASE;
	vin_info->disinfo = DIS_ATTR_GC4663_LINEAR_BASE;
	vin_info->ldcinfo = LDC_ATTR_GC4663_LINEAR_BASE;
	vin_info->vin_vps_mode = VIN_OFFLINE_VPS_OFFINE;

	vin_info->enable_dev_attr_ex = 0;

	return 0;
}

/******************************** OS8a10 4K 方案 ******************************/
int os8a10_linear_vin_param_init(x3_vin_info_t *vin_info)
{
    vin_info->snsinfo = SENSOR_OS8A10_30FPS_10BIT_LINEAR_INFO;
    vin_info->mipi_attr = MIPI_SENSOR_OS8A10_30FPS_10BIT_LINEAR_ATTR;
    vin_info->devinfo = DEV_ATTR_OS8A10_LINEAR_BASE;
    vin_info->pipeinfo = PIPE_ATTR_OS8A10_LINEAR_BASE;
    vin_info->disinfo = DIS_ATTR_OS8A10_BASE;
    vin_info->ldcinfo = LDC_ATTR_OS8A10_BASE;
    vin_info->vin_vps_mode = VIN_SIF_ONLINE_DDR_ISP_DDR_VPS_ONLINE;

    vin_info->enable_dev_attr_ex = 0;

    return 0;
}

/******************************** IMX415 4K 方案 ******************************/
int imx415_linear_vin_param_init(x3_vin_info_t *vin_info)
{
    vin_info->snsinfo = SENSOR_4LANE_IMX415_30FPS_10BIT_LINEAR_INFO;
    vin_info->mipi_attr = MIPI_4LANE_SENSOR_IMX415_30FPS_10BIT_LINEAR_ATTR;
    vin_info->devinfo = DEV_ATTR_IMX415_LINEAR_BASE;
    vin_info->pipeinfo = PIPE_ATTR_IMX415_LINEAR_BASE;
    vin_info->disinfo = DIS_ATTR_IMX415_BASE;
    vin_info->ldcinfo = LDC_ATTR_IMX415_BASE;
    /*vin_info->vin_vps_mode = VIN_SIF_ONLINE_DDR_ISP_DDR_VPS_ONLINE;*/
    vin_info->vin_vps_mode = VIN_OFFLINE_VPS_ONLINE;

    vin_info->enable_dev_attr_ex = 0;

    return 0;
}

/******************************** OV5647 ******************************/
int ov5647_linear_vin_param_init(x3_vin_info_t *vin_info)
{
    vin_info->snsinfo = SENSOR_2LANE_OV5647_30FPS_10BIT_LINEAR_INFO;
    vin_info->mipi_attr = MIPI_2LANE_SENSOR_OV5647_30FPS_10BIT_LINEAR_ATTR;
    vin_info->devinfo = DEV_ATTR_OV5647_LINEAR_BASE;
    vin_info->pipeinfo = PIPE_ATTR_OV5647_LINEAR_BASE;
    vin_info->disinfo = DIS_ATTR_OV5647_LINEAR_BASE;
    vin_info->ldcinfo = LDC_ATTR_OV5647_LINEAR_BASE;
    vin_info->vin_vps_mode = VIN_OFFLINE_VPS_OFFINE;//TBD
    vin_info->enable_dev_attr_ex = 0;
    vin_info->snsinfo.sensorInfo.sensor_addr = 0x36;

    return 0;
}

/******************************** IMX219 ******************************/
int imx219_linear_vin_param_init(x3_vin_info_t *vin_info)
{
    vin_info->snsinfo = SENSOR_2LANE_IMX219_30FPS_10BIT_LINEAR_INFO;
    vin_info->mipi_attr = MIPI_2LANE_SENSOR_IMX219_30FPS_10BIT_LINEAR_ATTR;
    vin_info->devinfo = DEV_ATTR_IMX219_LINEAR_BASE;
    vin_info->pipeinfo = PIPE_ATTR_IMX219_LINEAR_BASE;
    vin_info->disinfo = DIS_ATTR_IMX219_LINEAR_BASE;
    vin_info->ldcinfo = LDC_ATTR_IMX219_LINEAR_BASE;
    vin_info->vin_vps_mode = VIN_OFFLINE_VPS_OFFINE;//TBD
    vin_info->enable_dev_attr_ex = 0;
    vin_info->snsinfo.sensorInfo.sensor_addr = 0x10;
    return 0;
}

/******************************** IMX477 ******************************/
int imx477_linear_vin_param_init(x3_vin_info_t *vin_info)
{
    vin_info->snsinfo = SENSOR_2LANE_IMX477_50FPS_12BIT_LINEAR_INFO;
    vin_info->mipi_attr = MIPI_2LANE_SENSOR_IMX477_50FPS_12BIT_LINEAR_ATTR;
    vin_info->devinfo = DEV_ATTR_IMX477_LINEAR_BASE;
    vin_info->pipeinfo = PIPE_ATTR_IMX477_LINEAR_BASE;
    vin_info->disinfo = DIS_ATTR_IMX477_LINEAR_BASE;
    vin_info->ldcinfo = LDC_ATTR_IMX477_LINEAR_BASE;
    vin_info->vin_vps_mode = VIN_OFFLINE_VPS_OFFINE;//TBD
    vin_info->enable_dev_attr_ex = 0;
    vin_info->snsinfo.sensorInfo.sensor_addr = 0x1a;
    return 0;
}

/******************************** OV10635 ******************************/
int ov10635_yuv422_vin_param_init(x3_vin_info_t *vin_info)
{
    vin_info->snsinfo = SENSOR_4LANE_OV10635_12BIT_YUV422_INFO;
    vin_info->mipi_attr = MIPI_4LANE_OV10635_12BIT_YUV422_ATTR;
    vin_info->devinfo = DEV_ATTR_4LANE_OV10635_12BIT_YUV422_BASE;
    vin_info->pipeinfo = PIPE_ATTR_4LANE_OV10635_12BIT_YUV422_BASE;
    vin_info->disinfo = DIS_ATTR_OV10635_BASE;
    vin_info->ldcinfo = LDC_ATTR_OV10635_BASE;
    vin_info->vin_vps_mode = VIN_SIF_OFFLINE_VPS_OFFLINE;
    // vin_info->vin_vps_mode = VIN_ONLINE_VPS_ONLINE;

    vin_info->enable_dev_attr_ex = 0;
    vin_info->snsinfo.sensorInfo.sensor_addr = OV10635_REG_ADDR;

    return 0;
}

// vps的输入参数
int vps_grp_param_init(x3_vps_info_t *vps_info, int pipe_id, int width, int height)
{
    vps_info->m_vps_grp_id = pipe_id;
    vps_info->m_vps_grp_attr.maxW = width;
    vps_info->m_vps_grp_attr.maxH = height;
    /* vps grp的framedepth是gdc的buf深度，如果只用groupRotate，配成1就行 */
    vps_info->m_vps_grp_attr.frameDepth = 1;

    return 0;
}

// vps的通道参数，为什么要grp和chn分开两个调用，目的是为了更加灵活的配置一个group的输出
int vps_chn_param_init(x3_vps_chn_attr_t *vps_chn_attr, int chn_id, int width, int height, int fps,
    int crop_x, int crop_y, int crop_width, int crop_height, int rotate)
{
    vps_chn_attr->m_chn_id = chn_id; // vps 通道，只有chn2 和 chn5 支持4K输出
    vps_chn_attr->m_chn_enable = 1;
    vps_chn_attr->m_is_bind = 0;
    vps_chn_attr->m_chn_attr.width = width;
    vps_chn_attr->m_chn_attr.height = height;

    vps_chn_attr->m_chn_attr.enMirror = 0;
    vps_chn_attr->m_chn_attr.enFlip = 0;
    vps_chn_attr->m_chn_attr.enScale = 1;
    vps_chn_attr->m_chn_attr.frameDepth = 2; /* 图像队列长度, 最大32 */
    vps_chn_attr->m_chn_attr.frameRate.srcFrameRate = fps;
    vps_chn_attr->m_chn_attr.frameRate.dstFrameRate = fps;

    return 0;
}

// vps的通道裁剪参数，
int vps_chn_crop_param_init(x3_vps_chn_attr_t *vps_chn_attr, int x, int y, int width, int height)
{
    vps_chn_attr->m_chn_crop_attr.en = 1;
    vps_chn_attr->m_chn_crop_attr.cropRect.x = x;
    vps_chn_attr->m_chn_crop_attr.cropRect.y = y;
    vps_chn_attr->m_chn_crop_attr.cropRect.width = width;
    vps_chn_attr->m_chn_crop_attr.cropRect.height = height;

    return 0;
}

// vps的通道旋转参数，
int vps_chn_rotate_param_init(x3_vps_chn_attr_t *vps_chn_attr, int rotate)
{
    vps_chn_attr->m_rotate = rotate;

    return 0;
}

static sensor_id_t *check_sensor(sensor_id_t *sensos_list, int length)
{
    int i = 0;
    char cmd[128] = {0};
    char result[1024] = {0};

    for (i = 0; i < length; i++) {
        /* 通过i2ctransfer命令读取特定寄存器，判断是否读取成功来判断是否支持相应的sensor */
        memset(cmd, '\0', sizeof(cmd));
        memset(result, '\0', sizeof(result));
        if (sensos_list[i].i2c_addr_width == I2C_ADDR_8) {
            sprintf(cmd, "i2ctransfer -y -f %d w1@0x%x 0x%x r1 2>&1", sensos_list[i].i2c_bus,
                    sensos_list[i].i2c_dev_addr, sensos_list[i].det_reg);
        } else if (sensos_list[i].i2c_addr_width == I2C_ADDR_16) {
            sprintf(cmd, "i2ctransfer -y -f %d w2@0x%x 0x%x 0x%x r1 2>&1", sensos_list[i].i2c_bus,
                    sensos_list[i].i2c_dev_addr,
                    sensos_list[i].det_reg >> 8, sensos_list[i].det_reg & 0xFF);
        } else {
            continue;
        }
        exec_cmd_ex(cmd, result, 1024);
        if (strstr(result, "Error") == NULL) { // 返回结果中不带Error, 说明sensor找到了
            // printf("match sensor:%s\n", sensos_list[i].sensor_name);
            return &sensos_list[i];
        }
    }

    return NULL;
}

static sensor_id_t s_sensor_id_list[] = {
    {1, 0x40, I2C_ADDR_8, 0x0B, "f37", f37_linear_vin_param_init},                // F37
    {2, 0x40, I2C_ADDR_8, 0x0B, "f37", f37_linear_vin_param_init},                // F37
    {1, 0x29, I2C_ADDR_16, 0x03f0, "gc4663", gc4663_linear_vin_param_init},       // GC4663
    {2, 0x29, I2C_ADDR_16, 0x03f0, "gc4663", gc4663_linear_vin_param_init},       // GC4663
    {1, 0x36, I2C_ADDR_16, 0x300A, "ov5647", ov5647_linear_vin_param_init}, // ov5647 for x3-pi
    {1, 0x10, I2C_ADDR_16, 0x0000, "imx219", imx219_linear_vin_param_init}, // imx219 for x3-pi
    {1, 0x1a, I2C_ADDR_16, 0x0200, "imx477", imx477_linear_vin_param_init}, // imx477 for x3-pi
    {3, 0x36, I2C_ADDR_16, 0x300A, "ov5647", ov5647_linear_vin_param_init}, // ov5647 for x3-cm
    {3, 0x10, I2C_ADDR_16, 0x0000, "imx219", imx219_linear_vin_param_init}, // imx219 for x3-cm
    {3, 0x1a, I2C_ADDR_16, 0x0200, "imx477", imx477_linear_vin_param_init}, // imx477 for x3-cm

};


int vin_param_init(const int video_index, x3_vin_info_t *vin_info)
{
    sensor_id_t *sensor_id;

    sensor_id = check_sensor(s_sensor_id_list, ARRAY_SIZE(s_sensor_id_list));
    if (sensor_id == NULL)
        return -1;
    sensor_id->sensor_vin_param(vin_info);
    vin_info->isp_enable = 1;
    vin_info->snsinfo.sensorInfo.bus_num = sensor_id->i2c_bus;
    vin_info->snsinfo.sensorInfo.entry_index = video_index;

    return 0;
}
