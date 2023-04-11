/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2022 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/

#ifndef SENSOR_OV10635_CONFIG_H_
#define SENSOR_OV10635_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "vio/hb_mipi_api.h"
#include "vio/hb_vin_api.h"

#define OV10635_REG_ADDR 0x40

extern MIPI_SENSOR_INFO_S SENSOR_4LANE_OV10635_12BIT_YUV422_INFO;
extern MIPI_ATTR_S MIPI_4LANE_OV10635_12BIT_YUV422_ATTR;
extern VIN_DEV_ATTR_S DEV_ATTR_4LANE_OV10635_12BIT_YUV422_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_4LANE_OV10635_12BIT_YUV422_BASE;

extern VIN_DIS_ATTR_S DIS_ATTR_OV10635_BASE;
extern VIN_LDC_ATTR_S LDC_ATTR_OV10635_BASE;

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // SENSOR_IMX415_CONFIG_H_