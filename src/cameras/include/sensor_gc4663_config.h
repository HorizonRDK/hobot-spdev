/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef SENSOR_GC4663_CONFIG_H_
#define SENSOR_GC4663_CONFIG_H_

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

#include "vio/hb_mipi_api.h"
#include "vio/hb_vin_api.h"

extern MIPI_SENSOR_INFO_S SENSOR_2LANE_GC4663_30FPS_10BIT_LINEAR_INFO;
extern MIPI_ATTR_S MIPI_2LANE_SENSOR_GC4663_30FPS_10BIT_LINEAR_ATTR;
extern VIN_DEV_ATTR_S DEV_ATTR_GC4663_LINEAR_BASE;
extern VIN_PIPE_ATTR_S PIPE_ATTR_GC4663_LINEAR_BASE;
extern VIN_DIS_ATTR_S DIS_ATTR_GC4663_LINEAR_BASE;
extern VIN_LDC_ATTR_S LDC_ATTR_GC4663_LINEAR_BASE;



#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
#endif // SENSOR_GC4663_CONFIG_H_