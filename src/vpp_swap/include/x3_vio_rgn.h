/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_VIO_RGN_H_
#define X3_VIO_RGN_H_

#ifdef __cplusplus
extern "C" {
#endif

int x3_rgn_init(RGN_HANDLE Handle, RGN_CHN_S *chn, RGN_ATTR_S *pstRegion, RGN_CHN_ATTR_S *pstChnAttr);
int x3_rgn_deinit(RGN_HANDLE Handle, RGN_CHN_S *chn);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // X3_VIO_RGN_H_
