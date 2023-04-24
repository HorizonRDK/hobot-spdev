/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2022 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <cJSON.h>
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

#define MAX_CAMERAS 3
typedef struct {
    int enable;
    int i2c_bus;
    int mipi_host;
} board_camera_info_t;

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

static cJSON *open_json_file(const char *path)
{
    FILE *fp = fopen(path, "r");
    int32_t ret = 0;
    if (fp == NULL)
    {
        perror("fopen");
        return NULL;
    }

    fseek(fp, 0, SEEK_END);
    long fsize = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    char *buf = malloc(fsize + 1);
    ret = fread(buf, fsize, 1, fp);
    if (ret != 1)
    {
        LOGE_print("Error fread size:%d\n", ret);
    }
    fclose(fp);

    buf[fsize] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (root == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
            LOGE_print("Error cJSON_Parse: %s\n", error_ptr);
        }
        free(buf);
        return NULL;
    }
    free(buf);

    return root;
}

static int parse_cameras(cJSON *cameras, board_camera_info_t camera_info[])
{
    cJSON *camera = NULL;
    int ret;

    for (int i = 0; i < MAX_CAMERAS; i++) {
        camera_info[i].enable = 0;

        camera = cJSON_GetArrayItem(cameras, i);
        if (camera == NULL) {
            break;
        }

        // parse reset
        cJSON *reset = cJSON_GetObjectItem(camera, "reset");
        if (reset == NULL) {
            ret = -1;
            goto exit;
        }

        int gpio_num = atoi(strtok(reset->valuestring, ":"));
        char *active = strtok(NULL, ":");

        // parse i2c_bus
        cJSON *i2c_bus_obj = cJSON_GetObjectItem(camera, "i2c_bus");
        if (i2c_bus_obj == NULL) {
            ret = -1;
            goto exit;
        }
        camera_info[i].i2c_bus = i2c_bus_obj->valueint;

        // parse mipi_host
        cJSON *mipi_host_obj = cJSON_GetObjectItem(camera, "mipi_host");
        if (mipi_host_obj == NULL) {
            ret = -1;
            goto exit;
        }
        camera_info[i].mipi_host = mipi_host_obj->valueint;
        camera_info[i].enable = 1;

        printf("Camera: gpio_num=%d, active=%s, i2c_bus=%d, mipi_host=%d\n",
            gpio_num, active, i2c_bus_obj->valueint, mipi_host_obj->valueint);
    }
    ret = 0;

exit:
    return ret;
}

static int parse_config(const char *json_file, board_camera_info_t camera_info[])
{
    int ret = 0;
    cJSON *root = NULL;
    cJSON *board_config = NULL;
    cJSON *cameras = NULL;
    char som_name[16];
    char board_name[32];

    FILE *fp = fopen("/sys/class/socinfo/som_name", "r");
    if (fp == NULL) {
        return -1 ;
    }
    fscanf(fp, "%s", som_name);
    fclose(fp);

    snprintf(board_name, sizeof(board_name), "board_%s", som_name);

    root = open_json_file(json_file);
    if (!root) {
        fprintf(stderr, "Failed to parse JSON string.\n");
        return -1 ;
    }

    board_config = cJSON_GetObjectItem(root, board_name);
    if (!board_config) {
        fprintf(stderr, "Failed to get board_config object.\n");
        ret = -1;
        goto exit;
    }

    cameras = cJSON_GetObjectItem(board_config, "cameras");
    if (!cameras) {
        fprintf(stderr, "Failed to get cameras array.\n");
        ret = -1;
        goto exit;
    }

    ret = parse_cameras(cameras, camera_info);

exit:
    cJSON_Delete(root);
    return ret;
}

static sensor_id_t *check_sensor(const int i2c_bus, sensor_id_t *sensos_list, int length)
{
    int i = 0;
    char cmd[128] = {0};
    char result[1024] = {0};

    for (i = 0; i < length; i++) {
        /* 通过i2ctransfer命令读取特定寄存器，判断是否读取成功来判断是否支持相应的sensor */
        memset(cmd, '\0', sizeof(cmd));
        memset(result, '\0', sizeof(result));
        if (sensos_list[i].i2c_addr_width == I2C_ADDR_8) {
            sprintf(cmd, "i2ctransfer -y -f %d w1@0x%x 0x%x r1 2>&1", i2c_bus,
                    sensos_list[i].i2c_dev_addr, sensos_list[i].det_reg);
        } else if (sensos_list[i].i2c_addr_width == I2C_ADDR_16) {
            sprintf(cmd, "i2ctransfer -y -f %d w2@0x%x 0x%x 0x%x r1 2>&1", i2c_bus,
                    sensos_list[i].i2c_dev_addr,
                    sensos_list[i].det_reg >> 8, sensos_list[i].det_reg & 0xFF);
        } else {
            continue;
        }
        exec_cmd_ex(cmd, result, 1024);
        if (strstr(result, "Error") == NULL && strstr(result, "error") == NULL) { // 返回结果中不带Error, 说明sensor找到了
            printf("cmd=%s, result=%s\n", cmd, result);
            sensos_list[i].i2c_bus = i2c_bus;
            return &sensos_list[i];
        }
    }

    return NULL;
}

static sensor_id_t s_sensor_id_list[] = {
    {1, 0x36, I2C_ADDR_16, 0x300A, "ov5647", ov5647_linear_vin_param_init}, // ov5647 for x3-pi
    {1, 0x10, I2C_ADDR_16, 0x0000, "imx219", imx219_linear_vin_param_init}, // imx219 for x3-pi
    {1, 0x1a, I2C_ADDR_16, 0x0200, "imx477", imx477_linear_vin_param_init}, // imx477 for x3-pi
    {1, 0x40, I2C_ADDR_8, 0x0B, "f37", f37_linear_vin_param_init},          // F37
    {1, 0x29, I2C_ADDR_16, 0x03f0, "gc4663", gc4663_linear_vin_param_init}, // GC4663
};

int vin_param_init(const int cam_idx, x3_vin_info_t *vin_info)
{
    int ret = 0;
    sensor_id_t *sensor_id;
    board_camera_info_t camera_info[MAX_CAMERAS];
    int i2c_bus = -1;
    int mipi_host = -1;

    memset(camera_info, 0, sizeof(camera_info));
    ret = parse_config("/etc/board_config.json", camera_info);
    if (ret != 0) {
        printf("Failed to parse cameras\n");
        return -1;
    }

    for (int i = 0; i < MAX_CAMERAS; i++) {
        printf("Camera %d:\n", i);
        printf("\tenable: %d\n", camera_info[i].enable);
        printf("\ti2c_bus: %d\n", camera_info[i].i2c_bus);
        printf("\tmipi_host: %d\n", camera_info[i].mipi_host);
    }

    if (cam_idx >= 0 && cam_idx < MAX_CAMERAS) {
        i2c_bus = camera_info[cam_idx].i2c_bus;
        mipi_host = camera_info[cam_idx].mipi_host;
        if (camera_info[cam_idx].enable)
            sensor_id = check_sensor(i2c_bus, s_sensor_id_list, ARRAY_SIZE(s_sensor_id_list));
    } else if (cam_idx == -1) {
        for (int i = 0; i < MAX_CAMERAS; i++) {
            if (!camera_info[cam_idx].enable)
                continue;
            i2c_bus = camera_info[i].i2c_bus;
            mipi_host = camera_info[i].mipi_host;
            sensor_id = check_sensor(i2c_bus, s_sensor_id_list, ARRAY_SIZE(s_sensor_id_list));
            if (sensor_id != NULL)
                break;
        }
    } else {
        printf("The parameter video_idx=%d is not supported. Please set it to one of [-1, 0, 1, 2].\n",
            cam_idx);
        return -1;
    }

    if (sensor_id == NULL) {
        printf("No camera sensor found, please check whether the camera connection or video_idx is correct.\n");
        return -1;
    }

    sensor_id->sensor_vin_param(vin_info);
    vin_info->isp_enable = 1;
    vin_info->snsinfo.sensorInfo.bus_num = i2c_bus;
    vin_info->snsinfo.sensorInfo.entry_index = mipi_host;

    printf("Found sensor:%s on i2c bus %d, use mipi host %d\n",
        sensor_id->sensor_name, i2c_bus, mipi_host);

    if (mipi_host == 0) {
        exec_cmd("echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart");
    } else if (mipi_host == 1) {
        exec_cmd("echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart");
    } else if (mipi_host == 2) {
        exec_cmd("echo 1 > /sys/class/vps/mipi_host2/param/stop_check_instart");
    }

    return 0;
}
