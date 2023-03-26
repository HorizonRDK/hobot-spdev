#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#include "cJSON.h"
#include "utils_log.h"

#include "x3_vio_vin.h"
#include "x3_vio_vps.h"
#include "cam/cam_common.h"

// 打印 vin isp vpu venc等模块的调试信息
void print_debug_infos(void)
{
    int ret;

    printf("========================= SIF ==========================\n");
    ret = system("cat /sys/devices/platform/soc/a4001000.sif/cfg_info");
    if (ret < 0)
    {
        printf("cmd:cat /sys/devices/platform/soc/a4001000.sif/cfg_info failed\n");
    }
    printf("========================= ISP ==========================\n");
    ret = system("cat /sys/devices/platform/soc/b3000000.isp/isp_status");
    if (ret < 0)
    {
        printf("cmd:cat /sys/devices/platform/soc/b3000000.isp/isp_status failed\n");
    }
    printf("========================= IPU PIPE enable ==============\n");
    ret = system("cat /sys/devices/platform/soc/a4040000.ipu/info/enabled_pipeline");
    if (ret < 0)
    {
        printf("cmd:cat /sys/devices/platform/soc/a4040000.ipu/info/enabled_pipeline failed\n");
    }
    printf("========================= IPU PIPE config ==============\n");
    ret = system("cat /sys/devices/platform/soc/a4040000.ipu/info/pipeline0_info");
    if (ret < 0)
    {
        printf("cmd:cat /sys/devices/platform/soc/a4040000.ipu/info/pipeline0_info failed\n");
    }
    ret = system("cat /sys/devices/platform/soc/a4040000.ipu/info/pipeline1_info");
    if (ret < 0)
    {
        printf("cmd:cat /sys/devices/platform/soc/a4040000.ipu/info/pipeline1_info failed\n");
    }
    ret = system("cat /sys/devices/platform/soc/a4040000.ipu/info/pipeline2_info");
    if (ret < 0)
    {
        printf("cmd:cat /sys/devices/platform/soc/a4040000.ipu/info/pipeline2_info failed\n");
    }
    printf("========================= VENC =========================\n");
    ret = system("cat /sys/kernel/debug/vpu/venc");
    if (ret < 0)
    {
        printf("cmd:cat /sys/kernel/debug/vpu/venc failed\n");
    }

    printf("========================= VDEC =========================\n");
    ret = system("cat /sys/kernel/debug/vpu/vdec");
    if (ret < 0)
    {
        printf("cmd:cat /sys/kernel/debug/vpu/vdec failed\n");
    }

    printf("========================= JENC =========================\n");
    ret = system("cat /sys/kernel/debug/jpu/jenc");
    if (ret < 0)
    {
        printf("cmd:cat /sys/kernel/debug/jpu/jenc failed\n");
    }

    printf("========================= IAR ==========================\n");
    ret = system("cat /sys/kernel/debug/iar");
    if (ret < 0)
    {
        printf("cmd:cat /sys/kernel/debug/iar failed\n");
    }

    printf("========================= ION ==========================\n");
    ret = system("cat /sys/kernel/debug/ion/heaps/carveout");
    if (ret < 0)
    {
        printf("cmd:cat /sys/kernel/debug/ion/heaps/carveout failed\n");
    }
    ret = system("cat /sys/kernel/debug/ion/heaps/ion_cma");
    if (ret < 0)
    {
        printf("cmd:cat /sys/kernel/debug/ion/heaps/ion_cma failed\n");
    }

    printf("========================= END ===========================\n");
}

#define I2C_ADDR_8 1
#define I2C_ADDR_16 2

#define MAX_PATH 1024
#define MAX_FILES 128

static cJSON *open_json_file(char *path)
{
    FILE *fp = fopen(path, "r");
    if (fp == NULL)
    {
        perror("fopen");
        return NULL;
    }

    fseek(fp, 0, SEEK_END);
    long fsize = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    char *buf = malloc(fsize + 1);
    fread(buf, fsize, 1, fp);
    fclose(fp);

    buf[fsize] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (root == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
            fprintf(stderr, "Error cJSON_Parse: %s\n", error_ptr);
        }
        free(buf);
        return NULL;
    }
    free(buf);

    return root;
}

int parse_player_cfg(char *path)
{
    cJSON *root = open_json_file(path);
    if (root == NULL)
    {
        printf("open %s failed\n", path);
        return -1;
    }

    cJSON_Delete(root);

    return 0;
}

static void traverse_directory(const char *dir_path, char **file_paths, int *count)
{
    DIR *dir = opendir(dir_path);
    if (dir == NULL)
    {
        fprintf(stderr, "Failed to open %s: %s\n", dir_path, strerror(errno));
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (entry->d_type == DT_DIR)
        {
            if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
            {
                char path[MAX_PATH];
                snprintf(path, sizeof(path), "%s/%s", dir_path, entry->d_name);
                traverse_directory(path, file_paths, count);
            }
        }
        else if (entry->d_type == DT_REG)
        {
            char file_path[MAX_PATH];
            snprintf(file_path, sizeof(file_path), "%s/%s", dir_path, entry->d_name);
            if (strlen(entry->d_name) >= 12 &&
                strcmp(entry->d_name + strlen(entry->d_name) - 12, "_player.json") == 0)
            {
                if (*count < MAX_FILES)
                {
                    file_paths[*count] = strdup(file_path);
                    (*count)++;
                }
                else
                {
                    fprintf(stderr, "Too many player files.\n");
                    return;
                }
            }
        }
    }

    closedir(dir);
}

static void *check_sensor(const int video_index, char **player_list, int count)
{
    int ret = 0, i = 0;
    char result[1024];
    char file_name[128];
    struct stat file_stat;
    FILE *stream;

    /* 使能sensor mclk, 否则i2c 通信不会成功 */
    for (i = 0; i < 4; i++)
    {
        /* 读取 /sys/class/vps/mipi_host[i]/param/snrclk_freq  的值 \
         * 如果该mipi host可以使用则会是一个大于1000的值，否则为0 \
         * 通过判断该值不为0作为可以使能该路mipi mclk的依据
         */
        memset(file_name, '\0', sizeof(file_name));
        memset(result, '\0', sizeof(result));
        sprintf(file_name, "/sys/class/vps/mipi_host%d/param/snrclk_freq", i);
        stream = fopen(file_name, "r");
        if (!stream)
        {
            continue;
        }
        ret = fread(result, sizeof(char), 32, stream);
        if (ret <= 0)
        {
            printf("read fail\n");
            fclose(stream);
            return -1;
        }
        fclose(stream);

        /* 如果频率不为0   就使能该路mclk */
        if (strncmp(result, "0", 1) != 0)
        {
            printf("Enable mipi host%d mclk\n", i);
            hb_cam_set_mclk(i, 24000000);
            hb_cam_enable_mclk(i);
        }
    }

    for (i = 0; i < count; i++)
    {
        printf("P: %s\n", player_list[i]);
    }
    return NULL;
}

int parse_sensor_cfg(const int cfg_index, x3_vin_info_t *vin_info)
{
    cJSON *root = open_json_file("/app/sensors/sensors_config.json");
    if (root == NULL)
    {
        printf("open /app/sensors/sensors_config.json failed\n");
        return -1;
    }

    // Get configs
    cJSON *configs = cJSON_GetObjectItemCaseSensitive(root, "configs");
    if (cJSON_IsArray(configs))
    {
        int cfg_size = cJSON_GetArraySize(configs);

        if (cfg_size < cfg_index + 1)
        {
            printf("Error: There are too few configurations in the sensor configuration.\n");
            cJSON_Delete(root);
            return -1;
        }

        // Get config object in the array
        cJSON *config = cJSON_GetArrayItem(configs, cfg_index);
        if (cJSON_IsObject(config))
        {
            // Get cam_cfg_path
            cJSON *cam_cfg_path = cJSON_GetObjectItemCaseSensitive(config, "cam_cfg_path");
            if (cJSON_IsString(cam_cfg_path) && (cam_cfg_path->valuestring != NULL))
            {
                printf("cam_cfg_path: %s\n", cam_cfg_path->valuestring);
                strcpy(vin_info->m_cam_cfg, cam_cfg_path->valuestring);
            }

            // Get vio_cfg_path
            cJSON *vio_cfg_path = cJSON_GetObjectItemCaseSensitive(config, "vio_cfg_path");
            if (cJSON_IsString(vio_cfg_path) && (vio_cfg_path->valuestring != NULL))
            {
                printf("vio_cfg_path: %s\n", vio_cfg_path->valuestring);
                strcpy(vin_info->m_vin_cfg, vio_cfg_path->valuestring);
            }
        }
    }

    cJSON_Delete(root);

    return 0;
}

int vin_param_init(const int cfg_index, x3_vin_info_t *vin_info)
{
    int ret = 0;
    char *player_list[MAX_FILES];
    // sensor_cfg_t *sensor_cfg;

    int list_count = 0;

    // 查找所有sensor的配置文件
    traverse_directory("/app/sensors", player_list, &list_count);

    // 1. 检查 index 对应类型的sensor是否存在
    // 2. 如果找到对应的sensor，完成参数初始化，否则返回错误
    check_sensor(cfg_index, player_list, list_count);

    ret = parse_sensor_cfg(cfg_index, vin_info);
    if (ret != 0)
    {
        return -1;
    }

    return ret;
}

static int set_ipu_chn_attr(int dst_width, int dst_height, x3_ipu_info_t *ipu_info)
{
    int src_width = ipu_info->source_width;
    int src_height = ipu_info->source_height;
    int chn_en = ipu_info->chn_en;
    int ds_idx = -1;

    if (((dst_width == src_width) || (dst_height == src_height)) &&
        (!(chn_en & 1 << HB_VIO_IPU_DS2_DATA)) &&
        (dst_width <= 4096 && dst_height <= 4096)) {
        ds_idx = HB_VIO_IPU_DS2_DATA;
    }
    if ((dst_width <= src_width) || (dst_height <= src_height)) {
        if ((dst_width <= 1920 && dst_height <= 1080) &&
            (!(chn_en & 1 << HB_VIO_IPU_DS1_DATA))) {
            ds_idx = HB_VIO_IPU_DS1_DATA;
        } else if ((dst_width <= 1920 && dst_height <= 1080) &&
                (!(chn_en & 1 << HB_VIO_IPU_DS3_DATA))) {
            ds_idx = HB_VIO_IPU_DS3_DATA;
        } else if ((dst_width <= 1280 && dst_height <= 720) &&
                (!(chn_en & 1 << HB_VIO_IPU_DS0_DATA))) {
            ds_idx = HB_VIO_IPU_DS0_DATA;
        } else if ((dst_width <= 1280 && dst_height <= 720) &&
                (!(chn_en & 1 << HB_VIO_IPU_DS4_DATA))) {
            ds_idx = HB_VIO_IPU_DS4_DATA;
        }
    }

    if (ds_idx != -1)
    {
        ipu_info->ipu_ds_config[ds_idx].ds_roi_height = dst_height;
        ipu_info->ipu_ds_config[ds_idx].ds_tag_height = dst_height;
        ipu_info->ipu_ds_config[ds_idx].ds_roi_width = dst_width;
        ipu_info->ipu_ds_config[ds_idx].ds_tag_width = dst_width;
        ipu_info->chn_en |= (1 << ds_idx);
        return 0;
    }

    if (((dst_width >= src_width) || (dst_height > src_height)) &&
            (!(chn_en & 1 << HB_VIO_IPU_US_DATA)) &&
            (dst_width <= 4096 && dst_height <= 4096)) {
        ipu_info->ipu_us_config.us_roi_height = dst_height;
        ipu_info->ipu_us_config.us_tag_height = dst_height;
        ipu_info->ipu_us_config.us_roi_width = dst_width;
        ipu_info->ipu_us_config.us_tag_width = dst_width;
        ipu_info->chn_en |= (1 << HB_VIO_IPU_US_DATA);
        return 0;
    }

    return -1;
}

static int json_replace_value_int(cJSON *root, char *key, int value)
{
    cJSON *field = cJSON_GetObjectItem(root, key);
    if (field == NULL) {
        printf("Field %s not found\n", key);
        return -1;
    }
    cJSON_ReplaceItemInObject(root, key, cJSON_CreateNumber(value));

    return 0;
}

static int modify_ipu_cfg_file(cJSON *root, x3_ipu_info_t *ipu_info)
{
    int chn_en = ipu_info->chn_en;
    char key[64];

    json_replace_value_int(root, "source_width", ipu_info->source_width);
    json_replace_value_int(root, "source_height", ipu_info->source_height);
    json_replace_value_int(root, "source_stride_y", ipu_info->source_stride_y);
    json_replace_value_int(root, "source_stride_uv", ipu_info->source_stride_uv);

    cJSON *ipu_us_config = cJSON_GetObjectItemCaseSensitive(root, "ipu_us_config");
    if (ipu_us_config == NULL) {
        printf("get object ipu_us_config failed!\n");
        return -1;
    }

    if (chn_en & (1 << HB_VIO_IPU_US_DATA))
    {
        json_replace_value_int(ipu_us_config, "upscale_us_en", 1);
        json_replace_value_int(ipu_us_config, "us_roi_width", ipu_info->ipu_us_config.us_roi_width);
        json_replace_value_int(ipu_us_config, "us_roi_height", ipu_info->ipu_us_config.us_roi_height);
        json_replace_value_int(ipu_us_config, "us_tag_width", ipu_info->ipu_us_config.us_tag_width);
        json_replace_value_int(ipu_us_config, "us_tag_height", ipu_info->ipu_us_config.us_tag_height);
    }
    else
    {
        json_replace_value_int(ipu_us_config, "upscale_us_en", 0);
    }

    cJSON *ipu_ds_configs = cJSON_GetObjectItemCaseSensitive(root, "ipu_ds_config");
    if (!cJSON_IsArray(ipu_ds_configs))
    {
        printf("get object ipu_ds_configs failed!\n");
        return -1;
    }

    for (int i = 0; i < 5; i++)
    {
        cJSON *ds_config = cJSON_GetArrayItem(ipu_ds_configs, i);
        if (cJSON_IsObject(ds_config))
        {
            if (chn_en & (1 << i))
            {
                sprintf(key, "downscale_ds%d_en", i);
                json_replace_value_int(ds_config, key, 1);
                sprintf(key, "ds%d_roi_en", i);
                json_replace_value_int(ds_config, key, 1);
                sprintf(key, "ds%d_roi_width", i);
                json_replace_value_int(ds_config, key, ipu_info->ipu_ds_config[i].ds_roi_width);
                sprintf(key, "ds%d_roi_height", i);
                json_replace_value_int(ds_config, key, ipu_info->ipu_ds_config[i].ds_roi_height);
                sprintf(key, "ds%d_tag_width", i);
                json_replace_value_int(ds_config, key, ipu_info->ipu_ds_config[i].ds_tag_width);
                sprintf(key, "ds%d_tag_height", i);
                json_replace_value_int(ds_config, key, ipu_info->ipu_ds_config[i].ds_tag_height);
            }
            else
            {
                sprintf(key, "downscale_ds%d_en", i);
                json_replace_value_int(ds_config, key, 0);
                sprintf(key, "ds%d_roi_en", i);
                json_replace_value_int(ds_config, key, 0);
            }
        }
    }

    return 0;
}

void remove_tempfile() {
    unlink("/tmp/hbviocfg_XXXXXX");
}

int x3_reconfig_ipu(int chn_num, int *width, int *height, x3_vin_info_t *vin_info, x3_ipu_info_t *ipu_info)
{
    int ret = 0;
    int isp_width = 0;
    int isp_height = 0;

    cJSON *root = open_json_file(vin_info->m_vin_cfg);
    if (root == NULL)
    {
        fprintf(stderr, "Failed to open %s: %s\n", vin_info->m_vin_cfg, strerror(errno));
        return -1;
    }

    cJSON *pipeline0 = cJSON_GetObjectItemCaseSensitive(root, "pipeline0");
    if (cJSON_IsObject(pipeline0))
    {
        cJSON *isp = cJSON_GetObjectItemCaseSensitive(pipeline0, "isp");
        if (cJSON_IsObject(isp))
        {
            cJSON *out_width = cJSON_GetObjectItemCaseSensitive(isp, "out_width");
            if (cJSON_IsNumber(out_width))
            {
                printf("out_width: %d\n", out_width->valueint);
                isp_width = out_width->valueint;
            }
            cJSON *out_height = cJSON_GetObjectItemCaseSensitive(isp, "out_height");
            if (cJSON_IsNumber(out_height))
            {
                printf("out_height: %d\n", out_height->valueint);
                isp_height = out_height->valueint;
            }
        }
    }

    printf("isp-width: %d\n", isp_width);
    printf("isp-height: %d\n", isp_height);

    ipu_info->source_height = isp_height;
    ipu_info->source_width = isp_width;
    ipu_info->source_stride_y = isp_width;
    ipu_info->source_stride_uv = isp_width;

    for (int i = 0; i < chn_num; i++)
    {
        if ((width[i] == 0) && (height[i] == 0))
        {
            width[i] = isp_width;
            height[i] = isp_height;
        }
        ret = set_ipu_chn_attr(width[i], height[i], ipu_info);
        if (ret != 0)
        {
            printf("Can not match the ipu channel by width(%d) height(%d)\n", width[i], height[i]);
            cJSON_Delete(root);
            return -1;
        } 
    }

    if (pipeline0 == NULL) {
        printf("get object pipeline0 failed!\n");
        cJSON_Delete(root);
        return -1;
    }

    cJSON *ipu = cJSON_GetObjectItemCaseSensitive(pipeline0, "ipu");
    if (ipu == NULL) {
        printf("get object ipu failed!\n");
        cJSON_Delete(root);
        return -1;
    }

    cJSON *cfg_size = cJSON_GetObjectItemCaseSensitive(ipu, "cfg_size");
    if (ipu == NULL) {
        printf("get object cfg_size failed!\n");
        cJSON_Delete(root);
        return -1;
    }

    ret = modify_ipu_cfg_file(cfg_size, ipu_info);
    if (ret != 0) {
        printf("modify_ipu_cfg_file failed!\n");
        cJSON_Delete(root);
        return -1;
    }

    // Generate a new temporary configuration file for ipu

    char template[] = "/tmp/hbviocfg_XXXXXX";
    int fd = mkstemp(template);
    if (fd == -1) {
        perror("mkstemp");
        cJSON_Delete(root);
        return -1;
    }
    printf("Created temporary file: %s\n", template);

    char *json_str = cJSON_PrintUnformatted(root);
    write(fd, json_str, strlen(json_str));

    close(fd);

    // Remove the temporary file when the program exits
    if (atexit(remove_tempfile) != 0) {
        perror("atexit");
        cJSON_Delete(root);
        return -1;
    }

    // Update vin config file
    sprintf(vin_info->m_vin_cfg, "%s", template);

    cJSON_Delete(root);
}

void x3_normal_buf_info_print(hb_vio_buffer_t *buf)
{
    int i = 0;
    printf("normal pipe_id (%d)type(%d)frame_id(%d)buf_index(%d)w x h(%d x %d)"
           "stride(%d) data_type %d img_format %d\n",
           buf->img_info.pipeline_id,
           buf->img_info.data_type,
           buf->img_info.frame_id,
           buf->img_info.buf_index,
           buf->img_addr.width,
           buf->img_addr.height,
           buf->img_addr.stride_size,
           buf->img_info.data_type,
           buf->img_info.img_format);

    for (i = 0; i < buf->img_info.planeCount; i++)
    {
        printf("addr[%d]: %p ", i, buf->img_addr.addr[i]);
    }
    printf("\n");
}

int x3_dump_nv12(char *filename, char *srcBuf, char *srcBuf1,
                 unsigned int size, unsigned int size1)
{
    FILE *yuvFd = NULL;
    char *buffer = NULL;

    yuvFd = fopen(filename, "w+");

    if (yuvFd == NULL)
    {
        printf("open(%s) fail", filename);
        return -1;
    }

    buffer = (char *)malloc(size + size1);

    if (buffer == NULL)
    {
        printf("ERR:malloc file");
        fclose(yuvFd);
        return -1;
    }

    memcpy(buffer, srcBuf, size);
    memcpy(buffer + size, srcBuf1, size1);

    fflush(stdout);

    fwrite(buffer, 1, size + size1, yuvFd);

    fflush(yuvFd);

    if (yuvFd)
        fclose(yuvFd);
    if (buffer)
        free(buffer);

    printf("DEBUG:filedump(%s, size(%d) is successed!!\n", filename, size);

    return 0;
}

int x3_dump_vio_buf_to_nv12(char *filename, hb_vio_buffer_t *vio_buf)
{
    FILE *yuvFd = NULL;
    char *buffer = NULL;
    int i = 0;
    int stride = 0, width = 0, height = 0;

    if (filename == NULL || vio_buf == NULL)
        return -1;

    stride = vio_buf->img_addr.stride_size;
    width = vio_buf->img_addr.width;
    height = vio_buf->img_addr.height;

    yuvFd = fopen(filename, "w+");

    if (yuvFd == NULL)
    {
        printf("open(%s) fail", filename);
        return -1;
    }

    buffer = (char *)malloc(width * height * 3 / 2);

    if (buffer == NULL)
    {
        printf("ERR:malloc file");
        fclose(yuvFd);
        return -1;
    }

    if (stride == width)
    {
        memcpy(buffer, vio_buf->img_addr.addr[0], width * height);
        memcpy(buffer + width * height, vio_buf->img_addr.addr[1], width * height / 2);
    }
    else
    {
        // jump over stride - width Y
        for (i = 0; i < height; i++)
        {
            memcpy(buffer + i * width, vio_buf->img_addr.addr[0] + i * stride, width);
        }

        // jump over stride - width UV
        for (i = 0; i < height / 2; i++)
        {
            memcpy(buffer + width * height + i * width, vio_buf->img_addr.addr[1] + i * stride, width);
        }
    }

    fflush(stdout);
    fwrite(buffer, 1, width * height * 3 / 2, yuvFd);
    fflush(yuvFd);

    if (yuvFd)
        fclose(yuvFd);
    if (buffer)
        free(buffer);

    printf("filedump(%s, size(%d) is successed\n", filename, width * height * 3 / 2);
    return 0;
}

int x3_save_jpeg(char *filename, char *srcBuf, unsigned int size)
{
    FILE *fd = NULL;

    fd = fopen(filename, "w+");

    if (fd == NULL)
    {
        printf("open(%s) fail", filename);
        return -1;
    }

    fflush(stdout);
    fwrite(srcBuf, 1, size, fd);
    fflush(fd);

    if (fd)
        fclose(fd);

    printf("DEBUG:save jpeg(%s, size(%d) is successed!!\n", filename, size);

    return 0;
}

int x3_dumpToFile(char *filename, char *srcBuf, unsigned int size)
{
    FILE *yuvFd = NULL;
    char *buffer = NULL;

    yuvFd = fopen(filename, "w+");

    if (yuvFd == NULL)
    {
        printf("ERRopen(%s) fail", filename);
        return -1;
    }

    buffer = (char *)malloc(size);

    if (buffer == NULL)
    {
        printf(":malloc file");
        fclose(yuvFd);
        return -1;
    }

    memcpy(buffer, srcBuf, size);
    fflush(stdout);
    fwrite(buffer, 1, size, yuvFd);
    fflush(yuvFd);

    if (yuvFd)
        fclose(yuvFd);
    if (buffer)
        free(buffer);

    printf("filedump(%s, size(%d) is successed\n", filename, size);

    return 0;
}