#include <thread>
using namespace std;

#include "x3_sdk_codec.h"
#include "x3_sdk_display.h"
#include "x3_sdk_camera.h"
#include "sp_display.h"

using namespace srpy_cam;


void *sp_init_display_module()
{
    return new VPPDisplay(); 
}

void sp_release_display_module(void *obj)
{
    if (obj != NULL)
    {
        delete static_cast<VPPDisplay *>(obj);
    }
}

int32_t sp_start_display(void *obj, int32_t chn, int32_t width, int32_t height)
{
    if (obj != NULL)
        return static_cast<VPPDisplay *>(obj)->x3_vot_init(chn, width, height, VOT_OUTPUT_1920x1080, HB_VOT_OUTPUT_BT1120, width, height);
    return -1;
}

int32_t sp_stop_display(void *obj)
{
    if (obj != NULL)
        return static_cast<VPPDisplay *>(obj)->x3_vot_deinit();
    return -1;
}

int32_t sp_display_set_image(void *obj, char *addr, int32_t size, int32_t chn)
{
    if (obj != NULL)
        return static_cast<VPPDisplay *>(obj)->set_img(addr, size, chn);
    return -1;
}

int32_t sp_display_draw_rect(void *obj, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t chn, int32_t flush, int32_t color, int32_t line_width)
{
    if (obj != NULL)
        return static_cast<VPPDisplay *>(obj)->set_graph_rect(x0, y0, x1, y1, chn, flush, color, line_width);
    return -1;
}

int32_t sp_display_draw_string(void *obj, int32_t x, int32_t y, char *str, int32_t chn, int32_t flush, int32_t color, int32_t line_width)
{
    if (obj != NULL)
        return static_cast<VPPDisplay *>(obj)->set_graph_word(x, y, str, chn, flush, (uint32_t)color, line_width);
    return -1;
}

static int32_t exec_cmd_ex(const char *cmd, char* res, int32_t max)
{
    if(cmd == NULL || res == NULL || max <= 0)
            return -1; 
        
    FILE *pp = popen(cmd, "r");
    if(!pp) {
        printf("[Error] Cannot popen cmd: %s\n", cmd);
        return -1; 
    }

    int32_t length;
    char tmp[1024] = {0};

    length = max;
    if(max > 1024) length = 1024;

    while(fgets(tmp, length, pp) != NULL) {
        sscanf(tmp, "%s", res);
    }

    pclose(pp);

    return strlen(res);
}

void sp_get_display_resolution(int32_t *width, int32_t *height)
{
    char result[128], buff[8];
    char *split = NULL;
    memset(result, '\0', sizeof(result));
    exec_cmd_ex("get_hdmi_res", result, 1024);

    split = strchr(result, ',');
    if (split == NULL) {
        *width = 1920;
        *height = 1080;
        return;
    }
    memset(buff, 0, sizeof(buff));
    strncpy(buff, result, split-result);
    *height = atoi(buff);

    memset(buff, 0, sizeof(buff));
    strcat(buff, split+1);
    *width = atoi(buff);
}
