#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>

#include "x3_sdk_display.h"

#include "sp_display.h"

using namespace std;
using namespace srpy_cam;


void *sp_init_display_module()
{
    return new SrPyDisplay(); 
}

void sp_release_display_module(void *obj)
{
    if (obj != NULL)
    {
        delete static_cast<SrPyDisplay *>(obj);
    }
}

int sp_start_display(void *obj, int chn, int width, int height)
{
    if (obj != NULL)
        return static_cast<SrPyDisplay *>(obj)->x3_vot_init(chn, width, height, 0, 0, width, height);
    return -1;
}

int sp_stop_display(void *obj)
{
    if (obj != NULL)
        return static_cast<SrPyDisplay *>(obj)->x3_vot_deinit();
    return -1;
}

int sp_display_set_image(void *obj, char *addr, int size, int chn)
{
    if (obj != NULL)
        return static_cast<SrPyDisplay *>(obj)->set_img(addr, size, chn);
    return -1;
}

int sp_display_draw_rect(void *obj, int x0, int y0, int x1, int y1, int chn, int flush, int color, int line_width)
{
    if (obj != NULL)
        return static_cast<SrPyDisplay *>(obj)->set_graph_rect(x0, y0, x1, y1, chn, flush, color, line_width);
    return -1;
}

int sp_display_draw_string(void *obj, int x, int y, char *str, int chn, int flush, int color, int line_width)
{
    if (obj != NULL)
        return static_cast<SrPyDisplay *>(obj)->set_graph_word(x, y, str, chn, flush, (uint32_t)color, line_width);
    return -1;
}

static int exec_cmd_ex(const char *cmd, char* res, int max)
{
    if(cmd == NULL || res == NULL || max <= 0)
            return -1; 
        
    FILE *pp = popen(cmd, "r");
    if(!pp) {
        printf("[Error] Cannot popen cmd: %s\n", cmd);
        return -1; 
    }

    int length;
    char tmp[1024] = {0};

    length = max;
    if(max > 1024) length = 1024;

    while(fgets(tmp, length, pp) != NULL) {
        sscanf(tmp, "%s", res);
    }

    pclose(pp);

    return strlen(res);
}

void sp_get_display_resolution(int *width, int *height)
{
    char result[128], buff[8];
    char *split = NULL;
    memset(result, '\0', sizeof(result));
    exec_cmd_ex("8618_get_edid", result, 1024);

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
