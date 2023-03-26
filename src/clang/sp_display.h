#ifndef SP_DISPLAY_H_
#define SP_DISPLAY_H_

#ifdef __cplusplus
extern "C"
{
#endif

    void *sp_init_display_module();
    void sp_release_display_module(void *obj);
    int sp_start_display(void *obj, int chn, int width, int height);
    int sp_stop_display(void *obj);
    int sp_display_set_image(void *obj, char *addr, int size, int chn);
    int sp_display_draw_rect(void *obj, int x0, int y0, int x1, int y1, int chn, int flush, int color, int line_width);
    int sp_display_draw_string(void *obj, int x, int y, char *str, int chn, int flush, int color, int line_width);
    void sp_get_display_resolution(int *width, int *height);

#ifdef __cplusplus
}
#endif /* End of #ifdef __cplusplus */

#endif /* SP_DISPLAY_H_ */