#ifndef SP_DISPLAY_H_
#define SP_DISPLAY_H_

#ifdef __cplusplus
extern "C"
{
#endif

    void *sp_init_display_module();
    void sp_release_display_module(void *obj);
    int32_t sp_start_display(void *obj, int32_t chn, int32_t width, int32_t height);
    int32_t sp_stop_display(void *obj);
    int32_t sp_display_set_image(void *obj, char *addr, int32_t size, int32_t chn);
    int32_t sp_display_draw_rect(void *obj, int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t chn, int32_t flush, int32_t color, int32_t line_width);
    int32_t sp_display_draw_string(void *obj, int32_t x, int32_t y, char *str, int32_t chn, int32_t flush, int32_t color, int32_t line_width);
    void sp_get_display_resolution(int32_t *width, int32_t *height);

#ifdef __cplusplus
}
#endif /* End of #ifdef __cplusplus */

#endif /* SP_DISPLAY_H_ */