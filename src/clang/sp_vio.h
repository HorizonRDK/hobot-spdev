#ifndef SP_VIO_H_
#define SP_VIO_H_

#define SP_DEV_SIF 0
#define SP_DEV_ISP 1
#define SP_DEV_IPU 2
#define SP_VPS_SCALE 1
#define SP_VPS_SCALE_CROP 2
#define SP_VPS_SCALE_ROTATE 3
#define SP_VPS_SCALE_ROTATE_CROP 4
#define FRAME_BUFFER_SIZE(w, h) ((w) * (h) * (3) / (2))

#ifdef __cplusplus
extern "C"
{
#endif
        void *sp_init_vio_module();
        void sp_release_vio_module(void *obj);

        int sp_open_camera(void *obj, const int pipe_id, int chn_num, int *width, int *height);
        int sp_open_vps(void *obj, const int pipe_id, int chn_num, int proc_mode,
                        int src_width, int src_height, int *dst_width, int *dst_height,
                        int *crop_x, int *crop_y, int *crop_width, int *crop_height, int *rotate);
        int sp_vio_close(void *obj);
        int sp_vio_get_frame(void *obj, char *frame_buffer, int width, int height, const int timeout);
        int sp_vio_set_frame(void *obj, void *frame_buffer, int size);
        int sp_vio_get_raw(void *obj, char *frame_buffer, int width, int height, const int timeout);
        int sp_vio_get_yuv(void *obj, char *frame_buffer, int width, int height, const int timeout);

#ifdef __cplusplus
}
#endif /* End of #ifdef __cplusplus */

#endif /* SP_VIO_H_ */