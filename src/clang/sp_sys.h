#ifndef SP_SYS_H_
#define SP_SYS_H_
#define SP_MTYPE_VIO     0
#define SP_MTYPE_ENCODER 1
#define SP_MTYPE_DECODER 2
#define SP_MTYPE_DISPLAY 3
#ifdef __cplusplus
extern "C"
{
#endif
int32_t sp_module_bind(void *src, int32_t src_type, void *dst, int32_t dst_type);
int32_t sp_module_unbind(void *src, int32_t src_type, void *dst, int32_t dst_type);
#ifdef __cplusplus
}
#endif /* End of #ifdef __cplusplus */

#endif /* SP_SYS_H_ */