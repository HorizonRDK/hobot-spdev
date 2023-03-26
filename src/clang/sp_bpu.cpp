#include <stdint.h>
#include "dnn/hb_sys.h"
#include "dnn/hb_dnn.h"
#include "x3_bpu.h"
#include "sp_bpu.h"
bpu_module *sp_init_bpu_module(const char *model_file_name)
{
    auto bpu_handle = x3_bpu_predict_init(model_file_name);
    return bpu_handle;
}

int sp_bpu_start_predict(bpu_module *bpu_handle, char *addr)
{
    if (bpu_handle)
    {
        return x3_bpu_start_predict(bpu_handle, addr);
    }
    return -1;
}

int sp_release_bpu_module(bpu_module *bpu_handle)
{
    if (bpu_handle)
    {
        return x3_bpu_predict_unint(bpu_handle);
    }
    return -1;
}
int sp_init_bpu_tensors(bpu_module *bpu_handle, hbDNNTensor *output_tensors)
{
    if (bpu_handle)
    {
        return x3_bpu_init_tensors(bpu_handle, output_tensors);
    }
    return -1;
}

int sp_deinit_bpu_tensor(hbDNNTensor *tensor, int len)
{
    if (tensor)
    {
        return x3_bpu_deinit_tensor(tensor, len);
    }
    return -1;
}