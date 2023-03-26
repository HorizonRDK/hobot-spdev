# vio python接口
## 介绍
vio的python接口支持，包含 camera、encode、decode 3个模块

## 编译
跟随ubuntu版本镜像版本编译, 在ubuntu源码build目录下
sudo ./ubuild_ut.sh
编译之后会生成output目录，会包含libhbspdev.so库文件

## 运行
将scripts/test.py拷贝至板子中：
export LD_LIBRARY_PATH='/usr/lib/hobot-spdev/':$LD_LIBRARY_PATH
python3 test.py

## 接口介绍
#### bind

/*! 绑定两个模块，使其数据流自动流转
 *
 * @param src: 源数据模块
 * @param dst: 目标数据模块
 * @return 负数表示错误 0表示成功.
 */
int bind(PyObject *src, PyObject *dst);
#### unbind

/*! 解绑两个模块
 *
 * @param src: 源数据模块
 * @param dst: 目标数据模块
 * @return 负数表示错误 0表示成功.
 */
int unbind(PyObject *src, PyObject *dst);

### Camera部分
libhbspdev.Camera：
#### open_cam;
/*! 打开camera，打开后内部数据流开始运转
 *
 * @param pipe_id：当前pipeline所属pipe_id
 * @param video_index 对应camera的索引 
 *        0:ov8856,imx514    1:F37,ofilm0092    2:sc031gs    3:ov10635
 * @param fps camera输出帧率
 * @param width camera输出宽，默认值与具体camera有关；类型也可为列表类型，表示多开通道，最多开6个
 * @param height camera输出高，默认值与具体camera有关；类型也可为列表类型，表示多开通道，最多开6个
 * @return 负数表示错误 0表示成功.
 */
int open_cam(int pipe_id, int video_index, int fps = 30,
             int width = 1920, int height = 1080);

#### get_img

/*! 获取图像，需要在open_cam之后调用
 *
 * @param module：获取对应模块的图像    0：SIF    1：ISP    2：IPU
 * @return 负数表示错误 0表示成功.
 */
PyObject *get_img(int module = 2);

#### close_cam
/*! 关闭camera
 *
 */
void close_cam();

### Encode部分
libhbspdev.Encoder：
#### encode
/*! 编码模块的encode方法，用于图像的编码
 *
 * @param[in] video_chn 对应的编码器通道（0~31）
 * @param[in] type 1: H264 2: H265 3: MJPEG
 * @param[in] width 编码的图像宽度
 * @param[in] height 编码的图像高度
 * @param[option] bits 默认编码速率为8000，该参数为可选参数，可以使用默认值（8000）
 * @return -1表示错误 0表示成功.
 */
  int encode(int video_chn, int type,
             int width, int height, int bits = 8000);

#### encode_file
/*! 编码模块的encode方法，用于图像的编码, 用户主动输入图片用作编码
 *
 * @param[in] img 需要编码的YUV buffer，需要使用NV12格式
 */
  int encode_file(PyObject *img);

#### get_img
/*! 编码模块的get_img方法，返回python对象
 * 该接口内部会释放buffer，无需用户释放
 *
 * @return 成功返回python对象，失败返回NULL.
 */
 PyObject *get_img();

#### close
/*! 编码模块的close方法，关闭编码模块
 * @return -1表示错误 0表示成功.
 */
 int close();

### Decode部分
libhbspdev.Decoder：
#### decode
/*! 解码模块的decode方法，用于图像的解码
 *
 * @param[in] file 需要解码的文件名
 * @param[in] video_chn 对应的解码器通道（0~31）
 * @param[in] type 1: H264 2: H265 3: MJPEG
 * @param[in] width 解码的图像宽度
 * @param[in] height 解码的图像高度
 * @return 返回一个list
 *         <1> -1表示错误 0表示成功.
 *         <2> 数值表示当前码流文件帧数
 */
  int decode(char *file, int video_chn, int type,
             int width, int height);

#### set_img
/*! 解码模块的set_img方法，设置需要解码的码流buffer
 * @param[in] img 解码的图像buffer
 * @param[in] chn 解码器通道
 * @param[in] eos 解码器结束标志
 *
 * @return 成功返回python对象，失败返回NULL.
 */
 int set_img(PyObject *img, int chn = 0, int eos = 0);

#### get_img
/*! 解码模块的get_img方法，返回python对象
 * 该接口内部会释放buffer，无需用户释放
 *
 * @return 成功返回python对象，失败返回NULL.
 */
 PyObject *get_img();

#### close
/*! 解码模块的close方法，关闭解码模块
 * @return -1表示错误 0表示成功.
 */
 int close();

 ### Display部分
libhbspdev.Display：
#### display
/*! 显示模块的display方法，用于显示模块的初始化
 *
 * @param[option] chn 显示输出层，0~1为video层，2~3为图形层
 * @param[in] width 显示的图像宽度
 * @param[in] height 显示的图像高度
 * @param[option] vot_intf vot默认输出接口VOT_OUTPUT_1920x1080
 * @param[option] vot_out_mode vot默认输出模式HB_VOT_OUTPUT_BT1120
 * @return -1表示错误 0表示成功.
 */
  int display(int chn = 0, int width = 1920, int height = 1080,
              int vot_intf = 0, int vot_out_mode = 1);

#### set_img
/*! 显示模块的set_img方法
 *
 * @param[in] img 需要显示的图像
 * @param[option] chn 显示输出层，0~1为video层
 * @return -1表示错误 0表示成功.
 */
 int set_img(PyObject *img, int chn = 0);

#### set_graph_rect
/*! 显示模块的set_graph_rect方法
 *
 * @param[in] x0 绘制矩形框第一个坐标的x值
 * @param[in] y0 绘制矩形框第一个坐标的y值
 * @param[in] x1 绘制矩形框第二个坐标的x值
 * @param[in] y1 绘制矩形框第二个坐标的y值
 * @param[option] chn 显示输出层，2~3为图形层
 * @param[option] flush 是否清零当前图形层buffer
 * @param[option] color 矩形框颜色（颜色格式为ARGB8888）
 * @param[option] line_width 矩形框边的宽度
 * @return -1表示错误 0表示成功.
 */
int set_graph_rect(int x0, int y0, int x1, int y1, int chn = 2,
    int flush = 0, int color = 0xffff0000, int line_width = 4);

#### set_graph_word
/*! 显示模块的set_graph_rect方法
 *
 * @param[in] x 绘制字符串坐标的x值
 * @param[in] y 绘制字符串坐标的y值
 * @param[in] str 需要绘制的字符串（需要是GB2312编码）
 * @param[option] chn 显示输出层，2~3为图形层
 * @param[option] flush 是否清零当前图形层buffer
 * @param[option] color 字符串颜色（颜色格式为ARGB8888）
 * @param[option] line_width 字符串边的宽度
 * @return -1表示错误 0表示成功.
 */
int set_graph_word(int x, int y, char *str, int chn = 2,
        int flush = 0, uint32_t color = 0xffff0000, int line_width = 1);

#### close
/*! 显示模块的close方法，关闭显示模块
 * @return -1表示错误 0表示成功.
 */
 int close();