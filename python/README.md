English| [简体中文](./README_cn.md)

# VIO Python Interface
## Introduction
The python interface of VIO supports 3 modules including camera, encode, and decode.

## Compilation
Compile following the Ubuntu version of the image version, in the build directory of the Ubuntu source code
sudo ./ubuild_ut.sh
After compilation, an output directory will be generated, containing the library file libsrcampy.so.

## Execution
Copy scripts/test.py to the board:
export LD_LIBRARY_PATH='/usr/lib/hobot-srcampy/':$LD_LIBRARY_PATH
python3 test.py

## Interface Introduction
#### bind

/*! Bind two modules to automatically flow data
 *
 * @param src: Source data module
 * @param dst: Destination data module
 * @return Negative number indicates error, 0 indicates success.
 */
int bind(PyObject *src, PyObject *dst);

#### unbind

/*! Unbind two modules
 *
 * @param src: Source data module
 * @param dst: Destination data module
 * @return Negative number indicates error, 0 indicates success.
 */
int unbind(PyObject *src, PyObject *dst);

### Camera Section
libsrcampy.Camera:
#### open_cam

/*! Open the camera, after opening the internal data flow starts running
 *
 * @param[in] pipe_id: Pipe_id corresponding to the current pipeline
 * @param[in] video_index: Index corresponding to the camera
 *        0: ov8856, imx514    1: F37, ofilm0092    2: sc031gs    3: ov10635
 * @param[option] fps: Camera output frame rate
 * @param[option] width: Camera output width, default value depends on the specific camera; the type can also be a list type, indicating multiple channels with a maximum of 6
 * @param[option] height: Camera output height, default value depends on the specific camera; the type can also be a list type, indicating multiple channels with a maximum of 6
 * @return Negative number indicates error, 0 indicates success.
 */
int open_cam(int pipe_id, int video_index, int fps = 30,
             int width = 1920, int height = 1080);#### open_vps;
/*! Open VPS, which allows image scaling and other processing
 *
 * @param[in] pipe_id: the pipe_id to which the current pipeline belongs
 * @param[option] proc_mode: vps processing mode
          1. Scale    2. Scale + Crop   3. Scale + Rotate   4. Scale + Crop + Rotate
 * @param[option] src_width: input width of VPS
 * @param[option] src_height: input height of VPS
 * @param[option] dst_width: output width of scaling function, default output is same size as input; can also be of list type, indicating multiple channels, with up to 6 channels at most
 * @param[option] dst_height: output height of scaling function, default output is same size as input; can also be of list type, indicating multiple channels, with up to 6 channels at most
 * @param[option] crop_rect: cropping area for crop function, passed in as a list, when nested lists, each list represents a channel's cropping area, with up to 6 channels at most; [Note: cropping is done before scaling]
 * @param[option] rotate: rotation angle, can enable rotation for up to two channels
          0.0 degrees   1.90 degrees    2.180 degrees   3.270 degrees
 * @param[option] src_size: input width and height optimization, same functionality as src_width and src_height parameters, can be passed in as a list
  [1920,1080]
 * @param[option] dst_size: output width and height optimization, same functionality as dst_width and dst_height parameters, can be passed in as a list
  [[1920,1080], [1920, 1080]]
 * @return Negative value indicates error, 0 indicates success.
 */
int open_cam(int pipe_id, int video_index, int fps = 30,
             int width = 1920, int height = 1080);

#### get_img

/*! Retrieve image, needs to be called after open_cam or open_vps
 *
 * @param module[in]: retrieve image for the corresponding module    0: SIF    1: ISP    2: IPU
 * @return PyNoneType indicates error, PyBytesObeject indicates success.
 */
PyObject *get_img(int module = 2);

#### set_img

/*! Set the image to be processed, needs to be called after open_vps
 *
 * @param img[in]: image buffer to be processed
 * @return Negative value indicates error, 0 indicates success.
 */
int set_img(PyObject *img);

#### close_cam
/*! Close the camera
 *
 */
void close_cam();

### Encode section
libsrcampy.Encoder：
#### encode
/*! encode method of the encoding module, used for image encoding
```/**
 * @param[in] video_chn Corresponding encoder channel (0~31)
 * @param[in] type 1: H264 2: H265 3: MJPEG
 * @param[in] width Width of the encoded image
 * @param[in] height Height of the encoded image
 * @param[option] bits Default encoding rate is 8000, this parameter is optional and can use the default value (8000)
 * @return Negative number indicates error, 0 indicates success.
 */
int encode(int video_chn, int type,
           int width, int height, int bits = 8000);

#### encode_file
/*! The encode method of the encoding module, used for encoding images with user-input images
 *
 * @param[in] img YUV buffer that needs to be encoded, using the NV12 format
 */
int encode_file(PyObject *img);

#### get_img
/*! The get_img method of the encoding module, returns a Python object
 * The interface will release the buffer internally, no need for user to release it
 *
 * @return PyNoneType indicates error, PyBytesObeject indicates success.
 */
PyObject *get_img();

#### close
/*! The close method of the encoding module, closes the encoding module
 * @return Negative number indicates error, 0 indicates success.
 */
int close();

### Decode Part
libsrcampy.Decoder:
#### decode
/*! The decode method of the decoding module, used for decoding images
 *
 * @param[in] file File name to be decoded
 * @param[in] video_chn Corresponding decoder channel (0~31)
 * @param[in] type 1: H264 2: H265 3: MJPEG
 * @param[in] width Width of the decoded image
 * @param[in] height Height of the decoded image
 * @return Returns a list
 *         <1> Negative number indicates error, 0 indicates success.
 *         <2> Value indicates the number of frames in the current bitstream file.
 */
int decode(char *file, int video_chn, int type,
           int width, int height);

#### set_img/*! Method set_img of the decoding module, sets the buffer of the stream to be decoded
 * @param[in] img Image buffer to be decoded
 * @param[in] chn Decoder channel
 * @param[in] eos Decoder end flag
 *
 * @return A negative number indicates an error, 0 indicates success.
 */
 int set_img(PyObject *img, int chn = 0, int eos = 0);

#### get_img
/*! Method get_img of the decoding module, returns a Python object
 * This interface will release the buffer internally, no need for user release
 *
 * @return PyNoneType indicates an error, PyBytesObeject indicates success.
 */
 PyObject *get_img();

#### close
/*! Method close of the decoding module, closes the decoding module
 * @return A negative number indicates an error, 0 indicates success.
 */
 int close();

### Display section
libsrcampy.Display:
#### display
/*! Method display of the display module, used for initializing the display module
 *
 * @param[option] chn Display output layer, 0~1 for video layer, 2~3 for graphic layer
 * @param[in] width Width of the displayed image
 * @param[in] height Height of the displayed image
 * @param[option] vot_intf VOT default output interface VOT_OUTPUT_1920x1080
 * @param[option] vot_out_mode VOT default output mode HB_VOT_OUTPUT_BT1120
 * @return A negative number indicates an error, 0 indicates success.
 */
  int display(int chn = 0, int width = 1920, int height = 1080,
              int vot_intf = 0, int vot_out_mode = 1);

#### set_img
/*! Method set_img of the display module
 *
 * @param[in] img Image to be displayed
 * @param[option] chn Display output layer, 0~1 for video layer
 * @return A negative number indicates an error, 0 indicates success.
 */
 int set_img(PyObject *img, int chn = 0);

#### set_graph_rect
/*! Method set_graph_rect of the display module
 * 
 */``` 
* @param[in] x0 The x value of the first coordinate of the rectangle box
* @param[in] y0 The y value of the first coordinate of the rectangle box
* @param[in] x1 The x value of the second coordinate of the rectangle box
* @param[in] y1 The y value of the second coordinate of the rectangle box
* @param[option] chn Display output layer, 2-3 are graphic layers
* @param[option] flush Whether to clear the current graphic layer buffer
* @param[option] color Rectangle box color (color format is ARGB8888)
* @param[option] line_width Width of the rectangle box border
* @return Negative number indicates error, 0 indicates success.
*/
int set_graph_rect(int x0, int y0, int x1, int y1, int chn = 2,
    int flush = 0, int color = 0xffff0000, int line_width = 4);

#### set_graph_word
/*! Show module's set_graph_rect method
* @param[in] x The x value of the coordinate to draw the string
* @param[in] y The y value of the coordinate to draw the string
* @param[in] str The string to be drawn (encoded in GB2312)
* @param[option] chn Display output layer, 2-3 are graphic layers
* @param[option] flush Whether to clear the current graphic layer buffer
* @param[option] color String color (color format is ARGB8888)
* @param[option] line_width Width of the string border
* @return Negative number indicates error, 0 indicates success.
*/
int set_graph_word(int x, int y, char *str, int chn = 2,
    int flush = 0, uint32_t color = 0xffff0000, int line_width = 1);

#### close
/*! Show module's close method, close the display module
* @return Negative number indicates error, 0 indicates success.
*/
int close();
```