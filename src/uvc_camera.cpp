#include <uvc_camera.h>
#define libuvc_VERSION (libuvc_VERSION_MAJOR * 10000 + libuvc_VERSION_MINOR * 100 + libuvc_VERSION_PATCH)

UVCCamera::UVCCamera() : state_(kInitial), ctx_(NULL), dev_(NULL), devh_(NULL), rgb_frame_(NULL), strmhp_(NULL), param_init_(false) {
  // 初始化config_
  config_.vendor = "0x2bc5";
  config_.product = "0x050e";
  config_.serial = "";
  config_.index = 0;
  config_.width = 640;
  config_.height = 480;
  config_.video_mode = "mjpeg";
  config_.frame_rate = 30;
  config_.timestamp_method = "start";
  config_.camera_info_url = "file:///tmp/cam.yaml";
  config_.auto_exposure = 3;
  config_.auto_white_balance = false;

  raw_rgb_ = cv::Mat(config_.height, config_.width, CV_8UC3, cv::Scalar(0, 0, 0));
  init();
}

UVCCamera::~UVCCamera() {
  if (rgb_frame_) uvc_free_frame(rgb_frame_);
  if (ctx_) uvc_exit(ctx_);  // Destroys dev_, devh_, etc.
}

/**
 * @brief 初始化uvc环境
 */
bool UVCCamera::init() {
  // 假设成立，继续运行，确保开始时是初始化状态
  assert(state_ == kInitial);
  // 初始化uvc接口
  uvc_error_t err;
  err = uvc_init(&ctx_, NULL);
  if (err != UVC_SUCCESS) {
    uvc_perror(err, "ERROR: uvc_init");
    return false;
  }
  state_ = kStopped;
  return true;
}

 void UVCCamera::openCamera(){
  assert(state_ == kStopped);
  // 读取设备信息
  int vendor_id = strtol(config_.vendor.c_str(), NULL, 0);
  int product_id = strtol(config_.product.c_str(), NULL, 0);
  printf("Opening camera with vendor=0x%x, product=0x%x\n", vendor_id, product_id);
      // 设备指针的列表,按照id去搜寻设备
      uvc_device_t **devs;
  uvc_error_t find_err = uvc_find_devices(ctx_, &devs, vendor_id, product_id, config_.serial.empty() ? NULL : config_.serial.c_str());
  if (find_err != UVC_SUCCESS) {
    uvc_perror(find_err, "uvc_find_device");
    return;
  }
  // 按照bus号去找相机
  dev_ = NULL;
  int dev_idx = 0;
  while (devs[dev_idx] != NULL) {
    int bus_num = uvc_get_bus_number(devs[dev_idx]);
    printf("device bus number: %d", bus_num);
    if (bus_num == camera_loc_) {
      dev_ = devs[dev_idx];
    } else {
      uvc_unref_device(devs[dev_idx]);
    }
    dev_idx++;
  }
  // 打开指定bus的相机
  uvc_error_t open_err = uvc_open(dev_, &devh_);

  if (open_err != UVC_SUCCESS) {
    switch (open_err) {
      case UVC_ERROR_ACCESS:
        printf("Permission denied opening /dev/bus/usb/%03d/%03d\n", uvc_get_bus_number(dev_), uvc_get_device_address(dev_));
        break;
      default:
        printf("Can't open /dev/bus/usb/%03d/%03d: %s (%d)\n", uvc_get_bus_number(dev_), uvc_get_device_address(dev_), uvc_strerror(open_err),
               open_err);
        break;
    }
    uvc_unref_device(dev_);
    return;
  }

  if (open_err == UVC_SUCCESS) {
    printf("Successfully open camera!!");
  }

  // 获取数据流
  uvc_stream_ctrl_t ctrl;
  uvc_error_t mode_err =
      uvc_get_stream_ctrl_format_size(devh_, &ctrl, GetVideoMode(config_.video_mode), config_.width, config_.height, config_.frame_rate);
  if (mode_err != UVC_SUCCESS) {
    uvc_perror(mode_err, "uvc_get_stream_ctrl_format_size");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    printf("check video_mode/width/height/frame_rate are available\n");
    uvc_print_diag(devh_, NULL);
    return;
  }

  // 开启数据流并设置回调
  uvc_error_t stream_err = uvc_start_streaming(devh_, &ctrl, &UVCCamera::ImageCallbackAdapter, this, 0);
  if (stream_err != UVC_SUCCESS) {
    uvc_perror(stream_err, "uvc_start_streaming");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    return;
  }
  printf("successfully start rgb stream ! \n");

  // 如果rgb_frame有东西，那么先清空
  if (rgb_frame_) uvc_free_frame(rgb_frame_);
  // uvc去读取一个frame
  rgb_frame_ = uvc_allocate_frame(config_.width * config_.height * 3);
  assert(rgb_frame_);
  state_ = kRunning;
}

/* static */ void UVCCamera::ImageCallbackAdapter(uvc_frame_t *frame, void *ptr) {
  UVCCamera *driver = static_cast<UVCCamera *>(ptr);

  driver->ImageCallback(frame);
}

void UVCCamera::ImageCallback(uvc_frame_t *frame) {
  assert(state_ == kRunning);
  assert(rgb_frame_);
  // 这里是如何把数据存起来
}

enum uvc_frame_format UVCCamera::GetVideoMode(std::string vmode) {
  if (vmode == "uncompressed") {
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  } else if (vmode == "compressed") {
    return UVC_COLOR_FORMAT_COMPRESSED;
  } else if (vmode == "yuyv") {
    return UVC_COLOR_FORMAT_YUYV;
  } else if (vmode == "uyvy") {
    return UVC_COLOR_FORMAT_UYVY;
  } else if (vmode == "rgb") {
    return UVC_COLOR_FORMAT_RGB;
  } else if (vmode == "bgr") {
    return UVC_COLOR_FORMAT_BGR;
  } else if (vmode == "mjpeg") {
    return UVC_COLOR_FORMAT_MJPEG;
  } else if (vmode == "gray8") {
    return UVC_COLOR_FORMAT_GRAY8;
  } else {
    printf("Invalid Video Mode: %s\n", vmode);
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  }
};

void UVCCamera::closeCamera() {}

void UVCCamera::setParams() { printf("Set params for child class UVCCamera.\n"); }