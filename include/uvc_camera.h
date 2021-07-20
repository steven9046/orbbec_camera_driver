/*****************************************************************************
*                                                                            *
*  Copyright (C) 2021 Steven Sun.                                        *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/

#ifndef _UVC_CAMERA_H_
#define _UVC_CAMERA_H_

#include <camera.h>
#include <lib_uvc/UVCCameraConfig.h>
#include <lib_uvc/libuvc.h>
#include <opencv4/opencv2/opencv.hpp>

// #include <cmath>

enum State {
  kInitial = 0,
  kStopped = 1,
  kRunning = 2,
};

class UVCCamera : public Camera {// 
 public:
  UVCCamera();
  ~UVCCamera();
  //   bool getRGBImage();
  bool start();
  void Stop();
  UVCCameraConfig config_;
  cv::Mat raw_rgb_;
  int camera_loc_;



  bool init();
  static void ImageCallbackAdapter(uvc_frame_t *frame, void *ptr);
  void ImageCallback(uvc_frame_t *frame);
  enum uvc_frame_format GetVideoMode(std::string mode);
  void setParams();
  void openCamera();
  void closeCamera();
 private:

  State state_;
  uvc_context_t *ctx_;
  uvc_device_t *dev_;
  uvc_device_handle_t *devh_;
  uvc_frame_t *rgb_frame_;
  uvc_stream_handle_t *strmhp_;
  bool param_init_;
};

#endif