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

enum State {
  kInitial = 0,
  kStopped = 1,
  kRunning = 2,
};

class UVCCamera : public Camera {  //
 public:
  UVCCamera();
  ~UVCCamera();

  /**
   * @brief init uvc environment
   */
  bool init();

  /**
   * @brief Callback adapter
   */
  static void ImageCallbackAdapter(uvc_frame_t *frame, void *ptr);

  /**
   * @brief Image callback func. Mainly does some format transfer and data copy work.
   * @param frame input uvc frame
   */
  void ImageCallback(uvc_frame_t *frame);

  /**
   * @brief Convert mode string to macros.
   * @param mode input mode string
   */
  enum uvc_frame_format GetVideoMode(std::string mode);

  /**
   * @brief Set camera params like resulation...
   */
  void setParams();

  /**
   * @brief Open uvc camera.
   */
  void openCamera();

  /**
   * @brief Close uvc camera and destroy environment.
   */
  void closeCamera();

  /**
   * @brief Get rgb image from uvc_camera 
   */
  cv::Mat getImage();
  
  UVCCameraConfig config_;  // Camera config
  cv::Mat raw_rgb_;         // store rgb image
  int camera_loc_;          // camera bus number
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