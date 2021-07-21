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

#ifndef _ONI_CAMERA_H_
#define _ONI_CAMERA_H_

// c++
#include <iostream>
#include <string>
// camera
#include <camera.h>
#include <openni2/OpenNI.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// openCV
#include <opencv4/opencv2/opencv.hpp>
// macros
#define ONI_WIDTH 320
#define ONI_HEIGHT 200
#define ONI_FPS 30
#define ONI_WAIT_TIMEOUT 200
#define XN_MODULE_PROPERTY_LDP_ENABLE 0x1080FFBE
#define RGB_REGISTERATION 0
#define RESOULTION_X 640
#define RESOULTION_Y 480
#define MM2M 0.001

// namespace oni_camera {
class OniCamera : public Camera {
 public:
  OniCamera();
  ~OniCamera();
  bool oniEnvironmentInitialize();
  void openCamera();
  void closeCamera();

  // openni camera menmebers
  openni::Status oni_rc_;
  openni::Device oni_device_;

  openni::VideoStream** oni_streams_;

  openni::VideoStream oni_depth_stream_;
  openni::VideoFrameRef oni_depth_frame_;
  openni::VideoMode oni_depthVideoMode_;

  openni::VideoStream oni_ir_stream_;
  openni::VideoFrameRef oni_ir_frame_;
  openni::VideoMode oni_irVideoMode_;
  // openni::CoordinateConverter oni_converter; // We don't use this openni API cause it is not accurate.
  OBCameraParams cameraParams_;

  /**
   * @brief Get camera params which have been calibrated in the factory by orbbec. RGB camera params and depth camera params.
   */
  bool getCameraParams();

  // openni camera tools
  /**
   * @brief Enumerate devices plugged in. And find the desired camera by input bus number.
   */
  std::string enumerateDevices();

  /**
   * @brief Open orbbec camera and start depth stream(we did't start the IR stream currentlly).
   * @param depth_uri The uri got from enumerateDevices.
   */
  bool OpenOniCamera(const char* depth_uri);

  /**
   * @brief Get frame data from data stream.
   */
  bool GetOniStreamData();

  /**
   * @brief Turn on/off LDP. If LDP is on, the projector will turn off if somethin is too close to the camera.
   * @param enable On/Off
   */
  void seOnitLDP(bool enable);

  /**
   * @brief Generate point cloud using PCL.
   * @param rgb_frame We can register point cloud with a rgb image.
   */
  void generatePointCloud(const cv::Mat& rgb_frame);

  char camera_loc_;                                     // camera bus loc
  std::string depth_uri_str_;                           // camera uri: 2bc5/060e@1/6, the last number is device number, it will varies when reboot.
  float fdx_, fdy_, u0_, v0_;                           // camera params
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_;  // point cloud generated. need to init or will crash.
};
// }  // namespace oni_camera
#endif