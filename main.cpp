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
#define LINUX_X86
#ifndef LINUX_X86
#define LINUX_ARM
#endif

// for common
#include <iostream>
#include <thread>
// for camera driver
#include <oni_camera.h>
#include <uvc_camera.h>

// for pangolin visualization
#ifdef LINUX_X86
#include <viewer.h>
#endif

// for opencv
#include <opencv4/opencv2/opencv.hpp>

// for tflite
#include "common_helper_cv.h"
#include "image_processor.h"

/*** Macro ***/
#define WORK_DIR RESOURCE_DIR
#define DEFAULT_INPUT_IMAGE RESOURCE_DIR "/kite.jpg"
#define LOOP_NUM_FOR_TIME_MEASUREMENT 10

// Input camera bus number to choose a specific camera. You can use "lsusb" to check which bus your camera is on.
// https://www.cnblogs.com/avril/archive/2010/03/22/1691477.html
int main(int argc, char** argv) {
  printf("Orbbec camera driver!\n");
  printf("Chooseing camera on bus:%s\n", argv[1]);
  // RGB camera using UVC
  UVCCamera uvc_camera;
  uvc_camera.setParams();
  uvc_camera.openCamera();
  // Depth camera using OpneNI2
  OniCamera oni_camera;
  oni_camera.camera_loc_ = *argv[1];
  oni_camera.depth_uri_str_ = oni_camera.enumerateDevices();
  oni_camera.openCamera();
  oni_camera.seOnitLDP(false);

#ifdef LINUX_X86
  // Viewer for point cloud
  viewer::Viewer main_viewer(480, 640, "pcl");
  main_viewer.setInputCloud(oni_camera.point_cloud_);
  std::thread display_loop;
  display_loop = std::thread(&viewer::Viewer::run, &main_viewer);
#endif

  /************ TFLITE INIT **************/
  /*** Initialize ***/
  /* variables for processing time measurement */
  double total_time_all = 0;
  double total_time_cap = 0;
  double total_time_image_process = 0;
  double total_time_pre_process = 0;
  double total_time_inference = 0;
  double total_time_post_process = 0;
  auto tp = std::chrono::system_clock::now();
  std::time_t tt = std::chrono::system_clock::to_time_t(tp);
  std::cout << tt << " seconds from 1970-01-01 00:00:00 UTC" << std::endl;
  // /* Find source image */
  // std::string input_name = DEFAULT_INPUT_IMAGE;
  // cv::VideoCapture cap; /* if cap is not opened, src is still image */
  // if (!CommonHelper::FindSourceImage(input_name, cap)) {
  //   std::cout << "Can't find source image!" << std::endl;
  // }
  /* Initialize image processor library */
  ImageProcessor::InputParam input_param = {WORK_DIR, 4};
  if (ImageProcessor::Initialize(input_param) != 0) {
    printf("Initialization Error\n");
  }
  /*** Process for each frame ***/
  int32_t frame_cnt = 0;
  // for (frame_cnt = 0; cap.isOpened() || frame_cnt < LOOP_NUM_FOR_TIME_MEASUREMENT; frame_cnt++) {
  /* Display result */
  // cv::imshow("test", image);

  // /* Input key command */
  // if (cap.isOpened()) {
  //   /* this code needs to be before calculating processing time because cv::waitKey includes image output */
  //   /* however, when 'q' key is pressed (cap.released()), processing time significantly incraeases. So escape from the loop before calculating time
  //    */
  //   if (CommonHelper::InputKeyCommand(cap)) break;
  // };

  // /* Print processing time */
  // const auto& time_all1 = std::chrono::steady_clock::now();
  // double time_all = (time_all1 - time_all0).count() / 1000000.0;
  // double time_cap = (time_cap1 - time_cap0).count() / 1000000.0;
  // double time_image_process = (time_image_process1 - time_image_process0).count() / 1000000.0;
  // printf("Total:               %9.3lf [msec]\n", time_all);
  // printf("  Capture:           %9.3lf [msec]\n", time_cap);
  // printf("  Image processing:  %9.3lf [msec]\n", time_image_process);
  // printf("    Pre processing:  %9.3lf [msec]\n", result.time_pre_process);
  // printf("    Inference:       %9.3lf [msec]\n", result.time_inference);
  // printf("    Post processing: %9.3lf [msec]\n", result.time_post_process);
  // printf("=== Finished %d frame ===\n\n", frame_cnt);
  // Get depth data and show depth image.
  cv::namedWindow("raw_depth", 0);
  cv::namedWindow("rgb_img", 0);
  while (1) {
    // DEPTH
    oni_camera.GetOniStreamData();
    if (oni_camera.oni_depth_frame_.isValid()) {
      openni::DepthPixel* pDepth = (openni::DepthPixel*)oni_camera.oni_depth_frame_.getData();
      cv::Mat raw_depth(ONI_HEIGHT, ONI_WIDTH, CV_16UC1, pDepth);
      cv::flip(raw_depth, raw_depth, 1);
      cv::Mat tmp_depth;
      // Show raw_depth
      raw_depth.convertTo(tmp_depth, CV_8UC1, 1. / 2.);
      if (!tmp_depth.empty()) {
        cv::imshow("raw_depth", tmp_depth);
        cv::waitKey(1);
      }
      oni_camera.generatePointCloud(uvc_camera.raw_rgb_);
    }

    // RGB
    cv::Mat rgb_img = uvc_camera.getImage();
    if (!rgb_img.empty()) {
      const auto& time_all0 = std::chrono::steady_clock::now();
      const auto& time_image_process0 = std::chrono::steady_clock::now();
      ImageProcessor::Result result;
      ImageProcessor::Process(rgb_img, result);
      const auto& time_image_process1 = std::chrono::steady_clock::now();
      double time_image_process = (time_image_process1 - time_image_process0).count() / 1000000.0;
      // printf("Total:               %9.3lf [msec]\n", time_all);
      // printf("  Capture:           %9.3lf [msec]\n", time_cap);
      printf("  Image processing:  %9.3lf [msec]\n", time_image_process);
      // printf("    Pre processing:  %9.3lf [msec]\n", result.time_pre_process);
      // printf("    Inference:       %9.3lf [msec]\n", result.time_inference);
      // printf("    Post processing: %9.3lf [msec]\n", result.time_post_process);
      // printf("=== Finished %d frame ===\n\n", frame_cnt);
      cv::imshow("rgb_img", rgb_img);
      cv::waitKey(1);
    }

    usleep(100);
  }
  return 0;
}