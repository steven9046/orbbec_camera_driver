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

#include <oni_camera.h>
#include <uvc_camera.h>
#include <viewer.h>

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <thread>  // 多线程

void test() {
  while (1) {
    printf("testing thread...\n");
    usleep(200);
  }
}
// main函数传入参数
// https://www.cnblogs.com/avril/archive/2010/03/22/1691477.html
int main(int argc, char **argv) {
  printf("hello!\n");
  if (*argv[1] == '1') {
    printf("Choosing HEAD camera.\n");
  } else if (*argv[1] == '5') {
    printf("Choosing GROUND camera.\n");
  }

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
  // static cv::Mat raw_depth(ONI_HEIGHT, ONI_WIDTH, CV_16UC1);
  // static cv::Mat tmp_depth;
  // viewer
  viewer::Viewer main_viewer(480, 640, "pcl");
  main_viewer.setInputCloud(oni_camera.point_cloud_);
  std::thread display_loop;
  display_loop = std::thread(&viewer::Viewer::run, &main_viewer);

  while (1) {
    oni_camera.GetOniStreamData();
    // 获取深度信息
    if (oni_camera.oni_depth_frame_.isValid()) {
      openni::DepthPixel *pDepth = (openni::DepthPixel *)oni_camera.oni_depth_frame_.getData();
      cv::Mat raw_depth(ONI_HEIGHT, ONI_WIDTH, CV_16UC1, pDepth);
      cv::flip(raw_depth, raw_depth, 1);
      cv::Mat tmp_depth;
      // ** DEBUG ** : show raw_depth
      raw_depth.convertTo(tmp_depth, CV_8UC1, 1. / 2.);
      if (!tmp_depth.empty()) {
        cv::namedWindow("raw_depth", 0);
        cv::imshow("raw_depth", tmp_depth);
        cv::waitKey(1);
      }
      oni_camera.generatePointCloud(uvc_camera.raw_rgb_);
    }
    usleep(100);
  }
  return 0;
}