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

#include <camera.h>
#include <openni2/OpenNI.h>
#include <iostream>
#include <string>

#define ONI_WIDTH 320
#define ONI_HEIGHT 200
#define ONI_FPS 30
#define ONI_WAIT_TIMEOUT 200
#define XN_MODULE_PROPERTY_LDP_ENABLE 0x1080FFBE
#define RGB_REGISTERATION 0
#define RESOULTION_X 640
#define RESOULTION_Y 480
#define MM2M 0.001
// // point cloud
// #include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv4/opencv2/opencv.hpp>

// #include <pcl/filters/extract_indices.h>
// //ground camera tf calibraiton
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/segmentation/extract_clusters.h>
// namespace oni_camera {
class OniCamera : public Camera {
 public:
  OniCamera();
  ~OniCamera();
  bool oniEnvironmentInitialize();
  void openCamera();
  void closeCamera();

    // openni camera
    openni::Status oni_rc_;
    openni::Device oni_device_;

    openni::VideoStream** oni_streams_;

    openni::VideoStream oni_depth_stream_;
    openni::VideoFrameRef oni_depth_frame_;
    openni::VideoMode oni_depthVideoMode_;

    openni::VideoStream oni_ir_stream_;
    openni::VideoFrameRef oni_ir_frame_;
    openni::VideoMode oni_irVideoMode_;

    //
    openni::CoordinateConverter oni_converter;

    //
    OBCameraParams cameraParams_;
    bool getCameraParams();
    // openni camera tools
    std::string depth_uri_str_;
    bool OpenOniCamera(const char* depth_uri);
    bool GetOniStreamData();
    void seOnitLDP(bool enable);
    std::string enumerateDevices();
    void generatePointCloud(const cv::Mat& rgb_frame);
    char camera_loc_;
    float fdx_, fdy_, u0_, v0_;
  //   // point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_;
  //   ros::Publisher pub_;
  //   void calculateDepthHistgram(float dist_x, BinArray& depth_histogram, DistBinArray& bin_depth_sum_mm);
};
// }  // namespace oni_camera
#endif