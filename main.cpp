#include <oni_camera.h>
#include <uvc_camera.h>

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
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

  while (1) {
    oni_camera.GetOniStreamData();
    // 获取深度信息
    if (oni_camera.oni_depth_frame_.isValid()) {
      openni::DepthPixel *pDepth = (openni::DepthPixel *)oni_camera.oni_depth_frame_.getData();
      cv::Mat raw_depth(ONI_HEIGHT, ONI_WIDTH, CV_16UC1, pDepth);
      cv::flip(raw_depth, raw_depth, 1);
      cv::Mat tmp_depth;
      // // float px = 0, py = 0, pz = 0;
      // for(int j = 0; j < ONI_HEIGHT; j ++)
      // {
      //     for(int i = 0; i < ONI_WIDTH; i ++)
      //     {
      //         raw_depth.at<uint16_t>(j, i) = pDepth[j * ONI_WIDTH + i];
      //         // std::cout << pDepth[j * ONI_WIDTH + i] << std::endl;
      //     }
      // }
      // // ** DEBUG ** : show raw_depth
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