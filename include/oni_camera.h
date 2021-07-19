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
// // point cloud
// #include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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
    // openni::CoordinateConverter oni_converter;

    //
    OBCameraParams cameraParams_;
    bool getCameraParams();
    // openni camera tools
    std::string depth_uri_str_;
    bool OpenOniCamera(const char* depth_uri);
    bool GetOniStreamData();
    // void createPointCloud();
    void seOnitLDP(bool enable);
    std::string enumerateDevices();
  //   void publishPointCloud(const cv::Mat& rgb_frame);
    char camera_loc_;
  //   // point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr head_cloud_;
  //   ros::Publisher pub_;
  //   void calculateDepthHistgram(float dist_x, BinArray& depth_histogram, DistBinArray& bin_depth_sum_mm);
};
// }  // namespace oni_camera
#endif