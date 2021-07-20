#include <oni_camera.h>

using namespace openni;

OniCamera::OniCamera() {
  printf("Creating OniCamera instance.\n");
  // we can init something here
  oniEnvironmentInitialize();
}

OniCamera::~OniCamera() {}

bool OniCamera::oniEnvironmentInitialize() {
  oni_rc_ = OpenNI::initialize();
  printf("Oni environment initializing...\n");
  if (oni_rc_ != STATUS_OK) {
    printf("Oni environment initialize failed.\n");
    std::cout << OpenNI::getExtendedError() << std::endl;
    return false;
  }
  printf("Oni environment initialize success!\n");
  return true;
}

void OniCamera::openCamera() {
  printf("going to open oni camera.\n");
  OpenOniCamera(depth_uri_str_.c_str());
  getCameraParams();
  point_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
}

void OniCamera::closeCamera() { printf("oni camera.\n"); }

// get camera params
bool OniCamera::getCameraParams() {
  int data_size = sizeof(cameraParams_);
  printf("data size : %d", data_size);
  memset(&cameraParams_, 0, sizeof(cameraParams_));
  Status rc = oni_device_.getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t *)&cameraParams_, &data_size);
  if (!rc) {
    std::cout << "successfully get camera params:"
              << "k[0]: " << cameraParams_.l_k[0] << "\n"
              << "k[1]: " << cameraParams_.l_k[1] << "\n"
              << "k[2]: " << cameraParams_.l_k[2] << "\n"
              << "k[3]: " << cameraParams_.l_k[3] << "\n"
              << "k[4]: " << cameraParams_.l_k[4] << "\n"
              << "k[5]: " << cameraParams_.l_k[5] << std::endl;
    std::cout << "successfully get camera params:"
              << "r_intr_p[0]: " << cameraParams_.r_intr_p[0] << "\n"
              << "r_intr_p[1]: " << cameraParams_.r_intr_p[1] << "\n"
              << "r_intr_p[2]: " << cameraParams_.r_intr_p[2] << "\n"
              << "r_intr_p[3]: " << cameraParams_.r_intr_p[3] << std::endl;
    return true;
  } else {
    std::cout << "get camera params failed" << std::endl;
    return false;
  }
}

/** steps to open an openni camera
 *  1. initialize openni
 *  2. assert a device and open
 *  3. assert a depth stream and start
 */
/** 1. must create IR stream first, or you can only get depth stream
 *  2. Ir stream rate is larger than Depth
 */
bool OniCamera::OpenOniCamera(const char *depth_uri) {
  // 1. connect to openni camera
  std::cout << "going to open: " << depth_uri << std::endl;
  printf("going to open: %s \n", depth_uri);
  std::string uri = depth_uri;
  oni_rc_ = oni_device_.open(depth_uri);
  if (oni_rc_ != STATUS_OK) {
    printf("Couldn't open orbbec device\n");
    // printf(OpenNI::getExtendedError());
    return false;
  }
  // 2.create and start depth stream
  if (oni_device_.getSensorInfo(SENSOR_DEPTH) != NULL) {
    oni_rc_ = oni_depth_stream_.create(oni_device_, SENSOR_DEPTH);
    if (oni_rc_ != STATUS_OK) {
      printf("Couldn't create depth stream\n");
      return false;
    }

    // 重新设置帧率
    oni_depthVideoMode_ = oni_depth_stream_.getVideoMode();
    oni_depthVideoMode_.setResolution(ONI_WIDTH, ONI_HEIGHT);
    oni_depthVideoMode_.setFps(ONI_FPS);
    printf("Setting resolution width: %d, height: %d \n", ONI_WIDTH, ONI_HEIGHT);
    printf("Setting FPS : %d \n", ONI_FPS);
    oni_depth_stream_.setVideoMode(oni_depthVideoMode_);

    oni_rc_ = oni_depth_stream_.start();
    if (oni_rc_ != STATUS_OK) {
      printf("Couldn't start depth stream\n");
      // TEMI_LOG(error) << OpenNI::getExtendedError();
      return false;
    }
  }

  // 2.create and start ir stream
  if (oni_device_.getSensorInfo(SENSOR_IR) != NULL) {
    oni_rc_ = oni_ir_stream_.create(oni_device_, SENSOR_IR);
    if (oni_rc_ != STATUS_OK) {
      printf("Couldn't create depth stream\n");
      // TEMI_LOG(error) << OpenNI::getExtendedError();
      return false;
    }

    // 重新设置帧率
    oni_irVideoMode_ = oni_ir_stream_.getVideoMode();
    oni_irVideoMode_.setResolution(ONI_WIDTH, ONI_HEIGHT);
    oni_irVideoMode_.setFps(ONI_FPS / 2);
    printf("Setting resolution width: %d, height: %d\n", ONI_WIDTH, ONI_HEIGHT);
    printf("Setting FPS : %d\n", ONI_FPS);
    oni_ir_stream_.setVideoMode(oni_irVideoMode_);
    // // If you wan't to start IR stream
    // oni_rc_ = oni_ir_stream_.start();
    // if(oni_rc_ != STATUS_OK)
    // {
    //   printf("Couldn't start depth stream\n");
    //   // TEMI_LOG(error) << OpenNI::getExtendedError();
    //   return false;
    // }
  }

  // if (oni_rc_ != STATUS_OK)
  // {
  //   ROS_ERROR("Couldn't start the depth stream");
  //   // ROS_ERROR(OpenNI::getExtendedError());
  //   return false;
  // }
  // point_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  return true;
}

std::string OniCamera::enumerateDevices() {
  Array<DeviceInfo> deviceList;
  // TEMI_LOG(info) << "Going to enumerateDevices...";
  OpenNI::enumerateDevices(&deviceList);
  std::cout << deviceList.getSize() << "  devices are found" << std::endl;
  // ROS_INFO("％d evices are found", deviceList.getSize());
  // TEMI_LOG(info)<< deviceList.getSize() << " devices are found.";
  std::string depth_uri_;
  for (int i = 0; i != deviceList.getSize(); i++) {
    const openni::DeviceInfo &info = deviceList[i];
    std::string uri = info.getUri();
    // 下边这些属性都是一样的
    // Device Name: Astra
    // Device Vendor: Orbbec
    // Device Product ID: 1550
    // Device Vendor ID: 11205
    std::cout << "Device Name: " << info.getName() << std::endl;
    std::cout << "Device Vendor: " << info.getVendor() << std::endl;
    std::cout << "Device Product ID: " << info.getUsbProductId() << std::endl;
    std::cout << "Device Vendor ID: " << info.getUsbVendorId() << std::endl;
    std::cout << "Device URI: " << uri << std::endl;  // uri就是一个字符串
    if (uri.at(10) == camera_loc_) {
      printf("Catch camera!\n");
      // ROS_INFO("Device URI: %s", uri);//uri就是一个字符串
      std::cout << "Device URI:" << uri << std::endl;
      depth_uri_ = uri;  //.c_str();
      // ROS_INFO("depth_uri_: %s", depth_uri_);
    }
  }
  if (depth_uri_.empty()) {
    depth_uri_ = ANY_DEVICE;
  }
  return depth_uri_;
}

// set LDP on/off
void OniCamera::seOnitLDP(bool enable) {
  int data_size = 4;
  int enable_ = 1;
  if (enable == false) {
    enable_ = 0;
  }

  oni_depth_stream_.stop();
  oni_device_.setProperty(XN_MODULE_PROPERTY_LDP_ENABLE, (uint8_t *)&enable_, 4);
  oni_depth_stream_.start();
}

bool OniCamera::GetOniStreamData() {
  oni_depth_stream_.readFrame(&oni_depth_frame_);
  return true;
}

void OniCamera::generatePointCloud(const cv::Mat &rgb_frame) {
  // step1:init a blank point cloud whith colors.
  static int seq = 0;
  // clear and set point cloud
  point_cloud_->clear();
  point_cloud_->header.frame_id = "/base_link";
  point_cloud_->is_dense = false;
  // point_cloud_->header.stamp = ros::Time::now().toSec();
  point_cloud_->header.seq = seq++;
  pcl::PointXYZRGB point;
  int invalid_close_pixels_count = 0;
  int valid_points_counter = 0;
  pcl::PointXYZRGB invalid_point;
  invalid_point.x = invalid_point.y = invalid_point.z = std::numeric_limits<float>::quiet_NaN();
  invalid_point.r = invalid_point.g = invalid_point.b = 0;  // black

  // step3:generate point cloud
  openni::DepthPixel *pDepth = (openni::DepthPixel *)oni_depth_frame_.getData();
  float px = 0, py = 0, pz = 0;
  static int frame_width = oni_depth_frame_.getWidth();
  static int frame_height = oni_depth_frame_.getHeight();
  static float fdx = cameraParams_.r_intr_p[0] * ((float)(frame_width) / RESOULTION_X);
  static float fdy = cameraParams_.r_intr_p[1] * ((float)(frame_height) / RESOULTION_Y);
  static float u0 = cameraParams_.r_intr_p[2] * ((float)(frame_width) / RESOULTION_X);
  static float v0 = cameraParams_.r_intr_p[3] * ((float)(frame_height) / RESOULTION_Y);
  // BinArray depth_histogram = { 0 };
  // DistBinArray bin_depth_sum_mm = { 0.};
  for (int j = 0; j < ONI_HEIGHT; j++) {           // rows -> j
    for (int i = 0; i < ONI_WIDTH; i++) {          // cols ->i
                                                   // if (i >= 80 && i <= 240 && j >= 50 && j <= 150)
                                                   // {
      uint16_t depth = pDepth[j * ONI_WIDTH + i];  // get depth from frame

      if (depth < 200) {
        invalid_close_pixels_count++;
        point_cloud_->points.emplace_back(invalid_point);
      } else {
        //　这个函数把深度值转换为相机坐标系里的px,py,pz,相当于相机模型的作用
        // oni_rc_ = oni_converter.convertDepthToWorld(oni_depth_stream_, i, j, pDepth[j * ONI_WIDTH + i], &px, &py, &pz);

        // convert from pixel to camera
        float tx = (i - u0) / fdx;
        float ty = (j - v0) / fdy;
        px = depth * tx * MM2M;
        py = depth * ty * MM2M;
        pz = depth * MM2M;
        // 因为depth的tf和正常的相机tf不一样，是和baselink一样的，需要再转换一下
        // 这里z(py)也要翻转一下，因为我们的相机是倒着安的
        // tf::Vector3 point_vec = tf::Vector3(pz * 0.001, px * 0.001, py * 0.001);
        // tf::Vector3 point_vec = tf::Vector3(px * 0.001, py * 0.001, pz * 0.001);

        // if (point_vec.x() > 0.3 && point_vec.x() < 0.7)
        // {
        //   dist_1++;
        // }
        // else if (point_vec.x() > 1.3 && point_vec.x() < 1.7)
        // {
        //   dist_2++;
        // }
        // else if (point_vec.x() > 2.3 && point_vec.x() < 2.7)
        // {
        //   dist_3++;
        // }
        // avg_x += point_vec.x();
        // valid_points_counter++;
        // calculateDepthHistgram(point_vec.x(), depth_histogram, bin_depth_sum_mm);
        // point.x = point_vec.x();
        // point.y = point_vec.y();
        // point.z = point_vec.z();
        point.x = px;
        point.y = py;
        point.z = pz;
        point.r = 255;
        point.g = 255;
        point.b = 255;
        // step4.1:register point cloud with RGB(not activated now)
        // #if RGB_REGISTERATION
        //           // 对应的rgb中的点
        //           cv::Vec2i rgb_pixel = d2r.at<cv::Vec2i>(j, i);
        //           bool flag_1 = (0 <= i && i < 300);
        //           bool flag_2 = (0 <= j && j < 300);

        //           // TEMI_LOG(info) << "i :" << i << " in range 300?" << flag_1;
        //           // TEMI_LOG(info) << "j :" << j << " in range 300?" << flag_2;
        //           bool valid_index = (0 <= i && i < 300) && (0 <= j && j < 300);
        //           // TEMI_LOG(info) << "depth_pixel in range(300x300)?" << valid_index;
        //           if (valid_index) //如果depth点的索引在0-300之间(320X240,有超出范围的)
        //           {
        //             // TEMI_LOG(info) << "painting point ...";
        //             // TEMI_LOG(info) << "rgb_pixel x:" << rgb_pixel[1] << " y: " << rgb_pixel[0];
        //             if (rgb_pixel[0] == 0 && rgb_pixel[1] == 0) //depth point has no rgb point assigned,mark it as invalid.
        //             {
        //               invalid_close_pixels_count++;
        //               point_cloud_->points.emplace_back(invalid_point);
        //               continue;
        //             }
        //             else
        //             {
        //               int r = rgb_frame.at<cv::Vec3b>(rgb_pixel[0], rgb_pixel[1])[2];
        //               int g = rgb_frame.at<cv::Vec3b>(rgb_pixel[0], rgb_pixel[1])[1];
        //               int b = rgb_frame.at<cv::Vec3b>(rgb_pixel[0], rgb_pixel[1])[0];
        //               // TEMI_LOG(info) << "r:" << r;
        //               // TEMI_LOG(info) << "g:" << g;
        //               // TEMI_LOG(info) << "b:" << b;
        //               point.r = r;
        //               point.g = g;
        //               point.b = b;
        //             }
        //           }
        //           else
        //           {
        //             // TEMI_LOG(info) << "point index out of 300, paint this point black. ";
        //             invalid_close_pixels_count++;
        //             point_cloud_->points.emplace_back(invalid_point);
        //             continue;
        //           }
        // #endif
        point_cloud_->points.emplace_back(point);
      }
      // } //end else
    }  // innner loop
  }    // outerloop
}
