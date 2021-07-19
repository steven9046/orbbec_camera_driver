#include <oni_camera.h>

using namespace openni;

OniCamera::OniCamera() {
  printf("Creating OniCamera instance.\n");
  // we can init something here
  oniEnvironmentInitialize();
}

OniCamera::~OniCamera() {}

bool OniCamera::oniEnvironmentInitialize()
{
  oni_rc_ = OpenNI::initialize();
  printf("Oni environment initializing...\n");
  if (oni_rc_ != STATUS_OK)
  {
    printf("Oni environment initialize failed.\n");
    std::cout << OpenNI::getExtendedError() << std::endl;
    return false;
  }
  printf("Oni environment initialize success!\n");
  return true;
}  

void OniCamera::openCamera(){
  printf("going to open oni camera.\n");
  OpenOniCamera(depth_uri_str_.c_str());
}

void OniCamera::closeCamera(){
    printf("oni camera.\n");
}

// get camera params 
bool OniCamera::getCameraParams()
{
  int data_size = sizeof(cameraParams_);
  printf("data size : %d",data_size);
  memset(&cameraParams_, 0, sizeof(cameraParams_));
  Status rc = oni_device_.getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t*)&cameraParams_, &data_size);
  if(!rc)
  {
    std::cout << "successfully get camera params:" << "k[0]: " << cameraParams_.l_k[0] << "\n" 
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
  }
  else
  {
    std::cout<< "get camera params failed" << std::endl;
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
bool OniCamera::OpenOniCamera(const char* depth_uri)
{
  // 1. connect to openni camera
  std::cout << "going to open: " << depth_uri << std::endl;
  printf("going to open: %s \n", depth_uri);
  std::string uri = depth_uri;
  oni_rc_ = oni_device_.open(depth_uri);
  if (oni_rc_ != STATUS_OK)
  {
    printf("Couldn't open orbbec device\n");
    // printf(OpenNI::getExtendedError());
    return false;
  }
  // 2.create and start depth stream
  if (oni_device_.getSensorInfo(SENSOR_DEPTH) != NULL)
  {
    oni_rc_ = oni_depth_stream_.create(oni_device_, SENSOR_DEPTH);
    if (oni_rc_ != STATUS_OK)
    {
      printf("Couldn't create depth stream\n");
      return false;
    }

    // 重新设置帧率
    oni_depthVideoMode_ = oni_depth_stream_.getVideoMode();
    oni_depthVideoMode_.setResolution(ONI_WIDTH, ONI_HEIGHT);
    oni_depthVideoMode_.setFps(ONI_FPS);
    printf("Setting resolution width: %d, height: %d \n", ONI_WIDTH , ONI_HEIGHT);
    printf("Setting FPS : %d \n", ONI_FPS);
    oni_depth_stream_.setVideoMode(oni_depthVideoMode_);

    oni_rc_ = oni_depth_stream_.start();
    if(oni_rc_ != STATUS_OK)
    {
      printf("Couldn't start depth stream\n");
      // TEMI_LOG(error) << OpenNI::getExtendedError();
      return false;     
    }
  }

  // 2.create and start ir stream
  if (oni_device_.getSensorInfo(SENSOR_IR) != NULL)
  {
    oni_rc_ = oni_ir_stream_.create(oni_device_, SENSOR_IR);
    if (oni_rc_ != STATUS_OK)
    {
      printf("Couldn't create depth stream\n");
      // TEMI_LOG(error) << OpenNI::getExtendedError();
      return false;
    }

    // 重新设置帧率
    oni_irVideoMode_ = oni_ir_stream_.getVideoMode();
    oni_irVideoMode_.setResolution(ONI_WIDTH, ONI_HEIGHT);
    oni_irVideoMode_.setFps(ONI_FPS / 2);
    printf("Setting resolution width: %d, height: %d\n", ONI_WIDTH , ONI_HEIGHT);
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
  // head_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  return true;
}

std::string OniCamera::enumerateDevices()
{
  Array<DeviceInfo> deviceList;
  // TEMI_LOG(info) << "Going to enumerateDevices...";
  OpenNI::enumerateDevices(&deviceList);
  std::cout << deviceList.getSize() <<"  devices are found" << std::endl;
  // ROS_INFO("％d evices are found", deviceList.getSize());
  // TEMI_LOG(info)<< deviceList.getSize() << " devices are found.";
  std::string depth_uri_ ;
  for(int i = 0; i != deviceList.getSize(); i++)
  {
        const openni::DeviceInfo& info = deviceList[i];
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
        std::cout << "Device URI: " << uri << std::endl;//uri就是一个字符串 
        if(uri.at(10) == camera_loc_)
        {
          printf("Catch camera!\n");
          // ROS_INFO("Device URI: %s", uri);//uri就是一个字符串 
          std::cout << "Device URI:" << uri << std::endl;
          depth_uri_ = uri;//.c_str(); 
          // ROS_INFO("depth_uri_: %s", depth_uri_);
        }
  }
  if(depth_uri_.empty())
  {
    depth_uri_ = ANY_DEVICE;
  }
  return depth_uri_;
}

// set LDP on/off
void OniCamera::seOnitLDP(bool enable)
{
  int data_size = 4;
  int enable_ = 1;
  if (enable == false)
  {
    enable_ = 0;
  }

  oni_depth_stream_.stop();
  oni_device_.setProperty(XN_MODULE_PROPERTY_LDP_ENABLE, (uint8_t *)&enable_, 4);
  oni_depth_stream_.start();
  
}