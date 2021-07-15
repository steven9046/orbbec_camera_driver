#ifndef UVC_CAMERA_CONFIG_H
#define UVC_CAMERA_CONFIG_H
#include <string>

using namespace std;

// namespace orbbec_test
// {

enum video_modes {
    uncompressed = 0,
    compressed = 1,
    yuyv = 2,
    uyvy = 3,
    rgb = 4,
    bgr = 5,
    mjpeg = 6,
    gray8 = 7
};

class UVCCameraConfig
{

public:

    string vendor;
    string product;
    string serial;
    int index;
    int width;
    int height;

    // video_modes video_mode;
    string video_mode;

    int frame_rate;
    string timestamp_method;
    string camera_info_url;
    int auto_exposure;
    bool auto_white_balance;
};
// }// namespace

#endif

