#include <iostream>
#include <uvc_camera.h>
#include <oni_camera.h>
#include <opencv4/opencv2/opencv.hpp>
// main函数传入参数
// https://www.cnblogs.com/avril/archive/2010/03/22/1691477.html
int main(int argc, char **argv) 
{
    printf("hello!\n");
    if(*argv[1] == '1')
    {
        printf("Choosing HEAD camera.\n");
    }
    else if(*argv[1] == '5')
    {
        printf("Choosing GROUND camera.\n");
    }     
    UVCCamera uvc_camera;
    uvc_camera.setParams();
    uvc_camera.openCamera();
    OniCamera oni_camera;
    oni_camera.camera_loc_ = *argv[1];
    oni_camera.depth_uri_str_ = oni_camera.enumerateDevices();
    oni_camera.openCamera();
    oni_camera.seOnitLDP(false);

    while(true){
    }
    return 0;
}