#include <iostream>
#include <uvc_camera.h>
#include <opencv4/opencv2/opencv.hpp>
// main函数传入参数
// https://www.cnblogs.com/avril/archive/2010/03/22/1691477.html
int main(int argc, char **argv) 
{
    printf("hello!\n");
    UVCCamera uvc_camera;
    uvc_camera.setParams();
    uvc_camera.openCamera();
    while(true){
    }
    return 0;
}