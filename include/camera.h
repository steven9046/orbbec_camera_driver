// #ifndef _CMAERA_H_
// #define _CAMERA_H_
#pragma once
#include <iostream>

// namespace camera {
class Camera {
 public:
  Camera();
  // 虚析构函数 https://blog.csdn.net/sinat_27652257/article/details/79810567
  virtual ~Camera();
  // 纯虚函数定义接口 https://www.runoob.com/w3cnote/cpp-virtual-functions.html
  // 父类里可以不做实现
  virtual void openCamera() = 0;
  virtual void closeCamera() = 0;
  // 虚函数,父类里必须实现,子类里可以不实现,则调用父类的
  virtual void setParams();

 private:
  int camera_id;
};
// }  // namespace camera
// #endif