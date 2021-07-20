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