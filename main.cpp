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

#include <oni_camera.h>
#include <uvc_camera.h>
#include <viewer.h>

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <thread>
// // tflite
#include <cstdio>

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/optional_debug_tools.h"

// #define LOG(x) std::cerr

#define TFLITE_MINIMAL_CHECK(x)                              \
  if (!(x)) {                                                \
    fprintf(stderr, "Error at %s:%d\n", __FILE__, __LINE__); \
    exit(1);                                                 \
  }

void test() {
  while (1) {
    printf("testing thread...\n");
    usleep(200);
  }
}
// Input camera bus number to choose a specific camera. You can use "lsusb" to check which bus your camera is on.
// https://www.cnblogs.com/avril/archive/2010/03/22/1691477.html
int main(int argc, char **argv) {
  printf("Orbbec camera driver!\n");
  printf("Chooseing camera on bus:%s\n", argv[1]);
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
  // Viewer for point cloud
  viewer::Viewer main_viewer(480, 640, "pcl");
  main_viewer.setInputCloud(oni_camera.point_cloud_);
  std::thread display_loop;
  display_loop = std::thread(&viewer::Viewer::run, &main_viewer);

  //--------------------------------------------------
  const char *filename = "/home/ss/pure_c_program/orbbec_camera_driver/mssd_lite.tflite";
  // Load model
  std::unique_ptr<tflite::FlatBufferModel> model = tflite::FlatBufferModel::BuildFromFile(filename);
  // Build the interpreter with the InterpreterBuilder.
  // Note: all Interpreters should be built with the InterpreterBuilder,
  // which allocates memory for the Interpreter and does various set up
  // tasks so that the Interpreter can read the provided model.
  tflite::ops::builtin::BuiltinOpResolver resolver;
  tflite::InterpreterBuilder builder(*model, resolver);
  std::unique_ptr<tflite::Interpreter> interpreter;
  builder(&interpreter);
  TFLITE_MINIMAL_CHECK(interpreter != nullptr);

  // Allocate tensor buffers.
  TFLITE_MINIMAL_CHECK(interpreter->AllocateTensors() == kTfLiteOk);
  printf("=== Pre-invoke Interpreter State ===\n");
  tflite::PrintInterpreterState(interpreter.get());

  // Fill input buffers
  // TODO(user): Insert code to fill input tensors.
  // Note: The buffer of the input tensor with index `i` of type T can
  // be accessed with `T* input = interpreter->typed_input_tensor<T>(i);`

  int t_size = interpreter->tensors_size();
  for (int i = 0; i < t_size; i++) {
    if (interpreter->tensor(i)->name)
      LOG(INFO) << i << ": " << interpreter->tensor(i)->name << ", " << interpreter->tensor(i)->bytes << ", " << interpreter->tensor(i)->type << ", "
                << interpreter->tensor(i)->params.scale << ", " << interpreter->tensor(i)->params.zero_point;
  }
}
// Run inference
TFLITE_MINIMAL_CHECK(interpreter->Invoke() == kTfLiteOk);
printf("\n\n=== Post-invoke Interpreter State ===\n");
tflite::PrintInterpreterState(interpreter.get());
//--------------------------------------------------

// Get depth data and show depth image.
while (1) {
  oni_camera.GetOniStreamData();
  if (oni_camera.oni_depth_frame_.isValid()) {
    openni::DepthPixel *pDepth = (openni::DepthPixel *)oni_camera.oni_depth_frame_.getData();
    cv::Mat raw_depth(ONI_HEIGHT, ONI_WIDTH, CV_16UC1, pDepth);
    cv::flip(raw_depth, raw_depth, 1);
    cv::Mat tmp_depth;
    // Show raw_depth
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