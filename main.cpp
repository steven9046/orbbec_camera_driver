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

  // //--------------------------------------------------
  // const char *filename = "/home/ss/pure_c_program/orbbec_camera_driver/ssd_lite_v1_meta.tflite";  // mssd_lite
  // // Load model
  // std::unique_ptr<tflite::FlatBufferModel> model = tflite::FlatBufferModel::BuildFromFile(filename);
  // // Build the interpreter with the InterpreterBuilder.
  // // Note: all Interpreters should be built with the InterpreterBuilder,
  // // which allocates memory for the Interpreter and does various set up
  // // tasks so that the Interpreter can read the provided model.
  // tflite::ops::builtin::BuiltinOpResolver resolver;
  // tflite::InterpreterBuilder builder(*model, resolver);
  // std::unique_ptr<tflite::Interpreter> interpreter;
  // builder(&interpreter);
  // TFLITE_MINIMAL_CHECK(interpreter != nullptr);

  // // Allocate tensor buffers.
  // TFLITE_MINIMAL_CHECK(interpreter->AllocateTensors() == kTfLiteOk);
  // printf("=== Pre-invoke Interpreter State ===\n");
  // tflite::PrintInterpreterState(interpreter.get());

  // // Fill input buffers
  // // TODO(user): Insert code to fill input tensors.
  // // Note: The buffer of the input tensor with index `i` of type T can
  // // be accessed with `T* input = interpreter->typed_input_tensor<T>(i);`
  // // kInputIndex 是输入张量索引，kNum 是输入图片张数，即 batch size。
  // int kInputIndex = 0;
  // int kNum = 1;
  // int kInputHeight = 300;
  // int kInputWidth = 300;
  // int kInputChannels = 3;
  // interpreter->ResizeInputTensor(kInputIndex, {kNum, kInputHeight, kInputWidth, kInputChannels});
  // // 按照新的输入张量的大小重新分配内存
  // interpreter->AllocateTensors();
  // // // 循环填充输入张量的内存，其中 kInputIndex 是输入张量索引。
  // uint8_t *input_buffer = interpreter->typed_tensor<uint8_t>(
  //     kInputIndex);  //这个就是一个指针，数组，存放输入的tensor,所以索引 0 就是第一个，我们这里只输入一张图片，所以只用第一个
  // const int kInputBytes = sizeof(uint8_t) * kInputWidth * kInputHeight * kInputChannels;
  // cv::Size input_buffer_size(kInputWidth, kInputHeight);
  // int buffer_index = 0;
  // cv::Mat origin_img = cv::imread("/home/ss/pure_c_program/orbbec_camera_driver/2.jpg");
  // // cv::Mat origin_img = uvc_camera.getImage();
  // cv::Mat input_image;
  // // 输入预处理操作。
  // cv::resize(origin_img, input_image, input_buffer_size);
  // cv::cvtColor(input_image,input_image,cv::COLOR_BGR2RGB);
  // // cv::cvtColor(input_image, input_image, cv::COLOR_BGR2GRAY);
  // // //   input_image.convertTo(input_image, CV_32F, 2.f / 255, -0.5);
  // //   // 填充输入张量的内存，batch size  > 1 时，注意
  // //   // input_buffer 的数据类型需要强制转换。因为 buffer_index 是按 byte 为单位进行地址偏移的。
  // memcpy(input_buffer, input_image.data, kInputBytes);
  // // }

  // // Run inference
  // TFLITE_MINIMAL_CHECK(interpreter->Invoke() == kTfLiteOk);
  // printf("\n\n=== Post-invoke Interpreter State ===\n");
  // tflite::PrintInterpreterState(interpreter.get());
  // int out_put_size = interpreter->outputs().size();
  // printf("outputs size: %d\n", out_put_size);
  // for (int i = 0; i < out_put_size; i++) {
  //   printf("%d th output: %d\n", i, interpreter->outputs()[i]);
  // }

  // int output = interpreter->outputs()[0];
  // int output_1 = interpreter->outputs()[1];
  // int output_2 = interpreter->outputs()[2];
  // int output_3 = interpreter->outputs()[3];

  // std::cout << "checking out size: " << interpreter->outputs().size() << std::endl;  // 4
  // std::cout << "checking out nums: " << output << endl;                              // 167
  // std::cout << "checking out nums_1: " << interpreter->outputs()[1] << endl;         // 168
  // std::cout << "checking out nums_2: " << interpreter->outputs()[2] << endl;         // 169
  // std::cout << "checking out nums_3: " << interpreter->outputs()[3] << endl;         // 170
  // // interpreter 的 graph是固定的，其中 167 -170 是 output
  // // Tensor 167 TFLite_Detection_PostP... kTfLiteFloat32  kTfLiteArenaRw     160      / 0.00 [1,10,4] [1182592, 1182752)
  // // Tensor 168 TFLite_Detection_PostP... kTfLiteFloat32  kTfLiteArenaRw     40       / 0.00 [1,10] [1182848, 1182888)
  // // Tensor 169 TFLite_Detection_PostP... kTfLiteFloat32  kTfLiteArenaRw     40       / 0.00 [1,10] [1182784, 1182824)
  // // Tensor 170 TFLite_Detection_PostP... kTfLiteFloat32  kTfLiteArenaRw     4        / 0.00 [1] [1182912, 1182916)

  // std::cout << "checking out nums_4: " << interpreter->outputs()[4] << endl;  // random
  // std::cout << "checking out nums_5: " << interpreter->outputs()[5] << endl;  // random

  // TfLiteIntArray *output_dims = interpreter->tensor(output)->dims;       // mutable tensor? [167]的 dims [1,10,4]
  // std::cout << "out put dims :  " << output_dims->size << std::endl;     // [1,10,4]
  // std::cout << "out put dim 0:  " << output_dims->data[0] << std::endl;  // 1
  // std::cout << "out put dim 1:  " << output_dims->data[1] << std::endl;  // 10
  // std::cout << "out put dim 2:  " << output_dims->data[2] << std::endl;  // 4
  // std::cout << "out put dim 3:  " << output_dims->data[3] << std::endl;  // null
  // std::cout << "out put dim 4:  " << output_dims->data[4] << std::endl;  // null

  // // assume output dims to be something like (1, 1, ... ,size)
  // auto output_size = output_dims->data[output_dims->size - 1];  // 最后一个维度: 4
  // std::cout << "output_size: " << output_size << std::endl;     // 4
  // // switch (interpreter->tensor(output)->type) {
  // //   case kTfLiteFloat32:
  // //     printf("kTfLiteFloat32");
  // //     break;
  // //   case kTfLiteInt8:
  // //     printf("kTfLiteInt8");
  // //     break;
  // //   case kTfLiteUInt8:
  // //     printf("kTfLiteUInt8");
  // //     break;
  // //   default:
  // //     std::cout << "cannot handle output type "
  // //                << interpreter->tensor(output)->type << " yet" << std::endl;
  // //     break;
  // // }
  // auto results = interpreter->typed_output_tensor<float>(0);  // 这个就应该是 167 ,返回的是指向该tensor的data的指针 [1,10,4]
  // // std::cout << "results: " << results[0] << std::endl;
  // // std::cout << "results: " << results[1] << std::endl;
  // // std::cout << "results: " << results[2] << std::endl;
  // // std::cout << "results: " << results[3] << std::endl;
  // // std::cout << "results: " << results[4] << std::endl;
  // // std::cout << "results: " << results[5] << std::endl;
  // for (int i = 0; i < 40; i++) {
  //   printf("results: %f  ", results[i]);
  //   if ((i + 1) % 4 == 0) {
  //     printf("\n");
  //   }
  // }
  // printf("----------------\n");
  // // 直接读取interpreter的tensor也可以
  // std::cout << " tensor info: " << interpreter->tensor(168)->bytes << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(168)->data.f[0] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(168)->data.f[1] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(168)->data.f[2] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(168)->data.f[3] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(168)->data.f[4] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(168)->data.f[5] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(168)->data.f[6] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(168)->data.f[7] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(168)->data.f[8] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(168)->data.f[9] << std::endl;
  // printf("----------------\n");
  // std::cout << " tensor data: " << interpreter->tensor(169)->data.f[0] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(169)->data.f[1] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(169)->data.f[2] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(169)->data.f[3] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(169)->data.f[4] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(169)->data.f[5] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(169)->data.f[6] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(169)->data.f[7] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(169)->data.f[8] << std::endl;
  // std::cout << " tensor data: " << interpreter->tensor(169)->data.f[9] << std::endl;
  // printf("----------------\n");
  // std::cout << " tensor data: " << interpreter->tensor(170)->data.f[0] << std::endl;
  // const char *out_name = interpreter->GetOutputName(0);
  // std::cout << "Out name: " << out_name << std::endl;
  // // ymin, xmin, ymax, xmax = obj['bounding_box']
  // // xmin = int(xmin * CAMERA_WIDTH)
  // // xmax = int(xmax * CAMERA_WIDTH)
  // // ymin = int(ymin * CAMERA_HEIGHT)
  // // ymax = int(ymax * CAMERA_HEIGHT)
  // // [	top,	left,	bottom,	right	]
  // for (int i = 0; i < 40; i++) {
  //   if (i % 4 == 0) {
  //     int top = 300 * results[i];
  //     int left = 300 * results[i + 1];
  //     int bottom = 300 * results[i + 2];
  //     int right = 300 * results[i + 3];
  //     int index = i / 4;
  //     printf("index: %d \n", index);
  //     printf("score: %f \n", interpreter->tensor(169)->data.f[index]);
  //     printf("top: %d, left: %d,  bottom: %d, right: %d \n", top, left, bottom, right);
  //     cv::rectangle(input_image, cv::Point(300 * left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 255), 3, 4, 0);
  //     // cv::putText(input_image, (string)index, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0, 0, 255), 2);
  //     printf("\n");
  //   }
  // }

  // cv::namedWindow("rgb_img", 0);
  // cv::imshow("rgb_img", input_image);
  // cv::waitKey(0);
  // //--------------------------------------------------

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
        // cv::namedWindow("rgb_img", 0);
        // cv::imshow("rgb_img", input_image);
        // cv::waitKey(1);
    usleep(100);
  }
  return 0;
}