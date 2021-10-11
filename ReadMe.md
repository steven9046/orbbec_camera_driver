What is orbbec camera driver
======================================

Orbbec camera is a light dirver using c++ for orbbec depth camera. The official driver is bind with ROS. If you don't want to install ROS and only want to do some simple test with orbbec camera you can use this light driver.

## Code ##

Find the latest version on [Github](https://github.com/sunforever1990/orbbec_camera_driver):

```
git clone https://github.com/sunforever1990/orbbec_camera_driver.git
```

## Dependencies ##

* libuvc
    * Orbbec cameras use libuvc to get RGB streams. You can get it from [libuvc](https://github.com/ktossell/libuvc)
    ```
    git clone https://github.com/ktossell/libuvc
    ```
    ```
    cd libuvc
    mkdir build
    cd build
    cmake ..
    make && sudo make install
    ```
    * You may use other drivers for RGB such as v4l.

* OpenNI2
    * This is for DEPTH stream and IR stream. The code is already include in libs folder. So you don't need to install it.

* OpenCV
    * This is used for image processing and visualization. You may visit [OpenCV](https://opencv.org/)

* PCL
    * This is used for point cloud processing. You may visit [PCL](https://pointclouds.org/)

* Pangolin
    * This is used for visualization. You can install it from [Pangoliln](https://github.com/stevenlovegrove/Pangolin/tree/v0.6)

## Building ##

Orbbec camera driver us CMake to build.
```
cd orbbec_camera_driver
mkdir build
cd build
cmake ..
make
```

