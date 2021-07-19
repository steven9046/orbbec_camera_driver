cmake_minimum_required(VERSION 2.8)
project(hello)

set (Tutorial_VERSION_MAJOR 1)
set (Tutorial_VERSION_MINOR 0)

configure_file (
"${PROJECT_SOURCE_DIR}/TutorialConfig.h.in"
"${PROJECT_BINARY_DIR}/TutorialConfig.h"
)

# 这里find的package之后的include和ibraries都是用这个宏扩展的
find_package(OpenCV REQUIRED)

# uvc libs
set(UVC_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/libuvc)
set(UVC_LIBS "${UVC_LIBS_DIR}/libuvc.so")

# openni2 libs
set(OPENNI2_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/openni2)
set (OPENNI2_LIBS "${OPENNI2_LIBS_DIR}/libOpenNI2.so;${OPENNI2_LIBS_DIR}/OpenNI2/Drivers/liborbbec.so")# 这个路径是固定的，不是这个目录结构OpenNI2/Drivers/lib*.so就找不到

include_directories (${OpenCV_INCLUDE_DIRS})
include_directories ("${PROJECT_SOURCE_DIR}/include")

# add_library(uvc_camera
#            )           

add_executable(hello
                src/camera.cpp
                src/uvc_camera.cpp
                src/oni_camera.cpp
                main.cpp
                )
                
target_link_libraries(hello 
                    ${OpenCV_LIBRARIES}
                    ${UVC_LIBS}
                    ${OPENNI2_LIBS})
