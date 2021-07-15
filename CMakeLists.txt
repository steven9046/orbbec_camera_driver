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
set(UVC_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/uvc_camera)
set(UVC_LIBS "${UVC_LIBS_DIR}/libuvc.so")

include_directories (${OpenCV_INCLUDE_DIRS})
include_directories ("${PROJECT_SOURCE_DIR}/include")

# add_library(uvc_camera
#            )           

add_executable(hello
                main.cpp
                src/uvc_camera.cpp
                src/camera.cpp)
                
target_link_libraries(hello 
                    ${OpenCV_LIBRARIES}
                    ${UVC_LIBS})
