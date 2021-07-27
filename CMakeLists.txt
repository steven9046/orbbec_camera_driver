cmake_minimum_required(VERSION 2.8)
project(orbbec_camera_driver)

set (Tutorial_VERSION_MAJOR 1)
set (Tutorial_VERSION_MINOR 0)

configure_file (
"${PROJECT_SOURCE_DIR}/TutorialConfig.h.in"
"${PROJECT_BINARY_DIR}/TutorialConfig.h"
)

# 这里find的package之后的include和ibraries都是用这个宏扩展的
# 这里没有版本要求，用的都是最基本的功能
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    find_package(Pangolin 0.4 REQUIRED)
endif()

# uvc libs
set(UVC_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/${CMAKE_SYSTEM_PROCESSOR}/libuvc)
set(UVC_LIBS "${UVC_LIBS_DIR}/libuvc.so")

# openni2 libs
set(OPENNI2_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/${CMAKE_SYSTEM_PROCESSOR}/openni2)
set (OPENNI2_LIBS "${OPENNI2_LIBS_DIR}/libOpenNI2.so;${OPENNI2_LIBS_DIR}/OpenNI2/Drivers/liborbbec.so")# 这个路径是固定的，不是这个目录结构OpenNI2/Drivers/lib*.so就找不到

include_directories (${OpenCV_INCLUDE_DIRS})
include_directories (${PCL_INCLUDE_DIRS})
include_directories ("${PROJECT_SOURCE_DIR}/include")
if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    include_directories (${Pangolin_INCLUDE_DIRS})
endif()

# tensorflow lite
# set(TENSORFLOW_SOURCE_DIR "/home/ss/tensorflow_src")
# if(NOT TENSORFLOW_SOURCE_DIR)
#   get_filename_component(TENSORFLOW_SOURCE_DIR
#     "${CMAKE_CURRENT_LIST_DIR}/../../../../"
#     ABSOLUTE
#   )
# endif()

# add_subdirectory(
#   "${TENSORFLOW_SOURCE_DIR}/tensorflow/lite"
#   "${CMAKE_CURRENT_BINARY_DIR}/tensorflow-lite"
#   EXCLUDE_FROM_ALL
# )      

# tflite library
set(TFLITE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/${CMAKE_SYSTEM_PROCESSOR}/tflite)
set(TFLITE_LIBS "${TFLITE_SOURCE_DIR}/libtensorflowlite.so")
include_directories ("${PROJECT_SOURCE_DIR}/include")

message(STATUS "TENSORFLOW_SOURCE_DIR: " ${TENSORFLOW_SOURCE_DIR})

add_executable(orbbec_camera_driver
                src/camera.cpp
                src/uvc_camera.cpp
                src/oni_camera.cpp
                src/viewer.cpp
                main.cpp
                )
                
target_link_libraries(orbbec_camera_driver 
                    ${OpenCV_LIBRARIES}
                    ${PCL_LIBRARIES}
                    ${UVC_LIBS}
                    ${OPENNI2_LIBS}
                    ${TFLITE_LIBS})

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    target_link_libraries(orbbec_camera_driver
                            ${Pangolin_LIBRARIES})
endif()
                    
