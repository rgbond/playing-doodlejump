cmake_minimum_required(VERSION 2.8.3)
project(score)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11 -Wall -D_REENTRANT)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs image_transport sensor_msgs)
find_package(catkin REQUIRED COMPONENTS roscpp message_generation)

add_message_files (
    FILES
    DoodleScore.msg
)

generate_messages (
    DEPENDENCIES
    std_msgs
)

catkin_package(
   LIBRARIES score
   DEPENDS std_msgs image_transport sensor_msgs
   CATKIN_DEPENDS message_runtime
)

include_directories( 
    ${catkin_INCLUDE_DIRS}
    /usr/local/include
    /usr/local/cuda-9.0/targets/aarch64-linux/include
    include
)
link_directories(
    /usr/local/lib
    /usr/local/cuda-9.0/targets/aarch64-linux/lib
)
add_executable(
    ${PROJECT_NAME}
    src/score.cpp
    src/scoreNet.cpp
    src/tensorNet.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    nvinfer
    nvcaffe_parser
    opencv_videostab 
    opencv_cudastereo 
    opencv_dnn 
    opencv_shape 
    opencv_ml 
    opencv_superres 
    opencv_cudaoptflow 
    opencv_viz 
    opencv_photo 
    opencv_cudabgsegm 
    opencv_cudacodec 
    opencv_stitching 
    opencv_cudafeatures2d 
    opencv_cudaobjdetect 
    opencv_cudalegacy 
    opencv_calib3d 
    opencv_features2d 
    opencv_highgui 
    opencv_videoio 
    opencv_imgcodecs 
    opencv_cudawarping 
    opencv_cudaimgproc 
    opencv_cudafilters 
    opencv_video 
    opencv_objdetect 
    opencv_imgproc 
    opencv_flann 
    opencv_cudaarithm 
    opencv_core 
    opencv_cudev
    cudart
)
# cudnn cublas cudart_static nvToolsExt cudart nvinfer nvcaffe_parser

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
