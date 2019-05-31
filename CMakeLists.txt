cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)

project( openpose_ros )
find_package( OpenCV REQUIRED )
find_package(Boost REQUIRED)

set(CMAKE_MODULE_PATH
/openpose-1.2.1/cmake/Modules ${CMAKE_BINARY_DIR}
${CMAKE_MODULE_PATH})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")


catkin_package()
include(
#/openpose-1.2.1/cmake/Modules/FindGlog.cmake
/openpose-1.2.1/cmake/Modules/FindGFlags.cmake
#/openpose-1.2.1/3rdparty/caffe/cmake/External/gflags.cmake
#/openpose-1.2.1/3rdparty/caffe/cmake/External/glog.cmake
)

include(
/openpose-1.2.1/cmake/Modules/FindGlog.cmake
#/openpose-1.2.1/cmake/Modules/FindGFlags.cmake
#/openpose-1.2.1/3rdparty/caffe/cmake/External/gflags.cmake
#/openpose-1.2.1/3rdparty/caffe/cmake/External/glog.cmake
)

include(/openpose-1.2.1/cmake/Modules/FindOpenPose.cmake)
find_package(GFlags)
find_package(Glog)
find_package(OpenCV)
find_package(OpenPose)

find_package(catkin REQUIRED COMPONENTS
	roscpp
cv_bridge
image_transport
)

include_directories(
/openpose-1.2.1/include/
/openpose-1.2.1/include/openpose
/openpose-1.2.1/examples_beta/openpose3d/include/openpose3d
#${OpenPose_INCLUDE_DIRS}
${GFLAGS_INCLUDE_DIR} ${GLOG_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

link_directories( /openpose-1.2.1/.build_release/lib/
/openpose-1.2.1/.build_release/src
/openpose-1.2.1/build/src/openpose/wrapper
/openpose-1.2.1/3rdparty/caffe/distribute/lib/
/usr/lib/x86_64-linux-gnu/libglog.so.0
/lib/x86_64-linux-gnu/
/openpose-1.2.1/include/)

add_executable( uu openpose.cpp )
target_link_libraries( uu ${OpenCV_LIBS} ${OpenPose_LIBS} ${GLOG_LIBRARY} ${GFLAGS_LIBRARY}
#${Caffe_LIBS}
)

add_executable( mynode ros_publisher.cpp )
add_dependencies(mynode ${catkin_EXPORTED_TARGETS})
target_link_libraries( mynode ${OpenCV_LIBS} ${OpenPose_LIBS} ${GLOG_LIBRARY} ${GFLAGS_LIBRARY} ${catkin_LIBRARIES}
#${Caffe_LIBS}
${Boost_LIBS}
)
