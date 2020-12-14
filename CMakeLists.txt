cmake_minimum_required(VERSION 2.8.3)
project(theta_v_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_srvs
    sensor_msgs
    cv_bridge
)

find_package(OpenCV REQUIRED)
set( Boost_USE_MULTITHREADED 1 )
find_package( Boost 1.53 REQUIRED system thread timer )
find_package( PkgConfig )
pkg_check_modules( GSTREAMER_1_0 gstreamer-1.0 )
pkg_check_modules( GSTREAMER_1_0_APP gstreamer-app-1.0 )
pkg_check_modules( LIBUVC libuvc )
include_directories( ${GSTREAMER_1_0_INCLUDE_DIRS} )
include_directories( ${GSTREAMER_1_0_APP_INCLUDE_DIRS} )
include_directories( ${LIBUVC_INCLUDE_DIRS} )
include_directories(
 # include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)
include_directories(
    /usr/include/libavcodec
    /usr/include/libavformat
    /usr/include/libswscale
    )

catkin_package()

####################################################################################################
# Project
####################################################################################################

FILE(GLOB theta_source
    "src/thetauvc/thetauvc.h"
    "src/thetauvc/thetauvc.c"
    "src/ffmpeg/H264Decoder.cpp"
    "src/ffmpeg/H264Decoder.h"
    "src/driver.cpp"
    "src/driver.h"
)


add_executable( driver src/node.cpp  ${theta_source} )
add_dependencies(driver theta_v_ros_generate_messages_cpp)
target_link_libraries( driver
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${GSTREAMER_1_0_LIBRARIES}
    ${LIBUVC_LIBRARIES}
    ${GSTREAMER_1_0_APP_LIBRARIES}
    ${OpenCV_LIBS}
    /usr/lib/x86_64-linux-gnu/libavcodec.so
    /usr/lib/x86_64-linux-gnu/libavformat.so
    /usr/lib/x86_64-linux-gnu/libswscale.so
    /usr/lib/x86_64-linux-gnu/libavutil.so
    )
