#ifndef DRIVER_H
#define DRIVER_H
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <compressed_image_transport/compression_common.h>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <libuvc/libuvc.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "thetauvc/thetauvc.h"
#include "ffmpeg/H264Decoder.h"

#define MAX_PIPELINE_LEN 1000

namespace enc = sensor_msgs::image_encodings;
namespace cit = compressed_image_transport;

struct gst_src {
  GstElement *pipeline;
  GstElement *appsrc;

  GMainLoop *loop;
  GTimer *timer;
  guint framecount;
  guint id;
  guint bus_watch_id;
  uint32_t dwFrameInterval;
  uint32_t dwClockFrequency;
};

class thetaVDriver
{
private:

  ros::NodeHandle m_nh;
  char *pipe_proc;

  ros::Publisher m_pub_image;
  ros::Publisher m_pub_imageComp;

  float m_param_timeOffset;
  bool m_param_useNVdec;
  bool m_param_use4K;
  int m_param_thetaIndex;
  bool m_param_rawOn;
  bool m_param_compressOn;
  int m_param_pngLevel;

  bool m_turnoff = false;

  void getparam();
  bool turnOff(std_srvs::EmptyRequest  &req,std_srvs::EmptyResponse &res);

public:
  thetaVDriver();
  ~thetaVDriver();

  void publishImage(cv::Mat image);
  void setPipeProc();
  bool gst_src_init(gst_src& srcIn);
  int findDevList(uvc_context_t *ctx);
  bool set4k();
  bool isOff();
  void cv2sensorImg(cv::Mat mat, sensor_msgs::Image& sensorImg);
  void cv2sensorImgComp(cv::Mat mat, sensor_msgs::CompressedImage& sensorImg);
};


#endif

