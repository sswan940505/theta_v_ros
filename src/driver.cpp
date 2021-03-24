#include "driver.h"


thetaVDriver::thetaVDriver()
{
  getparam();
  setPipeProc();
  static ros::ServiceServer service = m_nh.advertiseService("/turnoff", &thetaVDriver::turnOff,this);
  if(m_param_rawOn)
    m_pub_image = m_nh.advertise<sensor_msgs::Image>("/image/raw",100);
  if(m_param_compressOn)
    m_pub_imageComp = m_nh.advertise<sensor_msgs::CompressedImage>("/image/compressed",100);
}

thetaVDriver::~thetaVDriver()
{

}

void thetaVDriver::getparam()
{
  m_param_useNVdec    = false;
  m_param_use4K       = false;
  m_param_rawOn       = false;
  m_param_compressOn  = false;
  m_param_pngLevel    = 3;
  // get param
  ros::param::get("~nvdec",                m_param_useNVdec   );
  ros::param::get("~use4k",                m_param_use4K      );
  ros::param::get("~image/raw/on",         m_param_rawOn      );
  ros::param::get("~image/compress/on",    m_param_compressOn );
  ros::param::get("~image/compress/level", m_param_pngLevel   );
}

bool thetaVDriver::turnOff(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  m_turnoff = true;
  return true;
}

void thetaVDriver::setPipeProc()
{
  if(m_param_useNVdec)
  {
    pipe_proc = "nvdec ! glimagesink qos=false sync=false";
  }
  else {
    pipe_proc = " decodebin ! autovideosink qos=false sync=false";
  }
}


bool thetaVDriver::gst_src_init(gst_src& srcIn)
{
  GstCaps *caps;
  char pipeline_str[MAX_PIPELINE_LEN];

  snprintf(pipeline_str, MAX_PIPELINE_LEN, "appsrc name=ap ! queue ! h264parse ! queue ! %s ", pipe_proc);

  int *argc1;
  char ***argv1;
  gst_init(argc1, argv1);
  srcIn.timer = g_timer_new();
  srcIn.loop = g_main_loop_new(NULL, TRUE);
  srcIn.pipeline = gst_parse_launch(pipeline_str, NULL);


  g_assert(srcIn.pipeline);
  if (srcIn.pipeline == NULL)
    return FALSE;
  gst_pipeline_set_clock(GST_PIPELINE(srcIn.pipeline), gst_system_clock_obtain());

  srcIn.appsrc = gst_bin_get_by_name(GST_BIN(srcIn.pipeline), "ap");

  caps = gst_caps_new_simple("video/x-h264",
                             "framerate", GST_TYPE_FRACTION, 30000, 1001,
                             "stream-format", G_TYPE_STRING, "byte-stream",
                             "profile", G_TYPE_STRING, "constrained-baseline", NULL);
  gst_app_src_set_caps(GST_APP_SRC(srcIn.appsrc), caps);
  return TRUE;
}

int thetaVDriver::findDevList(uvc_context_t *ctx)
{
  uvc_error_t res;
  uvc_device_t **devlist;

  std::cout<<"[THETAV]FOUND THETA V NOW"<<std::endl;
  res = thetauvc_find_devices(ctx, &devlist);
  if (res != UVC_SUCCESS) {
    uvc_perror(res,"");
    uvc_exit(ctx);
    return -1;
  }

  int idx = 0;
  printf("No : %-18s : %-10s\n", "Product", "Serial");
  while (devlist[idx] != NULL) {
    uvc_device_descriptor_t *desc;

    if (uvc_get_device_descriptor(devlist[idx], &desc) != UVC_SUCCESS)
      continue;

    printf("%2d : %-18s : %-10s\n", idx, desc->product,
           desc->serialNumber);

    uvc_free_device_descriptor(desc);
    idx++;
  }

  uvc_free_device_list(devlist, 1);
  std::cout<<idx-1<<std::endl;
  if(idx-1<m_param_thetaIndex) return -1;
  return m_param_thetaIndex;
}

bool thetaVDriver::set4k()
{
  return m_param_use4K;
}

bool thetaVDriver::isOff()
{
  return m_turnoff;
}


void thetaVDriver::cv2sensorImg(cv::Mat mat, sensor_msgs::Image& sensorImg)
{
  cv_bridge::CvImage bridge;
  mat.copyTo(bridge.image);
  bridge.header.frame_id = "theta";
  bridge.header.stamp = ros::Time::now();
  if(mat.type() == CV_8UC1) bridge.encoding = sensor_msgs::image_encodings::MONO8;
  else if(mat.type() == CV_8UC3) bridge.encoding = sensor_msgs::image_encodings::BGR8;
  else if(mat.type() == CV_32FC1) bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  else if(mat.type() == CV_16UC1) bridge.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  else return;


  bridge.toImageMsg(sensorImg);
}



void thetaVDriver::cv2sensorImgComp(cv::Mat mat, sensor_msgs::CompressedImage& sensorImg)
{
  sensorImg.header.frame_id = "theta";
  sensorImg.header.stamp = ros::Time::now();
  std::vector<int> params;
  params.resize(3, 0);
  sensor_msgs::Image dummy;
  if(mat.type() == CV_8UC1) dummy.encoding = sensor_msgs::image_encodings::MONO8;
  else if(mat.type() == CV_8UC3) dummy.encoding = sensor_msgs::image_encodings::BGR8;
  else if(mat.type() == CV_32FC1) dummy.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  else if(mat.type() == CV_16UC1) dummy.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  else return;

  sensorImg.format = dummy.encoding;
  int bitDepth = enc::bitDepth(dummy.encoding);
  int numChannels = enc::numChannels(dummy.encoding);
  params[0] = cv::IMWRITE_PNG_COMPRESSION;
  params[1] = m_param_pngLevel;
  sensorImg.format += "; png compressed";
  if (((bitDepth == 16) || (bitDepth == 8)) && ((numChannels == 1) || (numChannels == 3)))
  {
    // Target image format
    std::stringstream targetFormat;
    if (enc::isColor(dummy.encoding))
    {
      cv::cvtColor(mat, mat, CV_BGR2RGB);
      targetFormat << "bgr" << bitDepth;
    }
    try
    {
      // Compress image
      if (cv::imencode(".png", mat, sensorImg.data, params))
      {

        float cRatio = (float)(mat.rows * mat.cols * mat.elemSize())
            / (float)sensorImg.data.size();
        ROS_DEBUG("Compressed Image Transport - Codec: png, Compression Ratio: 1:%.2f (%lu bytes)", cRatio, (long unsigned int)sensorImg.data.size());
      }
      else
      {
        ROS_ERROR("cv::imencode (png) failed on input image");
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("%s", e.what());
    }
    catch (cv::Exception& e)
    {
      ROS_ERROR("%s", e.what());
    }
  }
  else
    ROS_ERROR("Compressed Image Transport - PNG compression requires 8/16-bit, 1/3-channel images (input format is: %s)", dummy.encoding.c_str());
  return;
}

void thetaVDriver::publishImage(cv::Mat image)
{
  if(m_param_rawOn)
  {
    sensor_msgs::Image imgOut;
    cv2sensorImg(image,imgOut);
    m_pub_image.publish(imgOut);
  }
  if(m_param_compressOn)
  {
    sensor_msgs::CompressedImage imgOut;
    cv2sensorImgComp(image,imgOut);
    m_pub_imageComp.publish(imgOut);

  }
}

