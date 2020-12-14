#include <ros/ros.h>
#include "driver.h"

gst_src gsrc;
thetaVDriver * driverCLS;
H264Decoder HDecoder;
gboolean gst_bus_cb(GstBus *bus, GstMessage *message, gpointer data);
void cb(uvc_frame_t *frame, void *ptr);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thetaV_ros");

    driverCLS = new thetaVDriver();
    ros::spinOnce();

    if(!driverCLS->gst_src_init(gsrc)) return -1;

    uvc_error_t res;
    GstBus *bus;
    uvc_device_t *dev;
    uvc_device_handle_t *devh;
    uvc_stream_ctrl_t ctrl;
    uvc_context_t *ctx;

    bus = gst_pipeline_get_bus(GST_PIPELINE(gsrc.pipeline));
    gsrc.bus_watch_id = gst_bus_add_watch(bus, gst_bus_cb, NULL);
    gst_object_unref(bus);

    std::cout<<"[THETAV] DEVIEC CONNECTED"<<std::endl;
    res = uvc_init(&ctx, NULL);
    if (res != UVC_SUCCESS) {
      std::cout<<"[THETAV]UVC INIT FAIL"<<std::endl;
      return -1;
    }
    int selected = driverCLS->findDevList(ctx);
    if(selected<0)
    {
      std::cout<<"[THETAV] CANNOT SELECT DEVICE"<<std::endl;
      return -1;
    }
    std::cout<<"[THETAV] SELECT NO "<<selected<<std::endl;
    thetauvc_find_device(ctx, &dev, selected);
    uvc_open(dev, &devh);
    std::cout<<"[THETAV] DEVIEC CONNECTED"<<std::endl;

    gsrc.framecount=0;

    std::cout<<"[THETAV] SIZE SETTING NOW"<<std::endl;
    if(driverCLS->set4k()){
      thetauvc_get_stream_ctrl_format_size(devh,THETAUVC_MODE_UHD_2997, &ctrl);
      std::cout<<"[THETAV] SIZE SET TO 4K"<<std::endl;
    }
    else {
      thetauvc_get_stream_ctrl_format_size(devh,THETAUVC_MODE_FHD_2997, &ctrl);
      std::cout<<"[THETAV] SIZE SET TO 2K"<<std::endl;
    }

    std::cout<<"[THETAV] START STREAMING"<<std::endl;
    res = uvc_start_streaming(devh, &ctrl, cb, &gsrc, 0);
    if (res == UVC_SUCCESS) {
      g_main_loop_run(gsrc.loop);
      //IF STOP
      uvc_stop_streaming(devh);
      gst_element_set_state(gsrc.pipeline, GST_STATE_NULL);
      g_source_remove(gsrc.bus_watch_id);
      g_main_loop_unref(gsrc.loop);
    }
    return 0;
}

gboolean gst_bus_cb(GstBus *bus, GstMessage *message, gpointer data)
{
  GError *err;
  gchar *dbg;

  switch (GST_MESSAGE_TYPE(message)) {
  case GST_MESSAGE_ERROR:
    gst_message_parse_error(message, &err, &dbg);
    g_print("Error: %s\n", err->message);
    g_error_free(err);
    g_free(dbg);
    g_main_loop_quit(gsrc.loop);
    break;

  default:
    break;
  }

  return TRUE;
}

void cb(uvc_frame_t *frame, void *ptr)
{
  if(frame->data_bytes>5000)
  {
    cv::Mat cvimg;
    if(HDecoder.decode( (uchar*)frame->data,frame->data_bytes,cvimg))
    {
      driverCLS->publishImage(cvimg);
    }
  }
  ros::Rate rate(100);
  ros::spinOnce();
  rate.sleep();
  struct gst_src *s;
  s = (struct gst_src *)ptr;
  s->framecount++;
  usleep(1);
  //frame->data : H264 data

  if (driverCLS->isOff())
  {
    fprintf(stderr, "STOP PUB");
    exit(0);
  }

  return;
}

