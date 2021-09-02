#ifndef THETA_DERIVER_HPP
#define THETA_DERIVER_HPP

#include "theta_driver.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <libuvc/libuvc.h>
#include "thetauvc.h"
#include <opencv2/core.hpp>

namespace theta_driver {

class ThetaDriver {
public:
    ThetaDriver();
    bool init();
    bool open();
    void run();
    void publishImage(GstMapInfo map);

    uvc_device_handle_t* devh_;
    uvc_stream_ctrl_t ctrl_;
    uvc_context_t* ctx_;
    bool use4k_ = false;
    std::string serial_ = "";
    std::string camera_frame_ = "camera_link";
    std::string pipeline_;
    ros::Publisher image_pub_;
};

struct gst_src {
    GstElement* pipeline;
    GstElement* appsrc;

    GMainLoop* loop;
    GTimer* timer;
    guint framecount;
    guint id;
    guint bus_watch_id;
    uint32_t dwFrameInterval;
    uint32_t dwClockFrequency;
};

} // namespace theta_driver

#endif // THETA_DERIVER_HPP
