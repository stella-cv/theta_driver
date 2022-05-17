#ifndef THETA_DERIVER_LIB_HPP
#define THETA_DERIVER_LIB_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <libuvc/libuvc.h>
#include <opencv2/core.hpp>
#include "thetauvc.h"

namespace theta_driver {

class ThetaDriver : public rclcpp::Node {
public:
    ThetaDriver(const rclcpp::NodeOptions & options);
    virtual ~ThetaDriver();
    void onInit();
    bool init();
    bool open();
    void publishImage(GstMapInfo map);

    bool streaming_ = false;
    uvc_device_handle_t* devh_;
    uvc_stream_ctrl_t ctrl_;
    uvc_context_t* ctx_;
    bool use4k_ = false;
    std::string serial_ = "";
    std::string camera_frame_ = "camera_link";
    std::string pipeline_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
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

#endif // THETA_DERIVER_LIB_HPP
