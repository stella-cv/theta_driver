#include "theta_driver/theta_driver_lib.hpp"

namespace {
theta_driver::gst_src gsrc;
}

namespace theta_driver {

gboolean gst_bus_callback(GstBus* bus, GstMessage* message, gpointer data) {
    GError* err;
    gchar* dbg;
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

void uvc_streaming_callback(uvc_frame_t* frame, void* ptr) {
    struct gst_src* src = (struct gst_src*)ptr;
    GstBuffer* buffer;
    GstFlowReturn ret;
    GstMapInfo map;

    buffer = gst_buffer_new_allocate(NULL, frame->data_bytes, NULL);
    GST_BUFFER_PTS(buffer) = frame->sequence * src->dwFrameInterval * 100;
    GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
    GST_BUFFER_DURATION(buffer) = src->dwFrameInterval * 100;
    GST_BUFFER_OFFSET(buffer) = frame->sequence;
    src->framecount++;

    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, frame->data, frame->data_bytes);
    gst_buffer_unmap(buffer, &map);

    g_signal_emit_by_name(src->appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);
    if (ret != GST_FLOW_OK) {
        fprintf(stderr, "g_signal_emit_by_name push-buffer error");
    }
    return;
}

GstFlowReturn new_sample_callback(GstAppSink* sink, gpointer data) {
    GstSample* sample = gst_app_sink_pull_sample(sink);
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstBuffer* app_buffer = gst_buffer_copy_deep(buffer);
    GstMapInfo map;
    gst_buffer_map(app_buffer, &map, GST_MAP_WRITE);

    ThetaDriver* driver = static_cast<ThetaDriver*>(data);
    driver->publishImage(map);

    gst_sample_unref(sample);
    gst_buffer_unmap(app_buffer, &map);
    gst_buffer_unref(app_buffer);
    if (sample == NULL) {
        return GST_FLOW_EOS;
    }
    else {
        return GST_FLOW_OK;
    }
}

void ThetaDriver::publishImage(GstMapInfo map) {
    int dataLength;
    guint8* rdata;

    dataLength = map.size;
    rdata = map.data;

    sensor_msgs::msg::Image image;
    image.header.stamp = this->get_clock()->now();
    image.header.frame_id = camera_frame_;
    if (use4k_) {
        image.width = 3840;
        image.height = 1920;
    }
    else {
        image.width = 1920;
        image.height = 960;
    }
    image.encoding = "rgb8";
    image.is_bigendian = false;
    image.step = image.width * 3;

    std::vector<unsigned char> values(rdata, (unsigned char*)rdata + dataLength);
    image.data = values;
    image_pub_->publish(image);
}

ThetaDriver::ThetaDriver(const rclcpp::NodeOptions & options): Node("theta_driver", options){}

ThetaDriver::~ThetaDriver() {
    if (streaming_) {
        gst_element_set_state(gsrc.pipeline, GST_STATE_NULL);
        g_source_remove(gsrc.bus_watch_id);
        uvc_stop_streaming(devh_);
        uvc_close(devh_);
        uvc_exit(ctx_);
    }
}

void ThetaDriver::onInit() {
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

    declare_parameter<bool>("use4k",false);
    get_parameter("use4k",use4k_);
    declare_parameter<std::string>("serial","");
    get_parameter("serial",serial_);
    declare_parameter<std::string>("camera_frame", camera_frame_);
    get_parameter("camera_frame",camera_frame_);
    pipeline_ =
        "appsrc name=ap ! queue ! h264parse ! queue ! "
        "decodebin ! queue ! videoconvert n_threads=8 ! queue ! video/x-raw,format=RGB ! appsink name=appsink emit-signals=true";
    declare_parameter<std::string>("pipeline", pipeline_);
    get_parameter("pipeline",pipeline_);

    rclcpp::Rate rate(1);
    while (rclcpp::ok()) {
        bool ok = init();
        if (ok) {
            break;
        }
        else {
            RCLCPP_ERROR(get_logger(),"Initialization failed");
        }
        rate.sleep();
        RCLCPP_WARN(get_logger(),"retry");
    }
}

bool ThetaDriver::open() {
    uvc_error_t res;
    uvc_device_t** devlist;

    res = uvc_init(&ctx_, NULL);
    if (res != UVC_SUCCESS) {
        RCLCPP_ERROR_STREAM(get_logger(),"uvc_init failed");
        return false;
    }

    res = thetauvc_find_devices(ctx_, &devlist);
    if (res != UVC_SUCCESS) {
        uvc_perror(res, "find_thetauvc_device error");
        uvc_exit(ctx_);
        return false;
    }

    unsigned int idx = 0;
    while (devlist[idx] != NULL) {
        uvc_device_descriptor_t* desc;

        if (uvc_get_device_descriptor(devlist[idx], &desc) == UVC_SUCCESS) {
            RCLCPP_INFO_STREAM(get_logger(),"index: " << idx);
            if (desc->product) {
                RCLCPP_INFO_STREAM(get_logger(),"product: " << desc->product);
            }
            if (desc->serialNumber) {
                RCLCPP_INFO_STREAM(get_logger(),"serial: " << desc->serialNumber);
            }
            if (serial_.empty() || serial_ == std::string(desc->serialNumber)) {
                uvc_device_t* dev;
                thetauvc_find_device(ctx_, &dev, idx);
                uvc_open(dev, &devh_);
                uvc_free_device_descriptor(desc);
                uvc_free_device_list(devlist, 1);

                if (use4k_) {
                    thetauvc_get_stream_ctrl_format_size(devh_, THETAUVC_MODE_UHD_2997, &ctrl_);
                }
                else {
                    thetauvc_get_stream_ctrl_format_size(devh_, THETAUVC_MODE_FHD_2997, &ctrl_);
                }
                return true;
            }
            else {
                uvc_free_device_descriptor(desc);
            }
        }
        idx++;
    }
    return false;
}

bool ThetaDriver::init() {
    streaming_ = false;
    if (!gst_is_initialized()) {
        gst_init(0, 0);
    }

    GError* error = NULL;
    gsrc.framecount = 0;
    gsrc.loop = g_main_loop_new(NULL, TRUE);
    gsrc.timer = g_timer_new();
    gsrc.pipeline = gst_parse_launch(pipeline_.c_str(), &error);
    if (gsrc.pipeline == NULL) {
        RCLCPP_FATAL_STREAM(get_logger(),error->message);
        g_error_free(error);
        return false;
    }
    gst_pipeline_set_clock(GST_PIPELINE(gsrc.pipeline), gst_system_clock_obtain());
    gsrc.appsrc = gst_bin_get_by_name(GST_BIN(gsrc.pipeline), "ap");

    GstCaps* caps = gst_caps_new_simple("video/x-h264",
                                        "framerate", GST_TYPE_FRACTION, 30000, 1001,
                                        "stream-format", G_TYPE_STRING, "byte-stream",
                                        "profile", G_TYPE_STRING, "constrained-baseline", NULL);
    gst_app_src_set_caps(GST_APP_SRC(gsrc.appsrc), caps);

    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(gsrc.pipeline));
    gsrc.bus_watch_id = gst_bus_add_watch(bus, gst_bus_callback, NULL);
    gst_object_unref(bus);

    GstElement* appsink = gst_bin_get_by_name(GST_BIN(gsrc.pipeline), "appsink");
    if (appsink == NULL) {
        g_print("appsink is NULL\n");
    }
    g_signal_connect(appsink, "new-sample", G_CALLBACK(new_sample_callback), this);
    gst_object_unref(appsink);

    bool ok = open();
    if (!ok) {
        RCLCPP_FATAL(get_logger(),"Device open error");
        return false;
    }

    gsrc.dwFrameInterval = ctrl_.dwFrameInterval;
    gsrc.dwClockFrequency = ctrl_.dwClockFrequency;
    if (gst_element_set_state(gsrc.pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_FATAL(get_logger(),"Could not start streaming");
        return false;
    }

    RCLCPP_INFO_STREAM(get_logger(),"Start streaming");
    uvc_error_t res = uvc_start_streaming(devh_, &ctrl_, uvc_streaming_callback, &gsrc, 0);
    if (res != UVC_SUCCESS) {
        RCLCPP_ERROR(get_logger(),"uvc_start_streaming: failed");
    }
    else {
        streaming_ = true;
    }
    return res == UVC_SUCCESS;
}

} // namespace theta_driver


#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(theta_driver::ThetaDriver)