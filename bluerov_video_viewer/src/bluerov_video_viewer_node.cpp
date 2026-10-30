#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video-frame.h>
#include <gst/video/video-info.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class BlueROVVideoViewer : public rclcpp::Node
{
public:
  BlueROVVideoViewer()
  : Node("bluerov_video_viewer")
  {
    const int port = declare_parameter<int>("port", 5600);
    pipeline_description_ = declare_parameter<std::string>("pipeline", "");
    window_name_ = declare_parameter<std::string>("window_name", "BlueROV camera");
    frame_id_ = declare_parameter<std::string>("frame_id", "bluerov_camera");
    publish_ros_topic_ = declare_parameter<bool>("publish_ros_topic", true);
    show_window_ = declare_parameter<bool>("show_window", true);
    frame_timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(declare_parameter<double>("frame_timeout", 2.0)));
    reconnect_interval_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(declare_parameter<double>("reconnect_interval", 1.0)));

    if (port < 1 || port > 65535) {
      throw std::invalid_argument("Parameter 'port' must be between 1 and 65535");
    }
    if (frame_timeout_.count() <= 0.0 || reconnect_interval_.count() <= 0.0) {
      throw std::invalid_argument("frame_timeout and reconnect_interval must be greater than zero");
    }

    if (pipeline_description_.empty()) {
      pipeline_description_ =
        "udpsrc port=" + std::to_string(port) +
        " caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96\" ! "
        "rtpjitterbuffer latency=100 drop-on-latency=true ! "
        "rtph264depay ! avdec_h264 ! videoconvert ! "
        "video/x-raw,format=BGR ! appsink name=appsink max-buffers=1 drop=true sync=false";
    }

    gst_init(nullptr, nullptr);

    if (publish_ros_topic_) {
      publisher_ = create_publisher<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS());
    }

    if (show_window_) {
      cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    }

    if (!open_pipeline()) {
      throw std::runtime_error(
              "Could not open the GStreamer pipeline. Check its syntax and installed plugins.");
    }

    // try_pull_sample() below has a finite timeout, so this timer never blocks permanently.
    timer_ = create_wall_timer(1ms, std::bind(&BlueROVVideoViewer::read_frame, this));
  }

  ~BlueROVVideoViewer() override
  {
    close_pipeline();
    if (show_window_) {
      cv::destroyWindow(window_name_);
    }
  }

private:
  bool open_pipeline()
  {
    close_pipeline();

    GError * error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_description_.c_str(), &error);
    if (error != nullptr) {
      RCLCPP_ERROR(get_logger(), "Could not parse GStreamer pipeline: %s", error->message);
      g_error_free(error);
      close_pipeline();
      return false;
    }
    if (pipeline_ == nullptr) {
      RCLCPP_ERROR(get_logger(), "GStreamer returned an empty pipeline");
      return false;
    }

    GstElement * sink = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink");
    if (sink == nullptr) {
      // GStreamer uses this name when a custom pipeline contains one unnamed appsink.
      sink = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink0");
    }
    if (sink == nullptr || !GST_IS_APP_SINK(sink)) {
      RCLCPP_ERROR(
        get_logger(), "The pipeline must end in an appsink (preferably named 'appsink')");
      if (sink != nullptr) {
        gst_object_unref(sink);
      }
      close_pipeline();
      return false;
    }
    appsink_ = GST_APP_SINK(sink);
    gst_app_sink_set_emit_signals(appsink_, false);
    gst_app_sink_set_drop(appsink_, true);
    gst_app_sink_set_max_buffers(appsink_, 1);

    const GstStateChangeReturn state_result = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state_result == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "GStreamer pipeline could not enter the PLAYING state");
      close_pipeline();
      return false;
    }

    last_frame_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Listening for the BlueROV video stream");
    return true;
  }

  void close_pipeline()
  {
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (appsink_ != nullptr) {
      gst_object_unref(appsink_);
      appsink_ = nullptr;
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

  void schedule_reconnect(const std::string & reason)
  {
    RCLCPP_WARN(get_logger(), "%s; restarting the video pipeline", reason.c_str());
    close_pipeline();
    next_reconnect_time_ = std::chrono::steady_clock::now() + reconnect_interval_;
  }

  bool pipeline_has_error()
  {
    GstBus * bus = gst_element_get_bus(pipeline_);
    GstMessage * message = gst_bus_pop_filtered(
      bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    gst_object_unref(bus);
    if (message == nullptr) {
      return false;
    }

    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
      GError * error = nullptr;
      gchar * debug = nullptr;
      gst_message_parse_error(message, &error, &debug);
      RCLCPP_WARN(
        get_logger(), "GStreamer error: %s", error != nullptr ? error->message : "unknown error");
      if (error != nullptr) {
        g_error_free(error);
      }
      g_free(debug);
    } else {
      RCLCPP_WARN(get_logger(), "GStreamer pipeline reached end of stream");
    }
    gst_message_unref(message);
    return true;
  }

  void read_frame()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (pipeline_ == nullptr) {
      if (steady_now >= next_reconnect_time_) {
        if (!open_pipeline()) {
          next_reconnect_time_ = steady_now + reconnect_interval_;
        }
      }
      return;
    }

    // A short timed wait prevents a missing UDP stream from wedging the ROS executor.
    GstSample * sample = gst_app_sink_try_pull_sample(appsink_, 100 * GST_MSECOND);
    if (sample == nullptr) {
      if (pipeline_has_error()) {
        schedule_reconnect("The decoder stopped");
      } else if (steady_now - last_frame_time_ >= frame_timeout_) {
        schedule_reconnect("No video frames received before frame_timeout");
      }
      return;
    }

    last_frame_time_ = std::chrono::steady_clock::now();
    process_sample(sample);
    gst_sample_unref(sample);
  }

  void process_sample(GstSample * sample)
  {
    GstCaps * caps = gst_sample_get_caps(sample);
    GstBuffer * buffer = gst_sample_get_buffer(sample);
    GstVideoInfo info;
    if (caps == nullptr || buffer == nullptr || !gst_video_info_from_caps(&info, caps) ||
      GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_BGR)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "The appsink did not provide a valid BGR video frame");
      return;
    }

    GstVideoFrame video_frame;
    if (!gst_video_frame_map(&video_frame, &info, buffer, GST_MAP_READ)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Could not map a video frame");
      return;
    }

    const int width = static_cast<int>(GST_VIDEO_FRAME_WIDTH(&video_frame));
    const int height = static_cast<int>(GST_VIDEO_FRAME_HEIGHT(&video_frame));
    const auto stride = static_cast<std::size_t>(GST_VIDEO_FRAME_PLANE_STRIDE(&video_frame, 0));
    auto * data = GST_VIDEO_FRAME_PLANE_DATA(&video_frame, 0);
    cv::Mat frame(height, width, CV_8UC3, data, stride);

    if (show_window_) {
      cv::imshow(window_name_, frame);
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q' || key == 'Q') {
        RCLCPP_INFO(get_logger(), "Viewer closed by keyboard request");
        gst_video_frame_unmap(&video_frame);
        rclcpp::shutdown();
        return;
      }
    }

    if (publish_ros_topic_) {
      auto message = std::make_unique<sensor_msgs::msg::Image>();
      message->header.stamp = now();
      message->header.frame_id = frame_id_;
      message->height = static_cast<uint32_t>(height);
      message->width = static_cast<uint32_t>(width);
      message->encoding = sensor_msgs::image_encodings::BGR8;
      message->is_bigendian = false;
      message->step = static_cast<sensor_msgs::msg::Image::_step_type>(width * 3);
      message->data.resize(static_cast<std::size_t>(message->step) * message->height);
      for (int row = 0; row < height; ++row) {
        std::memcpy(
          message->data.data() + static_cast<std::size_t>(row) * message->step,
          frame.ptr(row), message->step);
      }
      publisher_->publish(std::move(message));
    }

    gst_video_frame_unmap(&video_frame);
  }

  std::string pipeline_description_;
  std::string window_name_;
  std::string frame_id_;
  bool publish_ros_topic_{};
  bool show_window_{};
  std::chrono::steady_clock::duration frame_timeout_{};
  std::chrono::steady_clock::duration reconnect_interval_{};
  std::chrono::steady_clock::time_point last_frame_time_{};
  std::chrono::steady_clock::time_point next_reconnect_time_{};
  GstElement * pipeline_{nullptr};
  GstAppSink * appsink_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<BlueROVVideoViewer>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("bluerov_video_viewer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
