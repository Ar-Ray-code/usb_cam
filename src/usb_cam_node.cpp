// Copyright 2014 Robert Bosch, LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Robert Bosch, LLC nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <filesystem>
#include <optional>
#include "usb_cam/usb_cam_node.hpp"
#include "usb_cam/utils.hpp"

const char BASE_TOPIC_NAME[] = "image_raw";

namespace usb_cam
{

UsbCamNode::UsbCamNode(const rclcpp::NodeOptions & node_options)
: Node("usb_cam", node_options),
  m_camera(new usb_cam::UsbCam()),
  m_image_msg(new sensor_msgs::msg::Image()),
  m_compressed_img_msg(nullptr),
  m_image_publisher(std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(this, BASE_TOPIC_NAME,
      rclcpp::QoS {100}.get_rmw_qos_profile()))),
  m_compressed_image_publisher(nullptr),
  m_compressed_cam_info_publisher(nullptr),
  m_parameters(),
  m_camera_info_msg(new sensor_msgs::msg::CameraInfo()),
  m_service_capture(
    this->create_service<std_srvs::srv::SetBool>(
      "set_capture",
      std::bind(
        &UsbCamNode::service_capture,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3)))
{
  // declare params
  this->declare_parameter("camera_name", "default_cam");
  this->declare_parameter("camera_info_url", "");
  this->declare_parameter("framerate", 30.0);
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", 480);
  this->declare_parameter("image_width", 640);
  this->declare_parameter("io_method", "mmap");
  this->declare_parameter("pixel_format", "yuyv");
  this->declare_parameter("av_device_format", "YUV422P");
  this->declare_parameter("usb_port_id", "");
  this->declare_parameter("video_device", "/dev/video0");
  this->declare_parameter("brightness", 50);  // 0-255, -1 "leave alone"
  this->declare_parameter("contrast", -1);    // 0-255, -1 "leave alone"
  this->declare_parameter("saturation", -1);  // 0-255, -1 "leave alone"
  this->declare_parameter("sharpness", -1);   // 0-255, -1 "leave alone"
  this->declare_parameter("gain", -1);        // 0-100?, -1 "leave alone"
  this->declare_parameter("auto_white_balance", true);
  this->declare_parameter("white_balance", 4000);
  this->declare_parameter("autoexposure", true);
  this->declare_parameter("exposure", 100);
  this->declare_parameter("autofocus", false);
  this->declare_parameter("focus", -1);  // 0-255, -1 "leave alone"

  get_params();
  init();
  m_parameters_callback_handle = add_on_set_parameters_callback(
    std::bind(
      &UsbCamNode::parameters_callback, this,
      std::placeholders::_1));
}

UsbCamNode::~UsbCamNode()
{
  RCLCPP_WARN(this->get_logger(), "Shutting down");
  m_image_msg.reset();
  m_compressed_img_msg.reset();
  m_camera_info_msg.reset();
  m_camera_info.reset();
  m_timer.reset();
  m_service_capture.reset();
  m_parameters_callback_handle.reset();

  delete (m_camera);
}

void UsbCamNode::service_capture(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void) request_header;
  if (request->data) {
    m_camera->start_capturing();
    response->message = "Start Capturing";
  } else {
    m_camera->stop_capturing();
    response->message = "Stop Capturing";
  }
}

std::string resolve_device_path(const std::string & path)
{
  if (std::filesystem::is_symlink(path)) {
    std::filesystem::path target_path = std::filesystem::read_symlink(path);

    // if the target path is relative, resolve it
    if (target_path.is_relative()) {
      target_path = std::filesystem::absolute(path).parent_path() / target_path;
      target_path = std::filesystem::canonical(target_path);
    }

    return target_path.string();
  }
  return path;
}

// Attempt to find a /dev/videoX device corresponding to a given USB port id
extern "C" {
#include <linux/videodev2.h>
#include <fcntl.h>
}

// Determine if a v4l2 pixel format is color
static bool is_color_fourcc(uint32_t fourcc)
{
  switch (fourcc) {
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_RGB24:
    case V4L2_PIX_FMT_MJPEG:
#ifdef V4L2_PIX_FMT_M420
    case V4L2_PIX_FMT_M420:
#endif
    case V4L2_PIX_FMT_YUV420:
    case V4L2_PIX_FMT_YUV422P:
      return true;
    default:
      return false;
  }
}

static std::optional<uint32_t> preferred_fourcc_from_name(const std::string & name)
{
  // Map driver pixel_format names to v4l2 fourcc where possible
  if (name == "yuyv" || name == "yuyv2rgb") return V4L2_PIX_FMT_YUYV;
  if (name == "uyvy" || name == "uyvy2rgb") return V4L2_PIX_FMT_UYVY;
  if (name == "rgb8") return V4L2_PIX_FMT_RGB24;
  if (name == "mjpeg" || name == "mjpeg2rgb" || name == "raw_mjpeg") return V4L2_PIX_FMT_MJPEG;
  if (name == "m4202rgb") {
#ifdef V4L2_PIX_FMT_M420
    return V4L2_PIX_FMT_M420;
#else
    return std::nullopt;
#endif
  }
  // mono formats intentionally omitted
  return std::nullopt;
}

static bool device_has_preferred_color_format(const std::string & dev, const std::string & preferred_name)
{
  int fd = open(dev.c_str(), O_RDONLY);
  if (fd == -1) {
    return false;
  }
  auto preferred_fourcc = preferred_fourcc_from_name(preferred_name);
  if (!preferred_fourcc) {
    close(fd);
    return false;
  }

  struct v4l2_fmtdesc fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  for (fmt.index = 0; usb_cam::utils::xioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0; ++fmt.index) {
    if (fmt.pixelformat == preferred_fourcc.value()) { close(fd); return true; }
  }
  close(fd);
  return false;
}

static bool device_has_any_color_format(const std::string & dev)
{
  int fd = open(dev.c_str(), O_RDONLY);
  if (fd == -1) {
    return false;
  }
  struct v4l2_fmtdesc fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  for (fmt.index = 0; usb_cam::utils::xioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0; ++fmt.index) {
    if (is_color_fourcc(fmt.pixelformat)) { close(fd); return true; }
  }
  close(fd);
  return false;
}

static std::string find_video_device_by_usb_port_id(const std::string & usb_port_id,
  const std::string & preferred_pixel_format_name)
{
  if (usb_port_id.empty()) {
    return "";
  }

  const std::string v4l2_symlinks_dir = "/sys/class/video4linux/";
  std::map<std::string, v4l2_capability> candidates = usb_cam::utils::available_devices();

  // Prefer capture-capable nodes
  auto is_capture_cap = [](const v4l2_capability & caps) -> bool {
    return (caps.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
           (caps.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE);
  };

  std::string first_match;
  std::string first_color_match;
  for (const auto & entry : std::filesystem::directory_iterator(v4l2_symlinks_dir)) {
    if (!std::filesystem::is_symlink(entry)) {
      continue;
    }

    // Resolve canonical target under /sys/devices
    auto target = std::filesystem::canonical(
      v4l2_symlinks_dir + std::filesystem::read_symlink(entry).generic_string());

    const auto target_str = target.generic_string();
    // Match either "/<id>/" or "/<id>:" patterns in the path
    const std::string pat1 = std::string("/") + usb_port_id + "/";
    const std::string pat2 = std::string("/") + usb_port_id + ":";
    if (target_str.find(pat1) == std::string::npos && target_str.find(pat2) == std::string::npos) {
      continue;
    }

    // Parse DEVNAME from uevent to get /dev/videoX
    std::ifstream uevent_file(target_str + "/uevent");
    std::string line;
    std::string devname;
    while (std::getline(uevent_file, line)) {
      auto dev_name_index = line.find("DEVNAME=");
      if (dev_name_index != std::string::npos) {
        devname = std::string("/dev/") + line.substr(dev_name_index + 8);
        break;
      }
    }

    if (devname.empty()) {
      continue;
    }

    auto it = candidates.find(devname);
    if (it != candidates.end() && is_capture_cap(it->second)) {
      // Prefer device that supports preferred color format, then any color format
      if (!preferred_pixel_format_name.empty() &&
          preferred_fourcc_from_name(preferred_pixel_format_name).has_value() &&
          device_has_preferred_color_format(devname, preferred_pixel_format_name)) {
        return devname;
      }
      if (first_color_match.empty() && device_has_any_color_format(devname)) {
        first_color_match = devname;
      }
      if (first_match.empty()) {
        first_match = devname;
      }
    }
  }

  if (!first_color_match.empty()) return first_color_match;
  return first_match;  // Fallback to any match
}

void UsbCamNode::init()
{
  while (m_parameters.frame_id == "") {
    RCLCPP_WARN_ONCE(
      this->get_logger(), "Required Parameters not set...waiting until they are set");
    get_params();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // load the camera info
  m_camera_info.reset(
    new camera_info_manager::CameraInfoManager(
      this, m_parameters.camera_name, m_parameters.camera_info_url));
  // check for default camera info
  if (!m_camera_info->isCalibrated()) {
    m_camera_info->setCameraName(m_parameters.device_name);
    m_camera_info_msg->header.frame_id = m_parameters.frame_id;
    m_camera_info_msg->width = m_parameters.image_width;
    m_camera_info_msg->height = m_parameters.image_height;
    m_camera_info->setCameraInfo(*m_camera_info_msg);
  }

  // Check if given device name is an available v4l2 device
  auto available_devices = usb_cam::utils::available_devices();
  if (available_devices.find(m_parameters.device_name) == available_devices.end()) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "Device specified is not available or is not a vaild V4L2 device: `" <<
        m_parameters.device_name << "`"
    );
    RCLCPP_INFO(this->get_logger(), "Available V4L2 devices are:");
    for (const auto & device : available_devices) {
      RCLCPP_INFO_STREAM(this->get_logger(), "    " << device.first);
      RCLCPP_INFO_STREAM(this->get_logger(), "        " << device.second.card);
    }
    rclcpp::shutdown();
    return;
  }

  // if pixel format is equal to 'mjpeg', i.e. raw mjpeg stream, initialize compressed image message
  // and publisher
  if (m_parameters.pixel_format_name == "mjpeg") {
    m_compressed_img_msg.reset(new sensor_msgs::msg::CompressedImage());
    m_compressed_img_msg->header.frame_id = m_parameters.frame_id;
    m_compressed_image_publisher =
      this->create_publisher<sensor_msgs::msg::CompressedImage>(
      std::string(BASE_TOPIC_NAME) + "/compressed", rclcpp::QoS(100));
    m_compressed_cam_info_publisher =
      this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::QoS(100));
  }

  m_image_msg->header.frame_id = m_parameters.frame_id;
  RCLCPP_INFO(
    this->get_logger(), "Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS",
    m_parameters.camera_name.c_str(), m_parameters.device_name.c_str(),
    m_parameters.image_width, m_parameters.image_height, m_parameters.io_method_name.c_str(),
    m_parameters.pixel_format_name.c_str(), m_parameters.framerate);
  // set the IO method
  io_method_t io_method =
    usb_cam::utils::io_method_from_string(m_parameters.io_method_name);
  if (io_method == usb_cam::utils::IO_METHOD_UNKNOWN) {
    RCLCPP_ERROR_ONCE(
      this->get_logger(),
      "Unknown IO method '%s'", m_parameters.io_method_name.c_str());
    rclcpp::shutdown();
    return;
  }

  // configure the camera
  m_camera->configure(m_parameters, io_method);

  set_v4l2_params();

  // start the camera
  m_camera->start();

  // TODO(lucasw) should this check a little faster than expected frame rate?
  // TODO(lucasw) how to do small than ms, or fractional ms- std::chrono::nanoseconds?
  const int period_ms = 1000.0 / m_parameters.framerate;
  m_timer = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
    std::bind(&UsbCamNode::update, this));
  RCLCPP_INFO_STREAM(this->get_logger(), "Timer triggering every " << period_ms << " ms");
}

void UsbCamNode::get_params()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  auto parameters = parameters_client->get_parameters(
    {
      "camera_name", "camera_info_url", "frame_id", "framerate", "image_height", "image_width",
      "io_method", "pixel_format", "av_device_format", "usb_port_id", "video_device", "brightness", "contrast",
      "saturation", "sharpness", "gain", "auto_white_balance", "white_balance", "autoexposure",
      "exposure", "autofocus", "focus"
    }
  );

  assign_params(parameters);
}

void UsbCamNode::assign_params(const std::vector<rclcpp::Parameter> & parameters)
{
  for (auto & parameter : parameters) {
    if (parameter.get_name() == "camera_name") {
      RCLCPP_INFO(this->get_logger(), "camera_name value: %s", parameter.value_to_string().c_str());
      m_parameters.camera_name = parameter.value_to_string();
    } else if (parameter.get_name() == "camera_info_url") {
      m_parameters.camera_info_url = parameter.value_to_string();
    } else if (parameter.get_name() == "frame_id") {
      m_parameters.frame_id = parameter.value_to_string();
    } else if (parameter.get_name() == "framerate") {
      RCLCPP_WARN(this->get_logger(), "framerate: %f", parameter.as_double());
      m_parameters.framerate = parameter.as_double();
    } else if (parameter.get_name() == "image_height") {
      m_parameters.image_height = parameter.as_int();
    } else if (parameter.get_name() == "image_width") {
      m_parameters.image_width = parameter.as_int();
    } else if (parameter.get_name() == "io_method") {
      m_parameters.io_method_name = parameter.value_to_string();
    } else if (parameter.get_name() == "pixel_format") {
      m_parameters.pixel_format_name = parameter.value_to_string();
    } else if (parameter.get_name() == "av_device_format") {
      m_parameters.av_device_format = parameter.value_to_string();
    } else if (parameter.get_name() == "usb_port_id") {
      const auto port_id = parameter.value_to_string();
      if (!port_id.empty()) {
        // Prefer current pixel_format if set
        const auto dev = find_video_device_by_usb_port_id(port_id, m_parameters.pixel_format_name);
        if (!dev.empty()) {
          RCLCPP_INFO(this->get_logger(), "usb_port_id '%s' resolved to device '%s'",
            port_id.c_str(), dev.c_str());
          m_parameters.device_name = dev;
        } else {
          RCLCPP_ERROR(this->get_logger(),
            "usb_port_id '%s' did not resolve to any V4L2 device", port_id.c_str());
        }
      }
    } else if (parameter.get_name() == "video_device") {
      auto current_port_id = this->get_parameter("usb_port_id").as_string();
      if (current_port_id.empty()) {
        m_parameters.device_name = resolve_device_path(parameter.value_to_string());
      } else {
        RCLCPP_INFO(this->get_logger(),
          "Ignoring 'video_device' because 'usb_port_id' is set to '%s'",
          current_port_id.c_str());
      }
    } else if (parameter.get_name() == "brightness") {
      m_parameters.brightness = parameter.as_int();
    } else if (parameter.get_name() == "contrast") {
      m_parameters.contrast = parameter.as_int();
    } else if (parameter.get_name() == "saturation") {
      m_parameters.saturation = parameter.as_int();
    } else if (parameter.get_name() == "sharpness") {
      m_parameters.sharpness = parameter.as_int();
    } else if (parameter.get_name() == "gain") {
      m_parameters.gain = parameter.as_int();
    } else if (parameter.get_name() == "auto_white_balance") {
      m_parameters.auto_white_balance = parameter.as_bool();
    } else if (parameter.get_name() == "white_balance") {
      m_parameters.white_balance = parameter.as_int();
    } else if (parameter.get_name() == "autoexposure") {
      m_parameters.autoexposure = parameter.as_bool();
    } else if (parameter.get_name() == "exposure") {
      m_parameters.exposure = parameter.as_int();
    } else if (parameter.get_name() == "autofocus") {
      m_parameters.autofocus = parameter.as_bool();
    } else if (parameter.get_name() == "focus") {
      m_parameters.focus = parameter.as_int();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid parameter name: %s", parameter.get_name().c_str());
    }
  }
}

/// @brief Send current parameters to V4L2 device
/// TODO(flynneva): should this actuaully be part of UsbCam class?
void UsbCamNode::set_v4l2_params()
{
  // set camera parameters
  if (m_parameters.brightness >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'brightness' to %d", m_parameters.brightness);
    m_camera->set_v4l_parameter("brightness", m_parameters.brightness);
  }

  if (m_parameters.contrast >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'contrast' to %d", m_parameters.contrast);
    m_camera->set_v4l_parameter("contrast", m_parameters.contrast);
  }

  if (m_parameters.saturation >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'saturation' to %d", m_parameters.saturation);
    m_camera->set_v4l_parameter("saturation", m_parameters.saturation);
  }

  if (m_parameters.sharpness >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'sharpness' to %d", m_parameters.sharpness);
    m_camera->set_v4l_parameter("sharpness", m_parameters.sharpness);
  }

  if (m_parameters.gain >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'gain' to %d", m_parameters.gain);
    m_camera->set_v4l_parameter("gain", m_parameters.gain);
  }

  // check auto white balance
  if (m_parameters.auto_white_balance) {
    m_camera->set_v4l_parameter("white_balance_temperature_auto", 1);
    RCLCPP_INFO(this->get_logger(), "Setting 'white_balance_temperature_auto' to %d", 1);
  } else {
    RCLCPP_INFO(this->get_logger(), "Setting 'white_balance' to %d", m_parameters.white_balance);
    m_camera->set_v4l_parameter("white_balance_temperature_auto", 0);
    m_camera->set_v4l_parameter("white_balance_temperature", m_parameters.white_balance);
  }

  // check auto exposure
  if (!m_parameters.autoexposure) {
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure_auto' to %d", 1);
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure' to %d", m_parameters.exposure);
    // turn down exposure control (from max of 3)
    m_camera->set_v4l_parameter("exposure_auto", 1);
    // change the exposure level
    m_camera->set_v4l_parameter("exposure_absolute", m_parameters.exposure);
  } else {
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure_auto' to %d", 3);
    m_camera->set_v4l_parameter("exposure_auto", 3);
  }

  // check auto focus
  if (m_parameters.autofocus) {
    m_camera->set_auto_focus(1);
    RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 1);
    m_camera->set_v4l_parameter("focus_auto", 1);
  } else {
    RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 0);
    m_camera->set_v4l_parameter("focus_auto", 0);
    if (m_parameters.focus >= 0) {
      RCLCPP_INFO(this->get_logger(), "Setting 'focus_absolute' to %d", m_parameters.focus);
      m_camera->set_v4l_parameter("focus_absolute", m_parameters.focus);
    }
  }
}

bool UsbCamNode::take_and_send_image()
{
  // Only resize if required
  if (sizeof(m_image_msg->data) != m_camera->get_image_size_in_bytes()) {
    m_image_msg->width = m_camera->get_image_width();
    m_image_msg->height = m_camera->get_image_height();
    m_image_msg->encoding = m_camera->get_pixel_format()->ros();
    m_image_msg->step = m_camera->get_image_step();
    if (m_image_msg->step == 0) {
      // Some formats don't have a linesize specified by v4l2
      // Fall back to manually calculating it step = size / height
      m_image_msg->step = m_camera->get_image_size_in_bytes() / m_image_msg->height;
    }
    m_image_msg->data.resize(m_camera->get_image_size_in_bytes());
  }

  // grab the image, pass image msg buffer to fill
  m_camera->get_image(reinterpret_cast<char *>(&m_image_msg->data[0]));

  auto stamp = m_camera->get_image_timestamp();
  m_image_msg->header.stamp.sec = stamp.tv_sec;
  m_image_msg->header.stamp.nanosec = stamp.tv_nsec;

  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_image_msg->header;
  m_image_publisher->publish(*m_image_msg, *m_camera_info_msg);
  return true;
}

bool UsbCamNode::take_and_send_image_mjpeg()
{
  // Only resize if required
  if (sizeof(m_compressed_img_msg->data) != m_camera->get_image_size_in_bytes()) {
    m_compressed_img_msg->format = "jpeg";
    m_compressed_img_msg->data.resize(m_camera->get_image_size_in_bytes());
  }

  // grab the image, pass image msg buffer to fill
  m_camera->get_image(reinterpret_cast<char *>(&m_compressed_img_msg->data[0]));

  auto stamp = m_camera->get_image_timestamp();
  m_compressed_img_msg->header.stamp.sec = stamp.tv_sec;
  m_compressed_img_msg->header.stamp.nanosec = stamp.tv_nsec;

  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_compressed_img_msg->header;

  m_compressed_image_publisher->publish(*m_compressed_img_msg);
  m_compressed_cam_info_publisher->publish(*m_camera_info_msg);
  return true;
}

rcl_interfaces::msg::SetParametersResult UsbCamNode::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  RCLCPP_DEBUG(this->get_logger(), "Setting parameters for %s", m_parameters.camera_name.c_str());
  m_timer->reset();
  assign_params(parameters);
  set_v4l2_params();
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void UsbCamNode::update()
{
  if (m_camera->is_capturing()) {
    // If the camera exposure longer higher than the framerate period
    // then that caps the framerate.
    // auto t0 = now();
    bool isSuccessful = (m_parameters.pixel_format_name == "mjpeg") ?
      take_and_send_image_mjpeg() :
      take_and_send_image();
    if (!isSuccessful) {
      RCLCPP_WARN_ONCE(this->get_logger(), "USB camera did not respond in time.");
    }
  }
}
}  // namespace usb_cam


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usb_cam::UsbCamNode)
