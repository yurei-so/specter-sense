#include "specter_sense/frame_source.hpp"

#include <libfreenect.h>
#include <libfreenect_registration.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <sys/time.h>

namespace specter {
namespace {

class KinectV1Source final : public FrameSource {
 public:
  explicit KinectV1Source(const std::optional<std::string>& serial) {
    try {
      if (freenect_init(&context_, nullptr) < 0) throw std::runtime_error("failed to initialize libfreenect");
      freenect_select_subdevices(context_, FREENECT_DEVICE_CAMERA);
      const int opened = serial
          ? freenect_open_device_by_camera_serial(context_, &device_, serial->c_str())
          : freenect_open_device(context_, &device_, 0);
      if (opened < 0 || !device_) throw std::runtime_error("failed to open Kinect V1 depth camera");
      freenect_set_user(device_, this);
      freenect_set_depth_callback(device_, depth_callback);
      const auto mode = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_MM);
      if (!mode.is_valid || freenect_set_depth_mode(device_, mode) < 0)
        throw std::runtime_error("Kinect V1 metric depth mode is unavailable");

      double x0{}, y0{}, x1{}, y1{};
      freenect_camera_to_world(device_, 0, 0, 1000, &x0, &y0);
      freenect_camera_to_world(device_, 1, 1, 1000, &x1, &y1);
      const double x_step = x1 - x0;
      const double y_step = y1 - y0;
      if (!(x_step > 0) || !(y_step > 0)) throw std::runtime_error("invalid Kinect V1 calibration data");
      intrinsics_ = {1000.0 / x_step, 1000.0 / y_step,
                     -x0 / x_step, -y0 / y_step};

      if (freenect_start_depth(device_) < 0) throw std::runtime_error("failed to start Kinect V1 depth stream");
      streaming_ = true;
    } catch (...) {
      close();
      throw;
    }
  }

  ~KinectV1Source() override { close(); }

  std::optional<DepthFrame> next(std::chrono::milliseconds timeout) override {
    frame_ready_ = false;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!frame_ready_) {
      const auto remaining = deadline - std::chrono::steady_clock::now();
      if (remaining <= std::chrono::steady_clock::duration::zero()) return std::nullopt;
      const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(remaining);
      timeval wait{static_cast<time_t>(micros.count() / 1000000),
                   static_cast<suseconds_t>(micros.count() % 1000000)};
      const int result = freenect_process_events_timeout(context_, &wait);
      if (result < 0) throw std::runtime_error("Kinect V1 USB event processing failed");
    }
    return std::move(frame_);
  }

  std::string name() const override { return "kinect-v1"; }

 private:
  static void depth_callback(freenect_device* device, void* raw, std::uint32_t) {
    auto& source = *static_cast<KinectV1Source*>(freenect_get_user(device));
    constexpr std::size_t width = 640;
    constexpr std::size_t height = 480;
    const auto* depth = static_cast<const std::uint16_t*>(raw);
    source.frame_.width = width;
    source.frame_.height = height;
    source.frame_.depth_mm.resize(width * height);
    std::transform(depth, depth + width * height, source.frame_.depth_mm.begin(),
                   [](std::uint16_t value) { return static_cast<float>(value); });
    source.frame_.intrinsics = source.intrinsics_;
    source.frame_.observed_at = std::chrono::system_clock::now();
    source.frame_ready_ = true;
  }

  void close() noexcept {
    if (device_ && streaming_) freenect_stop_depth(device_);
    streaming_ = false;
    if (device_) freenect_close_device(device_);
    device_ = nullptr;
    if (context_) freenect_shutdown(context_);
    context_ = nullptr;
  }

  freenect_context* context_{};
  freenect_device* device_{};
  Intrinsics intrinsics_;
  DepthFrame frame_;
  bool streaming_{};
  bool frame_ready_{};
};

}  // namespace

std::unique_ptr<FrameSource> make_kinect_v1_source(const std::optional<std::string>& serial) {
  return std::make_unique<KinectV1Source>(serial);
}

std::vector<DiscoveredSensor> discover_kinect_v1_sensors() {
  freenect_context* context = nullptr;
  if (freenect_init(&context, nullptr) < 0) throw std::runtime_error("failed to initialize Kinect V1 discovery");
  freenect_select_subdevices(context, FREENECT_DEVICE_CAMERA);
  freenect_device_attributes* attributes = nullptr;
  const int count = freenect_list_device_attributes(context, &attributes);
  std::vector<DiscoveredSensor> sensors;
  for (auto* item = attributes; item; item = item->next)
    sensors.push_back({"kinect-v1", item->camera_serial && *item->camera_serial
          ? std::optional<std::string>(item->camera_serial) : std::nullopt});
  freenect_free_device_attributes(attributes);
  freenect_shutdown(context);
  if (count < 0) throw std::runtime_error("Kinect V1 discovery failed");
  return sensors;
}

}  // namespace specter
