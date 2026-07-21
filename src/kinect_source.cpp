#include "specter_sense/frame_source.hpp"

#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/libfreenect2.hpp>

#include <cstring>
#include <stdexcept>

namespace specter {
namespace {

class KinectSource final : public FrameSource {
 public:
  explicit KinectSource(const std::optional<std::string>& serial) : listener_(libfreenect2::Frame::Depth) {
    if (freenect2_.enumerateDevices() == 0) throw std::runtime_error("no Kinect V2 found");
    device_ = serial ? freenect2_.openDevice(*serial) : freenect2_.openDefaultDevice();
    if (!device_) throw std::runtime_error("failed to open Kinect V2");
    device_->setIrAndDepthFrameListener(&listener_);
    if (!device_->startStreams(false, true)) throw std::runtime_error("failed to start Kinect depth stream");
    const auto params = device_->getIrCameraParams();
    intrinsics_ = {params.fx, params.fy, params.cx, params.cy};
  }

  ~KinectSource() override {
    if (device_) {
      device_->stop();
      device_->close();
    }
  }

  std::optional<DepthFrame> next(std::chrono::milliseconds timeout) override {
    libfreenect2::FrameMap frames;
    if (!listener_.waitForNewFrame(frames, static_cast<int>(timeout.count()))) return std::nullopt;
    auto* depth = frames[libfreenect2::Frame::Depth];
    DepthFrame result;
    result.width = static_cast<std::size_t>(depth->width);
    result.height = static_cast<std::size_t>(depth->height);
    result.depth_mm.resize(result.width * result.height);
    std::memcpy(result.depth_mm.data(), depth->data, result.depth_mm.size() * sizeof(float));
    result.intrinsics = intrinsics_;
    result.observed_at = std::chrono::system_clock::now();
    listener_.release(frames);
    return result;
  }

  std::string name() const override { return "kinect-v2"; }

 private:
  libfreenect2::Freenect2 freenect2_;
  libfreenect2::Freenect2Device* device_{};
  libfreenect2::SyncMultiFrameListener listener_;
  Intrinsics intrinsics_;
};

}  // namespace

std::unique_ptr<FrameSource> make_kinect_source(const std::optional<std::string>& serial) {
  return std::make_unique<KinectSource>(serial);
}

}  // namespace specter
