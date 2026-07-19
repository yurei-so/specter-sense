#include "specter_sense/frame_source.hpp"

#include <thread>

namespace specter {
namespace {

class SyntheticSource final : public FrameSource {
 public:
  std::optional<DepthFrame> next(std::chrono::milliseconds timeout) override {
    std::this_thread::sleep_for(std::min(timeout, std::chrono::milliseconds(33)));
    constexpr std::size_t width = 64;
    constexpr std::size_t height = 48;
    DepthFrame frame{width, height, std::vector<float>(width * height, 2500.0F),
                     {60.0, 60.0, 31.5, 23.5}, std::chrono::system_clock::now()};
    if (frame_number_ >= 35 && frame_number_ < 95) {
      for (std::size_t y = 16; y < 37; ++y)
        for (std::size_t x = 24; x < 41; ++x) frame.depth_mm[y * width + x] = 1500.0F;
    }
    ++frame_number_;
    return frame;
  }
  std::string name() const override { return "synthetic"; }

 private:
  std::size_t frame_number_{};
};

}  // namespace

std::unique_ptr<FrameSource> make_synthetic_source() {
  return std::make_unique<SyntheticSource>();
}

}  // namespace specter
