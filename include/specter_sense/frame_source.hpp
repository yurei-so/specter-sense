#pragma once

#include "specter_sense/types.hpp"

#include <chrono>
#include <memory>
#include <optional>
#include <string>

namespace specter {

class FrameSource {
 public:
  virtual ~FrameSource() = default;
  virtual std::optional<DepthFrame> next(std::chrono::milliseconds timeout) = 0;
  virtual std::string name() const = 0;
};

std::unique_ptr<FrameSource> make_synthetic_source();
#ifdef SPECTER_SENSE_HAS_KINECT
std::unique_ptr<FrameSource> make_kinect_source(const std::optional<std::string>& serial = std::nullopt);
#endif

}  // namespace specter
