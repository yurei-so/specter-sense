#pragma once

#include "specter_sense/types.hpp"

#include <chrono>
#include <vector>

namespace specter {

Point3 transform_point(const Transform& transform, const Point3& point);
bool point_in_polygon(const Point2& point, const std::vector<Point2>& polygon);

class OccupancyPipeline {
 public:
  explicit OccupancyPipeline(AppConfig config);
  std::vector<ZoneState> process(const DepthFrame& frame);
  std::size_t frames_seen() const { return frames_seen_; }

 private:
  struct ZoneRuntime {
    bool occupied{};
    bool candidate{};
    bool candidate_initialized{};
    std::chrono::steady_clock::time_point candidate_since{};
  };

  AppConfig config_;
  std::vector<float> background_m_;
  std::vector<ZoneRuntime> runtime_;
  std::size_t width_{};
  std::size_t height_{};
  std::size_t frames_seen_{};
};

}  // namespace specter
