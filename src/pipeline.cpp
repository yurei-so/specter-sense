#include "specter_sense/pipeline.hpp"
#include "specter_sense/config.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace specter {

Point3 transform_point(const Transform& transform, const Point3& point) {
  const auto& m = transform.matrix;
  return {
      m[0] * point.x + m[1] * point.y + m[2] * point.z + m[3],
      m[4] * point.x + m[5] * point.y + m[6] * point.z + m[7],
      m[8] * point.x + m[9] * point.y + m[10] * point.z + m[11]};
}

bool point_in_polygon(const Point2& point, const std::vector<Point2>& polygon) {
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto& a = polygon[i];
    const auto& b = polygon[j];
    const bool crosses = ((a.y > point.y) != (b.y > point.y)) &&
                         (point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x);
    if (crosses) inside = !inside;
  }
  return inside;
}

OccupancyPipeline::OccupancyPipeline(AppConfig config, bool retain_foreground_points)
    : config_(std::move(config)), runtime_(config_.zones.size()),
      retain_foreground_points_(retain_foreground_points) {
  validate_config(config_);
}

std::vector<ZoneState> OccupancyPipeline::process(const DepthFrame& frame) {
  if (frame.width == 0 || frame.height == 0 || frame.depth_mm.size() != frame.width * frame.height)
    throw std::runtime_error("invalid depth frame dimensions");
  if (!(frame.intrinsics.fx > 0 && frame.intrinsics.fy > 0))
    throw std::runtime_error("invalid depth intrinsics");
  if (background_m_.empty()) {
    width_ = frame.width;
    height_ = frame.height;
    background_m_.assign(frame.depth_mm.size(), std::numeric_limits<float>::quiet_NaN());
  } else if (frame.width != width_ || frame.height != height_) {
    throw std::runtime_error("depth frame dimensions changed");
  }

  struct Evidence {
    std::size_t count{};
    Point3 sum{};
    double near_m{std::numeric_limits<double>::infinity()};
    double far_m{};
  };
  std::vector<Evidence> evidence(config_.zones.size());
  last_foreground_points_.clear();
  const auto& p = config_.processing;

  for (std::size_t index = 0; index < frame.depth_mm.size(); ++index) {
    const double depth_m = static_cast<double>(frame.depth_mm[index]) / 1000.0;
    if (!std::isfinite(depth_m) || depth_m < p.min_depth_m || depth_m > p.max_depth_m) continue;
    auto& background = background_m_[index];
    if (!std::isfinite(background)) {
      background = static_cast<float>(depth_m);
      continue;
    }
    const bool foreground = frames_seen_ >= p.warmup_frames &&
                            static_cast<double>(background) - depth_m >= p.foreground_delta_m;
    if (!foreground) {
      background = static_cast<float>((1.0 - p.background_alpha) * background + p.background_alpha * depth_m);
      continue;
    }
    const auto u = static_cast<double>(index % frame.width);
    const auto v = static_cast<double>(index / frame.width);
    const Point3 camera{
        (u - frame.intrinsics.cx) * depth_m / frame.intrinsics.fx,
        (v - frame.intrinsics.cy) * depth_m / frame.intrinsics.fy,
        depth_m};
    const Point3 room = transform_point(config_.camera_to_room, camera);
    if (retain_foreground_points_) last_foreground_points_.push_back(room);
    for (std::size_t zone_index = 0; zone_index < config_.zones.size(); ++zone_index) {
      const auto& zone = config_.zones[zone_index];
      if (room.z < zone.min_height_m || room.z > zone.max_height_m ||
          !point_in_polygon({room.x, room.y}, zone.floor_polygon)) continue;
      auto& item = evidence[zone_index];
      ++item.count;
      item.sum.x += room.x;
      item.sum.y += room.y;
      item.sum.z += room.z;
      item.near_m = std::min(item.near_m, depth_m);
      item.far_m = std::max(item.far_m, depth_m);
    }
  }

  ++frames_seen_;
  const auto now = std::chrono::steady_clock::now();
  std::vector<ZoneState> states;
  states.reserve(config_.zones.size());
  for (std::size_t i = 0; i < config_.zones.size(); ++i) {
    const auto& zone = config_.zones[i];
    auto& rt = runtime_[i];
    const auto threshold = rt.occupied ? zone.exit_points : zone.enter_points;
    const bool candidate = evidence[i].count >= threshold;
    if (!rt.candidate_initialized || candidate != rt.candidate) {
      rt.candidate = candidate;
      rt.candidate_initialized = true;
      rt.candidate_since = now;
    }
    const auto required = candidate ? zone.enter_after : zone.exit_after;
    if (candidate != rt.occupied && now - rt.candidate_since >= required) rt.occupied = candidate;

    ZoneState state;
    state.name = zone.name;
    state.occupied = rt.occupied;
    state.foreground_points = evidence[i].count;
    state.occupancy_score = zone.enter_points == 0
        ? (evidence[i].count > 0 ? 1.0 : 0.0)
        : std::min(1.0, static_cast<double>(evidence[i].count) / static_cast<double>(zone.enter_points));
    state.observed_at = frame.observed_at;
    if (evidence[i].count > 0) {
      const double count = static_cast<double>(evidence[i].count);
      state.centroid_m = Point3{evidence[i].sum.x / count, evidence[i].sum.y / count, evidence[i].sum.z / count};
      state.nearest_range_m = evidence[i].near_m;
      state.farthest_range_m = evidence[i].far_m;
    }
    states.push_back(std::move(state));
  }
  return states;
}

}  // namespace specter
