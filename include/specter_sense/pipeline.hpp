#pragma once

#include "specter_sense/types.hpp"

#include <chrono>
#include <vector>

namespace specter {

Point3 transform_point(const Transform& transform, const Point3& point);
Point3 deproject_depth(const Intrinsics& intrinsics, double pixel_x, double pixel_y, double depth_m);
bool point_in_polygon(const Point2& point, const std::vector<Point2>& polygon);

class OccupancyPipeline {
 public:
  explicit OccupancyPipeline(AppConfig config, bool retain_foreground_points = false);
  std::vector<ZoneState> process(const DepthFrame& frame);
  void set_tracking_enabled(bool enabled);
  void reset_tracking();
  std::size_t frames_seen() const { return frames_seen_; }
  const std::vector<Point3>& last_foreground_points() const { return last_foreground_points_; }
  const std::vector<TrackState>& last_tracks() const { return last_tracks_; }

 private:
  struct ZoneRuntime {
    bool occupied{};
    bool candidate{};
    bool candidate_initialized{};
    std::chrono::steady_clock::time_point candidate_since{};
  };

  struct TrackRuntime {
    std::uint64_t id{};
    Point3 centroid;
    Point3 velocity;
    Point3 bounds;
    std::size_t foreground_points{};
    std::string classification{"unknown"};
    double classification_confidence{};
    std::string posture{"unknown"};
    double posture_confidence{};
    std::size_t hits{};
    std::size_t missed{};
    std::size_t posture_candidate_frames{};
    std::string posture_candidate{"unknown"};
    std::chrono::system_clock::time_point observed_at;
    std::vector<std::string> zones;
  };

  AppConfig config_;
  std::vector<float> background_m_;
  std::vector<ZoneRuntime> runtime_;
  std::size_t width_{};
  std::size_t height_{};
  std::size_t frames_seen_{};
  bool retain_foreground_points_{};
  bool tracking_enabled_{};
  std::vector<Point3> last_foreground_points_;
  std::vector<TrackRuntime> tracks_;
  std::vector<TrackState> last_tracks_;
  std::uint64_t next_track_id_{1};
};

}  // namespace specter
