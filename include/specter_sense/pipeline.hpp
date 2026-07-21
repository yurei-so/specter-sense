#pragma once

#include "specter_sense/types.hpp"

#include <chrono>
#include <limits>
#include <optional>
#include <vector>

namespace specter {

Point3 transform_point(const Transform& transform, const Point3& point);
Transform invert_transform(const Transform& transform);
Point3 deproject_depth(const Intrinsics& intrinsics, double pixel_x, double pixel_y, double depth_m);
std::optional<Point2> project_room_to_depth(
    const Transform& camera_to_room, const Intrinsics& intrinsics, const Point3& room_point);
bool point_in_polygon(const Point2& point, const std::vector<Point2>& polygon);
std::optional<double> ray_ignore_plane_intersection(
    const IgnorePlaneConfig& plane, const Point3& origin, const Point3& direction);

class OccupancyPipeline {
 public:
  explicit OccupancyPipeline(SensorConfig config, bool retain_foreground_points = false);
  std::vector<ZoneState> process(const DepthFrame& frame);
  void set_tracking_enabled(bool enabled);
  void set_ignore_planes(std::vector<IgnorePlaneConfig> planes);
  void reset_tracking();
  std::size_t frames_seen() const { return frames_seen_; }
  const std::vector<Point3>& last_foreground_points() const { return last_foreground_points_; }
  const std::vector<TrackState>& last_tracks() const { return last_tracks_; }
  const std::vector<IgnorePlaneState>& last_ignore_plane_states() const { return last_ignore_plane_states_; }
  const std::vector<Point3>& last_ignored_points() const { return last_ignored_points_; }

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

  struct IgnoreRay {
    double depth_m{std::numeric_limits<double>::infinity()};
    std::size_t plane_index{std::numeric_limits<std::size_t>::max()};
  };

  SensorConfig config_;
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
  std::vector<IgnoreRay> ignore_rays_;
  Intrinsics ignore_intrinsics_;
  std::vector<IgnorePlaneState> last_ignore_plane_states_;
  std::vector<Point3> last_ignored_points_;
  std::uint64_t next_track_id_{1};

  void rebuild_ignore_rays(const DepthFrame& frame);
};

}  // namespace specter
