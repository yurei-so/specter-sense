#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace specter {

struct Point2 {
  double x{};
  double y{};
};

struct Point3 {
  double x{};
  double y{};
  double z{};
};

struct Intrinsics {
  double fx{};
  double fy{};
  double cx{};
  double cy{};
};

struct DepthFrame {
  std::size_t width{};
  std::size_t height{};
  std::vector<float> depth_mm;
  Intrinsics intrinsics;
  std::chrono::system_clock::time_point observed_at;
};

struct Transform {
  std::array<double, 16> matrix{
      1, 0, 0, 0,
      0, 0, 1, 0,
      0, -1, 0, 0,
      0, 0, 0, 1};
};

struct ZoneConfig {
  std::string name;
  std::vector<Point2> floor_polygon;
  double min_height_m{};
  double max_height_m{2.5};
  std::size_t enter_points{100};
  std::size_t exit_points{50};
  std::chrono::milliseconds enter_after{250};
  std::chrono::milliseconds exit_after{1000};
};

struct ProcessingConfig {
  double min_depth_m{0.5};
  double max_depth_m{4.5};
  double foreground_delta_m{0.15};
  double background_alpha{0.002};
  std::size_t warmup_frames{30};
};

struct TrackingConfig {
  bool enabled{true};
  double voxel_size_m{0.06};
  std::size_t min_cluster_points{80};
  double association_max_distance_m{0.75};
  std::size_t confirmation_frames{3};
  std::size_t max_missed_frames{20};
};

struct IgnorePlaneConfig {
  std::string name;
  bool enabled{true};
  std::array<Point3, 4> corners_m;
  double margin_m{0.05};
  double surface_tolerance_m{0.03};
  std::optional<std::size_t> noise_threshold_points;
};

struct SensorConfig {
  std::string name;
  bool enabled{true};
  std::string source{"synthetic"};
  std::optional<std::string> serial;
  Transform camera_to_room;
  ProcessingConfig processing;
  TrackingConfig tracking;
  std::vector<IgnorePlaneConfig> ignore_planes;
  std::vector<ZoneConfig> zones;
};

struct AppConfig {
  std::vector<SensorConfig> sensors;
};

struct ZoneState {
  std::string name;
  bool occupied{};
  double occupancy_score{};
  std::size_t foreground_points{};
  std::optional<Point3> centroid_m;
  std::optional<double> nearest_range_m;
  std::optional<double> farthest_range_m;
  std::chrono::system_clock::time_point observed_at;
};

struct SensorState {
  bool connected{};
  bool reconnecting{};
  std::string status{"starting"};
};

struct TrackState {
  std::string id;
  std::string tracking_state{"tentative"};
  std::string classification{"unknown"};
  double classification_confidence{};
  std::string posture{"unknown"};
  double posture_confidence{};
  Point3 centroid_m;
  Point3 velocity_mps;
  Point3 bounds_m;
  std::size_t foreground_points{};
  std::vector<std::string> zones;
  bool occluded{};
  std::chrono::system_clock::time_point observed_at;
};

struct IgnorePlaneState {
  std::string name;
  bool enabled{};
  std::size_t rejected_points{};
  std::size_t matched_points{};
  std::size_t activity_points{};
  std::optional<std::size_t> noise_threshold_points;
};

struct SensorSnapshot {
  std::optional<std::chrono::system_clock::time_point> last_valid_frame_at;
  SensorState sensor;
  std::vector<ZoneState> zones;
  std::vector<TrackState> tracks;
  std::vector<IgnorePlaneState> ignore_planes;
};

struct Snapshot {
  std::chrono::system_clock::time_point generated_at;
  std::vector<std::pair<std::string, SensorSnapshot>> sensors;
};

}  // namespace specter
