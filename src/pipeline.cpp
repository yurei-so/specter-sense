#include "specter_sense/pipeline.hpp"
#include "specter_sense/config.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <tuple>

namespace specter {
namespace {

struct ClusterObservation {
  Point3 centroid;
  Point3 bounds;
  std::size_t points{};
  std::string classification{"unknown"};
  double classification_confidence{};
  std::string posture{"unknown"};
  double posture_confidence{};
  std::vector<std::string> zones;
};

using VoxelKey = std::tuple<int, int, int>;

struct Voxel {
  std::size_t count{};
  Point3 sum{};
  Point3 minimum{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity()};
  Point3 maximum{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity()};
};

double distance(Point3 a, Point3 b) {
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                   (a.z - b.z) * (a.z - b.z));
}

Point3 subtract(Point3 left, Point3 right) {
  return {left.x - right.x, left.y - right.y, left.z - right.z};
}

double dot(Point3 left, Point3 right) {
  return left.x * right.x + left.y * right.y + left.z * right.z;
}

Point3 cross(Point3 left, Point3 right) {
  return {left.y * right.z - left.z * right.y,
          left.z * right.x - left.x * right.z,
          left.x * right.y - left.y * right.x};
}

std::pair<std::string, double> classify(const Point3& bounds) {
  const double footprint = std::max(bounds.x, bounds.y);
  if (bounds.z >= 1.05 && bounds.z <= 2.35 && footprint >= 0.20 && footprint <= 1.25) {
    const double height_score = 1.0 - std::min(1.0, std::abs(bounds.z - 1.7) / 0.8);
    return {"likely_human", 0.55 + 0.4 * height_score};
  }
  if (bounds.z >= 0.15 && bounds.z <= 0.90 && footprint >= 0.30 && footprint <= 1.60)
    return {"likely_animal", 0.60};
  if (bounds.z > 2.5 || footprint > 2.0) return {"likely_object", 0.70};
  return {"unknown", 0.25};
}

std::pair<std::string, double> infer_posture(const Point3& bounds, double minimum_z) {
  const double footprint = std::max(bounds.x, bounds.y);
  if (bounds.z >= 1.25 && bounds.z / std::max(0.15, footprint) >= 1.15)
    return {"standing", 0.82};
  if (bounds.z <= 0.70 && footprint >= 0.85 && footprint / std::max(0.15, bounds.z) >= 1.35)
    return {"lying", 0.78};
  if (bounds.z >= 0.65 && bounds.z < 1.25) {
    if (minimum_z <= 0.15 && footprint < 0.85) return {"crouching", 0.55};
    return {"sitting", 0.52};
  }
  return {"unknown", 0.20};
}

std::vector<ClusterObservation> cluster_points(
    const std::vector<Point3>& points, const AppConfig& config) {
  std::map<VoxelKey, Voxel> voxels;
  const double size = config.tracking.voxel_size_m;
  for (const auto& point : points) {
    const VoxelKey key{static_cast<int>(std::floor(point.x / size)),
                       static_cast<int>(std::floor(point.y / size)),
                       static_cast<int>(std::floor(point.z / size))};
    auto& voxel = voxels[key];
    ++voxel.count;
    voxel.sum.x += point.x;
    voxel.sum.y += point.y;
    voxel.sum.z += point.z;
    voxel.minimum.x = std::min(voxel.minimum.x, point.x);
    voxel.minimum.y = std::min(voxel.minimum.y, point.y);
    voxel.minimum.z = std::min(voxel.minimum.z, point.z);
    voxel.maximum.x = std::max(voxel.maximum.x, point.x);
    voxel.maximum.y = std::max(voxel.maximum.y, point.y);
    voxel.maximum.z = std::max(voxel.maximum.z, point.z);
  }

  std::set<VoxelKey> visited;
  std::vector<ClusterObservation> result;
  for (const auto& [start, ignored] : voxels) {
    (void)ignored;
    if (visited.contains(start)) continue;
    std::queue<VoxelKey> pending;
    pending.push(start);
    visited.insert(start);
    std::size_t count{};
    Point3 sum{};
    Point3 minimum{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                   std::numeric_limits<double>::infinity()};
    Point3 maximum{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
                   -std::numeric_limits<double>::infinity()};
    while (!pending.empty()) {
      const auto key = pending.front();
      pending.pop();
      const auto& voxel = voxels.at(key);
      count += voxel.count;
      sum.x += voxel.sum.x;
      sum.y += voxel.sum.y;
      sum.z += voxel.sum.z;
      minimum.x = std::min(minimum.x, voxel.minimum.x);
      minimum.y = std::min(minimum.y, voxel.minimum.y);
      minimum.z = std::min(minimum.z, voxel.minimum.z);
      maximum.x = std::max(maximum.x, voxel.maximum.x);
      maximum.y = std::max(maximum.y, voxel.maximum.y);
      maximum.z = std::max(maximum.z, voxel.maximum.z);
      const auto [x, y, z] = key;
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) continue;
            const VoxelKey neighbor{x + dx, y + dy, z + dz};
            if (voxels.contains(neighbor) && visited.insert(neighbor).second) pending.push(neighbor);
          }
    }
    if (count < config.tracking.min_cluster_points) continue;
    ClusterObservation observation;
    const double divisor = static_cast<double>(count);
    observation.centroid = {sum.x / divisor, sum.y / divisor, sum.z / divisor};
    observation.bounds = {maximum.x - minimum.x, maximum.y - minimum.y, maximum.z - minimum.z};
    observation.points = count;
    std::tie(observation.classification, observation.classification_confidence) = classify(observation.bounds);
    if (observation.classification == "likely_human")
      std::tie(observation.posture, observation.posture_confidence) = infer_posture(observation.bounds, minimum.z);
    for (const auto& zone : config.zones)
      if (observation.centroid.z >= zone.min_height_m && observation.centroid.z <= zone.max_height_m &&
          point_in_polygon({observation.centroid.x, observation.centroid.y}, zone.floor_polygon))
        observation.zones.push_back(zone.name);
    result.push_back(std::move(observation));
  }
  std::sort(result.begin(), result.end(), [](const auto& left, const auto& right) {
    return std::tie(left.centroid.x, left.centroid.y, left.centroid.z) <
           std::tie(right.centroid.x, right.centroid.y, right.centroid.z);
  });
  return result;
}

}  // namespace

Point3 transform_point(const Transform& transform, const Point3& point) {
  const auto& m = transform.matrix;
  return {
      m[0] * point.x + m[1] * point.y + m[2] * point.z + m[3],
      m[4] * point.x + m[5] * point.y + m[6] * point.z + m[7],
      m[8] * point.x + m[9] * point.y + m[10] * point.z + m[11]};
}

Transform invert_transform(const Transform& transform) {
  const auto& m = transform.matrix;
  const double determinant =
      m[0] * (m[5] * m[10] - m[6] * m[9]) -
      m[1] * (m[4] * m[10] - m[6] * m[8]) +
      m[2] * (m[4] * m[9] - m[5] * m[8]);
  if (!std::isfinite(determinant) || std::abs(determinant) < 1e-12)
    throw std::runtime_error("cannot invert singular camera transform");
  const double inverse = 1.0 / determinant;
  Transform result;
  auto& r = result.matrix;
  r[0] = (m[5] * m[10] - m[6] * m[9]) * inverse;
  r[1] = (m[2] * m[9] - m[1] * m[10]) * inverse;
  r[2] = (m[1] * m[6] - m[2] * m[5]) * inverse;
  r[4] = (m[6] * m[8] - m[4] * m[10]) * inverse;
  r[5] = (m[0] * m[10] - m[2] * m[8]) * inverse;
  r[6] = (m[2] * m[4] - m[0] * m[6]) * inverse;
  r[8] = (m[4] * m[9] - m[5] * m[8]) * inverse;
  r[9] = (m[1] * m[8] - m[0] * m[9]) * inverse;
  r[10] = (m[0] * m[5] - m[1] * m[4]) * inverse;
  r[3] = -(r[0] * m[3] + r[1] * m[7] + r[2] * m[11]);
  r[7] = -(r[4] * m[3] + r[5] * m[7] + r[6] * m[11]);
  r[11] = -(r[8] * m[3] + r[9] * m[7] + r[10] * m[11]);
  r[12] = 0; r[13] = 0; r[14] = 0; r[15] = 1;
  return result;
}

Point3 deproject_depth(const Intrinsics& intrinsics, double pixel_x, double pixel_y, double depth_m) {
  if (!(intrinsics.fx > 0 && intrinsics.fy > 0) || !(depth_m > 0) || !std::isfinite(depth_m))
    throw std::runtime_error("cannot deproject invalid depth or intrinsics");
  return {
      (pixel_x - intrinsics.cx) * depth_m / intrinsics.fx,
      (pixel_y - intrinsics.cy) * depth_m / intrinsics.fy,
      depth_m};
}

std::optional<Point2> project_room_to_depth(
    const Transform& camera_to_room, const Intrinsics& intrinsics, const Point3& room_point) {
  if (!(intrinsics.fx > 0 && intrinsics.fy > 0)) throw std::runtime_error("invalid depth intrinsics");
  const Point3 camera = transform_point(invert_transform(camera_to_room), room_point);
  if (!(camera.z > 0) || !std::isfinite(camera.z)) return std::nullopt;
  return Point2{intrinsics.fx * camera.x / camera.z + intrinsics.cx,
                intrinsics.fy * camera.y / camera.z + intrinsics.cy};
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

std::optional<double> ray_ignore_plane_intersection(
    const IgnorePlaneConfig& plane, const Point3& origin, const Point3& direction) {
  if (!plane.enabled) return std::nullopt;
  const Point3 u = subtract(plane.corners_m[1], plane.corners_m[0]);
  const Point3 v = subtract(plane.corners_m[3], plane.corners_m[0]);
  const Point3 normal = cross(u, v);
  const double denominator = dot(normal, direction);
  if (std::abs(denominator) < 1e-9) return std::nullopt;
  const double parameter = dot(normal, subtract(plane.corners_m[0], origin)) / denominator;
  if (!(parameter > 0) || !std::isfinite(parameter)) return std::nullopt;
  const Point3 intersection{origin.x + direction.x * parameter,
                            origin.y + direction.y * parameter,
                            origin.z + direction.z * parameter};
  const Point3 relative = subtract(intersection, plane.corners_m[0]);
  const double u_squared = dot(u, u), v_squared = dot(v, v);
  if (!(u_squared > 0 && v_squared > 0)) return std::nullopt;
  const double along_u = dot(relative, u) / u_squared;
  const double along_v = dot(relative, v) / v_squared;
  const double margin_u = plane.margin_m / std::sqrt(u_squared);
  const double margin_v = plane.margin_m / std::sqrt(v_squared);
  if (along_u < -margin_u || along_u > 1.0 + margin_u ||
      along_v < -margin_v || along_v > 1.0 + margin_v) return std::nullopt;
  return parameter;
}

OccupancyPipeline::OccupancyPipeline(AppConfig config, bool retain_foreground_points)
    : config_(std::move(config)), runtime_(config_.zones.size()),
      retain_foreground_points_(retain_foreground_points), tracking_enabled_(config_.tracking.enabled) {
  validate_config(config_);
}

void OccupancyPipeline::rebuild_ignore_rays(const DepthFrame& frame) {
  ignore_rays_.assign(frame.depth_mm.size(), {});
  ignore_intrinsics_ = frame.intrinsics;
  const Point3 origin = transform_point(config_.camera_to_room, {0, 0, 0});
  for (std::size_t index = 0; index < frame.depth_mm.size(); ++index) {
    const double u = static_cast<double>(index % frame.width);
    const double v = static_cast<double>(index / frame.width);
    const Point3 camera = deproject_depth(frame.intrinsics, u, v, 1.0);
    const Point3 room = transform_point(config_.camera_to_room, camera);
    const Point3 direction = subtract(room, origin);
    for (std::size_t plane_index = 0; plane_index < config_.ignore_planes.size(); ++plane_index) {
      const auto hit = ray_ignore_plane_intersection(config_.ignore_planes[plane_index], origin, direction);
      if (hit && *hit < ignore_rays_[index].depth_m)
        ignore_rays_[index] = {*hit, plane_index};
    }
  }
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
  if (ignore_rays_.size() != frame.depth_mm.size() ||
      ignore_intrinsics_.fx != frame.intrinsics.fx || ignore_intrinsics_.fy != frame.intrinsics.fy ||
      ignore_intrinsics_.cx != frame.intrinsics.cx || ignore_intrinsics_.cy != frame.intrinsics.cy)
    rebuild_ignore_rays(frame);

  struct Evidence {
    std::size_t count{};
    Point3 sum{};
    double near_m{std::numeric_limits<double>::infinity()};
    double far_m{};
  };
  std::vector<Evidence> evidence(config_.zones.size());
  last_foreground_points_.clear();
  last_ignored_points_.clear();
  last_ignore_plane_states_.clear();
  for (const auto& plane : config_.ignore_planes)
    last_ignore_plane_states_.push_back(
        {plane.name, plane.enabled, 0, 0, plane.noise_threshold_points});
  const auto& p = config_.processing;

  std::vector<std::size_t> plane_matches(config_.ignore_planes.size());
  if (std::any_of(config_.ignore_planes.begin(), config_.ignore_planes.end(),
                  [](const auto& plane) { return plane.noise_threshold_points.has_value(); }))
    for (std::size_t index = 0; index < frame.depth_mm.size(); ++index) {
      const double depth_m = static_cast<double>(frame.depth_mm[index]) / 1000.0;
      if (!std::isfinite(depth_m) || depth_m < p.min_depth_m || depth_m > p.max_depth_m) continue;
      const auto& ignore = ignore_rays_[index];
      if (ignore.plane_index < config_.ignore_planes.size() &&
          depth_m >= ignore.depth_m - config_.ignore_planes[ignore.plane_index].surface_tolerance_m)
        ++plane_matches[ignore.plane_index];
    }

  for (std::size_t index = 0; index < frame.depth_mm.size(); ++index) {
    const double depth_m = static_cast<double>(frame.depth_mm[index]) / 1000.0;
    if (!std::isfinite(depth_m) || depth_m < p.min_depth_m || depth_m > p.max_depth_m) continue;
    const auto& ignore = ignore_rays_[index];
    if (ignore.plane_index < config_.ignore_planes.size()) {
      const auto& plane = config_.ignore_planes[ignore.plane_index];
      const bool reject_plane = !plane.noise_threshold_points ||
                                plane_matches[ignore.plane_index] <= *plane.noise_threshold_points;
      if (depth_m >= ignore.depth_m - plane.surface_tolerance_m) {
        ++last_ignore_plane_states_[ignore.plane_index].matched_points;
        if (reject_plane) {
          ++last_ignore_plane_states_[ignore.plane_index].rejected_points;
          if (retain_foreground_points_) {
            const double u = static_cast<double>(index % frame.width);
            const double v = static_cast<double>(index / frame.width);
            last_ignored_points_.push_back(transform_point(
                config_.camera_to_room, deproject_depth(frame.intrinsics, u, v, depth_m)));
          }
          continue;
        }
      }
    }
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
    const Point3 camera = deproject_depth(frame.intrinsics, u, v, depth_m);
    const Point3 room = transform_point(config_.camera_to_room, camera);
    if (retain_foreground_points_ || tracking_enabled_) last_foreground_points_.push_back(room);
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

  last_tracks_.clear();
  if (!tracking_enabled_) return states;
  const auto observations = cluster_points(last_foreground_points_, config_);
  struct Candidate { std::size_t track{}; std::size_t observation{}; double cost{}; };
  std::vector<Candidate> candidates;
  for (std::size_t track_index = 0; track_index < tracks_.size(); ++track_index) {
    const auto& track = tracks_[track_index];
    const double dt = std::clamp(
        std::chrono::duration<double>(frame.observed_at - track.observed_at).count(), 1.0 / 120.0, 0.5);
    const Point3 predicted{track.centroid.x + track.velocity.x * dt,
                           track.centroid.y + track.velocity.y * dt,
                           track.centroid.z + track.velocity.z * dt};
    for (std::size_t observation_index = 0; observation_index < observations.size(); ++observation_index) {
      const auto& observation = observations[observation_index];
      const double position_cost = distance(predicted, observation.centroid);
      if (position_cost > config_.tracking.association_max_distance_m) continue;
      const double shape_cost = distance(track.bounds, observation.bounds);
      if (shape_cost > 1.5) continue;
      candidates.push_back({track_index, observation_index, position_cost + 0.25 * shape_cost});
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto& left, const auto& right) {
    return left.cost < right.cost;
  });
  std::vector<bool> track_matched(tracks_.size());
  std::vector<bool> observation_matched(observations.size());
  for (const auto& candidate : candidates) {
    if (track_matched[candidate.track] || observation_matched[candidate.observation]) continue;
    auto& track = tracks_[candidate.track];
    const auto& observation = observations[candidate.observation];
    const double dt = std::clamp(
        std::chrono::duration<double>(frame.observed_at - track.observed_at).count(), 1.0 / 120.0, 0.5);
    const Point3 measured_velocity{(observation.centroid.x - track.centroid.x) / dt,
                                   (observation.centroid.y - track.centroid.y) / dt,
                                   (observation.centroid.z - track.centroid.z) / dt};
    track.velocity = {0.65 * track.velocity.x + 0.35 * measured_velocity.x,
                      0.65 * track.velocity.y + 0.35 * measured_velocity.y,
                      0.65 * track.velocity.z + 0.35 * measured_velocity.z};
    track.centroid = {0.25 * track.centroid.x + 0.75 * observation.centroid.x,
                      0.25 * track.centroid.y + 0.75 * observation.centroid.y,
                      0.25 * track.centroid.z + 0.75 * observation.centroid.z};
    track.bounds = {0.35 * track.bounds.x + 0.65 * observation.bounds.x,
                    0.35 * track.bounds.y + 0.65 * observation.bounds.y,
                    0.35 * track.bounds.z + 0.65 * observation.bounds.z};
    track.foreground_points = observation.points;
    track.classification = observation.classification;
    track.classification_confidence = observation.classification_confidence;
    if (observation.posture == track.posture_candidate) ++track.posture_candidate_frames;
    else {
      track.posture_candidate = observation.posture;
      track.posture_candidate_frames = 1;
    }
    if (track.posture_candidate_frames >= config_.tracking.confirmation_frames) {
      track.posture = observation.posture;
      track.posture_confidence = observation.posture_confidence;
    }
    track.zones = observation.zones;
    track.observed_at = frame.observed_at;
    ++track.hits;
    track.missed = 0;
    track_matched[candidate.track] = true;
    observation_matched[candidate.observation] = true;
  }
  for (std::size_t i = 0; i < tracks_.size(); ++i) if (!track_matched[i]) ++tracks_[i].missed;
  for (std::size_t i = 0; i < observations.size(); ++i) {
    if (observation_matched[i]) continue;
    const auto& observation = observations[i];
    TrackRuntime track;
    track.id = next_track_id_++;
    track.centroid = observation.centroid;
    track.bounds = observation.bounds;
    track.foreground_points = observation.points;
    track.classification = observation.classification;
    track.classification_confidence = observation.classification_confidence;
    track.posture_candidate = observation.posture;
    track.posture_candidate_frames = 1;
    track.observed_at = frame.observed_at;
    track.zones = observation.zones;
    track.hits = 1;
    tracks_.push_back(std::move(track));
  }
  std::erase_if(tracks_, [&](const auto& track) { return track.missed > config_.tracking.max_missed_frames; });
  for (const auto& track : tracks_) {
    if (track.hits < config_.tracking.confirmation_frames) continue;
    TrackState state;
    state.id = "track-" + std::to_string(track.id);
    state.tracking_state = track.missed == 0 ? "confirmed" : "coasting";
    state.classification = track.classification;
    state.classification_confidence = track.classification_confidence;
    state.posture = track.posture;
    state.posture_confidence = track.posture_confidence;
    state.centroid_m = track.centroid;
    state.velocity_mps = track.velocity;
    state.bounds_m = track.bounds;
    state.foreground_points = track.foreground_points;
    state.zones = track.zones;
    state.occluded = track.missed > 0;
    state.observed_at = track.observed_at;
    last_tracks_.push_back(std::move(state));
  }
  return states;
}

void OccupancyPipeline::reset_tracking() {
  tracks_.clear();
  last_tracks_.clear();
}

void OccupancyPipeline::set_tracking_enabled(bool enabled) {
  if (tracking_enabled_ == enabled) return;
  tracking_enabled_ = enabled;
  reset_tracking();
}

void OccupancyPipeline::set_ignore_planes(std::vector<IgnorePlaneConfig> planes) {
  auto candidate = config_;
  candidate.ignore_planes = std::move(planes);
  validate_config(candidate);
  config_.ignore_planes = std::move(candidate.ignore_planes);
  ignore_rays_.clear();
  last_ignore_plane_states_.clear();
  last_ignored_points_.clear();
  reset_tracking();
}

}  // namespace specter
