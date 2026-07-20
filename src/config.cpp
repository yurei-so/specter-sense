#include "specter_sense/config.hpp"

#include <boost/json.hpp>

#include <fstream>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

namespace specter {
namespace {

double number(const boost::json::object& object, const char* key) {
  const auto& value = object.at(key);
  if (value.is_double()) return value.as_double();
  if (value.is_int64()) return static_cast<double>(value.as_int64());
  if (value.is_uint64()) return static_cast<double>(value.as_uint64());
  throw std::runtime_error(std::string(key) + " must be a number");
}

std::size_t size_value(const boost::json::object& object, const char* key) {
  const auto value = number(object, key);
  if (value < 0 || value != static_cast<double>(static_cast<std::size_t>(value))) {
    throw std::runtime_error(std::string(key) + " must be a non-negative integer");
  }
  return static_cast<std::size_t>(value);
}

bool bool_value(const boost::json::object& object, const char* key) {
  const auto& value = object.at(key);
  if (!value.is_bool()) throw std::runtime_error(std::string(key) + " must be a boolean");
  return value.as_bool();
}

double orientation(Point2 a, Point2 b, Point2 c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool proper_intersection(Point2 a, Point2 b, Point2 c, Point2 d) {
  const double ab_c = orientation(a, b, c);
  const double ab_d = orientation(a, b, d);
  const double cd_a = orientation(c, d, a);
  const double cd_b = orientation(c, d, b);
  return ((ab_c > 0 && ab_d < 0) || (ab_c < 0 && ab_d > 0)) &&
         ((cd_a > 0 && cd_b < 0) || (cd_a < 0 && cd_b > 0));
}

}  // namespace

AppConfig load_config(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open config: " + path.string());
  const std::string text((std::istreambuf_iterator<char>(input)), {});
  const auto root = boost::json::parse(text).as_object();

  AppConfig config;
  const auto& transform = root.at("camera_to_room").as_array();
  if (transform.size() != 16) throw std::runtime_error("camera_to_room must contain 16 numbers");
  for (std::size_t i = 0; i < 16; ++i) {
    const auto& value = transform[i];
    config.camera_to_room.matrix[i] = value.is_double()
        ? value.as_double()
        : value.is_int64() ? static_cast<double>(value.as_int64())
                           : static_cast<double>(value.as_uint64());
  }

  const auto& processing = root.at("processing").as_object();
  config.processing.min_depth_m = number(processing, "min_depth_m");
  config.processing.max_depth_m = number(processing, "max_depth_m");
  config.processing.foreground_delta_m = number(processing, "foreground_delta_m");
  config.processing.background_alpha = number(processing, "background_alpha");
  config.processing.warmup_frames = size_value(processing, "warmup_frames");

  if (const auto* value = root.if_contains("tracking")) {
    const auto& tracking = value->as_object();
    config.tracking.enabled = bool_value(tracking, "enabled");
    config.tracking.voxel_size_m = number(tracking, "voxel_size_m");
    config.tracking.min_cluster_points = size_value(tracking, "min_cluster_points");
    config.tracking.association_max_distance_m = number(tracking, "association_max_distance_m");
    config.tracking.confirmation_frames = size_value(tracking, "confirmation_frames");
    config.tracking.max_missed_frames = size_value(tracking, "max_missed_frames");
  }

  for (const auto& zone_value : root.at("zones").as_array()) {
    const auto& object = zone_value.as_object();
    ZoneConfig zone;
    zone.name = object.at("name").as_string().c_str();
    zone.min_height_m = number(object, "min_height_m");
    zone.max_height_m = number(object, "max_height_m");
    zone.enter_points = size_value(object, "enter_points");
    zone.exit_points = size_value(object, "exit_points");
    zone.enter_after = std::chrono::milliseconds(size_value(object, "enter_after_ms"));
    zone.exit_after = std::chrono::milliseconds(size_value(object, "exit_after_ms"));
    for (const auto& point_value : object.at("floor_polygon").as_array()) {
      const auto& pair = point_value.as_array();
      if (pair.size() != 2) throw std::runtime_error("floor polygon points must be [x, y]");
      auto get = [](const boost::json::value& v) {
        if (v.is_double()) return v.as_double();
        if (v.is_int64()) return static_cast<double>(v.as_int64());
        return static_cast<double>(v.as_uint64());
      };
      zone.floor_polygon.push_back({get(pair[0]), get(pair[1])});
    }
    config.zones.push_back(std::move(zone));
  }
  validate_config(config);
  return config;
}

void validate_config(const AppConfig& config) {
  const auto& p = config.processing;
  for (const auto value : config.camera_to_room.matrix)
    if (!std::isfinite(value)) throw std::runtime_error("camera_to_room contains a non-finite value");
  const auto& m = config.camera_to_room.matrix;
  if (std::abs(m[12]) > 1e-9 || std::abs(m[13]) > 1e-9 || std::abs(m[14]) > 1e-9 || std::abs(m[15] - 1.0) > 1e-9)
    throw std::runtime_error("camera_to_room must be an affine 4x4 transform with bottom row [0,0,0,1]");
  if (!(p.min_depth_m > 0 && p.max_depth_m > p.min_depth_m))
    throw std::runtime_error("processing depth range is invalid");
  if (!(p.foreground_delta_m > 0)) throw std::runtime_error("foreground_delta_m must be positive");
  if (!(p.background_alpha >= 0 && p.background_alpha <= 1))
    throw std::runtime_error("background_alpha must be between 0 and 1");
  const auto& tracking = config.tracking;
  if (!(tracking.voxel_size_m >= 0.02 && tracking.voxel_size_m <= 0.25))
    throw std::runtime_error("tracking voxel_size_m must be between 0.02 and 0.25");
  if (tracking.min_cluster_points == 0)
    throw std::runtime_error("tracking min_cluster_points must be positive");
  if (!(tracking.association_max_distance_m > 0 && tracking.association_max_distance_m <= 5))
    throw std::runtime_error("tracking association_max_distance_m must be between 0 and 5");
  if (tracking.confirmation_frames == 0)
    throw std::runtime_error("tracking confirmation_frames must be positive");
  if (tracking.max_missed_frames == 0)
    throw std::runtime_error("tracking max_missed_frames must be positive");
  if (config.zones.empty()) throw std::runtime_error("at least one zone is required");
  std::set<std::string> names;
  for (const auto& zone : config.zones) {
    if (zone.name.empty()) throw std::runtime_error("zone name cannot be empty");
    if (!names.insert(zone.name).second) throw std::runtime_error("duplicate zone name: " + zone.name);
    if (zone.floor_polygon.size() < 3) throw std::runtime_error("zone " + zone.name + " needs 3 polygon points");
    if (!(zone.max_height_m > zone.min_height_m)) throw std::runtime_error("zone " + zone.name + " height range is invalid");
    if (!std::isfinite(zone.min_height_m) || !std::isfinite(zone.max_height_m) ||
        std::abs(zone.min_height_m) > 20 || std::abs(zone.max_height_m) > 20)
      throw std::runtime_error("zone " + zone.name + " has unreasonable height bounds");
    if (zone.enter_points < zone.exit_points) throw std::runtime_error("zone " + zone.name + " enter_points must be >= exit_points");
    if (zone.exit_points == 0) throw std::runtime_error("zone " + zone.name + " exit_points must be positive");
    double twice_area{};
    for (std::size_t i = 0; i < zone.floor_polygon.size(); ++i) {
      const auto& point = zone.floor_polygon[i];
      const auto& next = zone.floor_polygon[(i + 1) % zone.floor_polygon.size()];
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || std::abs(point.x) > 50 || std::abs(point.y) > 50)
        throw std::runtime_error("zone " + zone.name + " has coordinates outside +/-50 metres");
      if (std::hypot(point.x - next.x, point.y - next.y) < 0.001)
        throw std::runtime_error("zone " + zone.name + " has duplicate adjacent vertices");
      twice_area += point.x * next.y - next.x * point.y;
      for (std::size_t j = i + 1; j < zone.floor_polygon.size(); ++j) {
        const auto i_next = (i + 1) % zone.floor_polygon.size();
        const auto j_next = (j + 1) % zone.floor_polygon.size();
        if (i == j || i_next == j || j_next == i) continue;
        if (proper_intersection(point, next, zone.floor_polygon[j], zone.floor_polygon[j_next]))
          throw std::runtime_error("zone " + zone.name + " polygon self-intersects");
      }
    }
    if (std::abs(twice_area) < 0.001) throw std::runtime_error("zone " + zone.name + " polygon area is too small");
  }
}

std::string serialize_config(const AppConfig& config) {
  validate_config(config);
  boost::json::array transform;
  for (const auto value : config.camera_to_room.matrix) transform.push_back(value);
  boost::json::object processing{
      {"min_depth_m", config.processing.min_depth_m},
      {"max_depth_m", config.processing.max_depth_m},
      {"foreground_delta_m", config.processing.foreground_delta_m},
      {"background_alpha", config.processing.background_alpha},
      {"warmup_frames", config.processing.warmup_frames}};
  boost::json::object tracking{
      {"enabled", config.tracking.enabled},
      {"voxel_size_m", config.tracking.voxel_size_m},
      {"min_cluster_points", config.tracking.min_cluster_points},
      {"association_max_distance_m", config.tracking.association_max_distance_m},
      {"confirmation_frames", config.tracking.confirmation_frames},
      {"max_missed_frames", config.tracking.max_missed_frames}};
  boost::json::array zones;
  for (const auto& zone : config.zones) {
    boost::json::array polygon;
    for (const auto& point : zone.floor_polygon) polygon.push_back({point.x, point.y});
    zones.push_back({
        {"name", zone.name},
        {"floor_polygon", std::move(polygon)},
        {"min_height_m", zone.min_height_m},
        {"max_height_m", zone.max_height_m},
        {"enter_points", zone.enter_points},
        {"exit_points", zone.exit_points},
        {"enter_after_ms", zone.enter_after.count()},
        {"exit_after_ms", zone.exit_after.count()}});
  }
  return boost::json::serialize(boost::json::object{
      {"camera_to_room", std::move(transform)},
      {"processing", std::move(processing)},
      {"tracking", std::move(tracking)},
      {"zones", std::move(zones)}}) + "\n";
}

void save_config_atomic(const std::filesystem::path& path, const AppConfig& config) {
  if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
  auto temporary = path;
  temporary += ".tmp";
  {
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    if (!output) throw std::runtime_error("cannot write config: " + temporary.string());
    output << serialize_config(config);
    output.flush();
    if (!output) throw std::runtime_error("failed writing config: " + temporary.string());
  }
  std::filesystem::rename(temporary, path);
}

}  // namespace specter
