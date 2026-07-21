#include "specter_sense/config.hpp"

#include <boost/json.hpp>

#include <algorithm>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>

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

Point3 point3(const boost::json::value& value) {
  const auto& object = value.as_object();
  return {number(object, "x"), number(object, "y"), number(object, "z")};
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

double length(Point3 value) { return std::sqrt(dot(value, value)); }

}  // namespace

SensorConfig parse_sensor(const boost::json::object& root) {
  SensorConfig config;
  config.name = root.at("name").as_string().c_str();
  config.source = root.at("source").as_string().c_str();
  if (const auto* serial = root.if_contains("serial")) config.serial = serial->as_string().c_str();
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

  if (const auto* value = root.if_contains("ignore_planes")) {
    for (const auto& plane_value : value->as_array()) {
      const auto& object = plane_value.as_object();
      IgnorePlaneConfig plane;
      plane.name = object.at("name").as_string().c_str();
      plane.enabled = bool_value(object, "enabled");
      plane.margin_m = number(object, "margin_m");
      plane.surface_tolerance_m = object.if_contains("surface_tolerance_m")
          ? number(object, "surface_tolerance_m") : 0.03;
      if (const auto* threshold = object.if_contains("noise_threshold_points"))
        plane.noise_threshold_points = threshold->to_number<std::size_t>();
      const auto& corners = object.at("corners_m").as_array();
      if (corners.size() != 4) throw std::runtime_error("ignore plane " + plane.name + " must contain 4 corners");
      for (std::size_t i = 0; i < 4; ++i) plane.corners_m[i] = point3(corners[i]);
      config.ignore_planes.push_back(std::move(plane));
    }
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
  return config;
}

void validate_sensor(const SensorConfig& config) {
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
  std::set<std::string> plane_names;
  for (const auto& plane : config.ignore_planes) {
    if (plane.name.empty()) throw std::runtime_error("ignore plane name cannot be empty");
    if (!plane_names.insert(plane.name).second) throw std::runtime_error("duplicate ignore plane name: " + plane.name);
    if (!(plane.margin_m >= 0 && plane.margin_m <= 1.0) || !std::isfinite(plane.margin_m))
      throw std::runtime_error("ignore plane " + plane.name + " margin_m must be between 0 and 1");
    if (!(plane.surface_tolerance_m >= 0 && plane.surface_tolerance_m <= 0.2) ||
        !std::isfinite(plane.surface_tolerance_m))
      throw std::runtime_error("ignore plane " + plane.name + " surface_tolerance_m must be between 0 and 0.2");
    if (plane.noise_threshold_points && *plane.noise_threshold_points > 1000000)
      throw std::runtime_error("ignore plane " + plane.name + " noise_threshold_points must not exceed 1000000");
    for (const auto& corner : plane.corners_m)
      if (!std::isfinite(corner.x) || !std::isfinite(corner.y) || !std::isfinite(corner.z) ||
          std::abs(corner.x) > 50 || std::abs(corner.y) > 50 || std::abs(corner.z) > 20)
        throw std::runtime_error("ignore plane " + plane.name + " has invalid or unreasonable corners");
    const Point3 u = subtract(plane.corners_m[1], plane.corners_m[0]);
    const Point3 v = subtract(plane.corners_m[3], plane.corners_m[0]);
    const double width = length(u), height = length(v);
    if (width < 0.05 || height < 0.05 || length(cross(u, v)) < 0.0025)
      throw std::runtime_error("ignore plane " + plane.name + " has degenerate area");
    if (std::abs(dot(u, v) / (width * height)) > 0.02)
      throw std::runtime_error("ignore plane " + plane.name + " adjacent edges must be perpendicular");
    const Point3 expected = {plane.corners_m[1].x + plane.corners_m[3].x - plane.corners_m[0].x,
                             plane.corners_m[1].y + plane.corners_m[3].y - plane.corners_m[0].y,
                             plane.corners_m[1].z + plane.corners_m[3].z - plane.corners_m[0].z};
    if (length(subtract(expected, plane.corners_m[2])) > 0.01)
      throw std::runtime_error("ignore plane " + plane.name + " corners must form an ordered rectangle");
    const Point3 camera_origin{config.camera_to_room.matrix[3], config.camera_to_room.matrix[7],
                               config.camera_to_room.matrix[11]};
    const Point3 from_corner = subtract(camera_origin, plane.corners_m[0]);
    const Point3 normal = cross(u, v);
    const double plane_distance = std::abs(dot(normal, from_corner)) / length(normal);
    const double along_u = dot(from_corner, u) / dot(u, u);
    const double along_v = dot(from_corner, v) / dot(v, v);
    if (plane_distance < 0.01 && along_u >= 0 && along_u <= 1 && along_v >= 0 && along_v <= 1)
      throw std::runtime_error("ignore plane " + plane.name + " intersects the calibrated camera origin");
  }
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

AppConfig load_config(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open config: " + path.string());
  const std::string text((std::istreambuf_iterator<char>(input)), {});
  const auto root = boost::json::parse(text).as_object();
  AppConfig config;
  for (const auto& value : root.at("sensors").as_array()) config.sensors.push_back(parse_sensor(value.as_object()));
  validate_config(config);
  return config;
}

void validate_config(const AppConfig& config) {
  if (config.sensors.empty()) throw std::runtime_error("at least one sensor is required");
  std::set<std::string> names;
  std::set<std::string> serials;
  for (const auto& sensor : config.sensors) {
    if (sensor.name.empty()) throw std::runtime_error("sensor name cannot be empty");
    if (!names.insert(sensor.name).second) throw std::runtime_error("duplicate sensor name: " + sensor.name);
    if (sensor.source != "synthetic" && sensor.source != "kinect" &&
        sensor.source != "kinect-v1" && sensor.source != "kinect-v2")
      throw std::runtime_error("sensor " + sensor.name + " has unknown source: " + sensor.source);
    if (sensor.serial && sensor.serial->empty()) throw std::runtime_error("sensor " + sensor.name + " serial cannot be empty");
    if (sensor.source == "synthetic" && sensor.serial)
      throw std::runtime_error("sensor " + sensor.name + " serial is only valid for a Kinect source");
    const auto model = sensor.source == "kinect" ? std::string_view("kinect-v2") : std::string_view(sensor.source);
    const auto same_model_count = std::count_if(config.sensors.begin(), config.sensors.end(),
        [&](const auto& candidate) {
          const auto candidate_model = candidate.source == "kinect"
              ? std::string_view("kinect-v2") : std::string_view(candidate.source);
          return candidate_model == model;
        });
    if (sensor.source != "synthetic" && same_model_count > 1 && !sensor.serial)
      throw std::runtime_error("sensor " + sensor.name + " requires a serial when multiple sensors of its model are configured");
    if (sensor.serial && !serials.insert(*sensor.serial).second)
      throw std::runtime_error("duplicate sensor serial: " + *sensor.serial);
    validate_sensor(sensor);
  }
}

boost::json::object serialize_sensor(const SensorConfig& config) {
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
  boost::json::array ignore_planes;
  for (const auto& plane : config.ignore_planes) {
    boost::json::array corners;
    for (const auto& corner : plane.corners_m)
      corners.push_back({{"x", corner.x}, {"y", corner.y}, {"z", corner.z}});
    boost::json::object serialized{
        {"name", plane.name},
        {"enabled", plane.enabled},
        {"corners_m", std::move(corners)},
        {"margin_m", plane.margin_m},
        {"surface_tolerance_m", plane.surface_tolerance_m}};
    if (plane.noise_threshold_points)
      serialized["noise_threshold_points"] = *plane.noise_threshold_points;
    ignore_planes.push_back(std::move(serialized));
  }
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
  boost::json::object result{
      {"name", config.name},
      {"source", config.source},
      {"camera_to_room", std::move(transform)},
      {"processing", std::move(processing)},
      {"tracking", std::move(tracking)},
      {"ignore_planes", std::move(ignore_planes)},
      {"zones", std::move(zones)}};
  if (config.serial) result["serial"] = *config.serial;
  return result;
}

std::string serialize_config(const AppConfig& config) {
  validate_config(config);
  boost::json::array sensors;
  for (const auto& sensor : config.sensors) sensors.push_back(serialize_sensor(sensor));
  return boost::json::serialize(boost::json::object{{"sensors", std::move(sensors)}}) + "\n";
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
