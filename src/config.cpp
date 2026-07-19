#include "specter_sense/config.hpp"

#include <boost/json.hpp>

#include <fstream>
#include <set>
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
  if (!(p.min_depth_m > 0 && p.max_depth_m > p.min_depth_m))
    throw std::runtime_error("processing depth range is invalid");
  if (!(p.foreground_delta_m > 0)) throw std::runtime_error("foreground_delta_m must be positive");
  if (!(p.background_alpha >= 0 && p.background_alpha <= 1))
    throw std::runtime_error("background_alpha must be between 0 and 1");
  if (config.zones.empty()) throw std::runtime_error("at least one zone is required");
  std::set<std::string> names;
  for (const auto& zone : config.zones) {
    if (zone.name.empty()) throw std::runtime_error("zone name cannot be empty");
    if (!names.insert(zone.name).second) throw std::runtime_error("duplicate zone name: " + zone.name);
    if (zone.floor_polygon.size() < 3) throw std::runtime_error("zone " + zone.name + " needs 3 polygon points");
    if (!(zone.max_height_m > zone.min_height_m)) throw std::runtime_error("zone " + zone.name + " height range is invalid");
    if (zone.enter_points < zone.exit_points) throw std::runtime_error("zone " + zone.name + " enter_points must be >= exit_points");
    if (zone.exit_points == 0) throw std::runtime_error("zone " + zone.name + " exit_points must be positive");
  }
}

}  // namespace specter
