#include "specter_sense/state_writer.hpp"

#include <boost/json.hpp>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace specter {
namespace {

std::string timestamp(std::chrono::system_clock::time_point time) {
  const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(time);
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(time - seconds).count();
  const std::time_t raw = std::chrono::system_clock::to_time_t(seconds);
  std::tm utc{};
  gmtime_r(&raw, &utc);
  std::ostringstream output;
  output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S") << '.'
         << std::setw(3) << std::setfill('0') << millis << 'Z';
  return output.str();
}

boost::json::value optional_number(const std::optional<double>& value) {
  return value ? boost::json::value(*value) : boost::json::value(nullptr);
}

}  // namespace

boost::json::value snapshot_to_json(const Snapshot& snapshot) {
  boost::json::object root;
  root["schema_version"] = 1;
  root["generated_at"] = timestamp(snapshot.generated_at);
  root["last_valid_frame_at"] = snapshot.last_valid_frame_at
      ? boost::json::value(timestamp(*snapshot.last_valid_frame_at)) : boost::json::value(nullptr);
  root["sensor"] = {
      {"connected", snapshot.sensor.connected},
      {"reconnecting", snapshot.sensor.reconnecting},
      {"status", snapshot.sensor.status}};
  boost::json::object zones;
  const auto generated = snapshot.generated_at;
  for (const auto& zone : snapshot.zones) {
    const auto age = std::max<std::int64_t>(0, std::chrono::duration_cast<std::chrono::milliseconds>(generated - zone.observed_at).count());
    boost::json::object item{
        {"occupied", zone.occupied},
        {"occupancy_score", zone.occupancy_score},
        {"foreground_points", zone.foreground_points},
        {"observed_at", timestamp(zone.observed_at)},
        {"age_ms", age},
        {"nearest_range_m", optional_number(zone.nearest_range_m)},
        {"farthest_range_m", optional_number(zone.farthest_range_m)}};
    if (zone.centroid_m) {
      item["centroid_m"] = {{"x", zone.centroid_m->x}, {"y", zone.centroid_m->y}, {"z", zone.centroid_m->z}};
    } else {
      item["centroid_m"] = nullptr;
    }
    zones[zone.name] = std::move(item);
  }
  root["zones"] = std::move(zones);
  return root;
}

void write_snapshot_atomic(const std::filesystem::path& path, const Snapshot& snapshot) {
  if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
  auto temporary = path;
  temporary += ".tmp";
  {
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    if (!output) throw std::runtime_error("cannot write state file: " + temporary.string());
    output << boost::json::serialize(snapshot_to_json(snapshot)) << '\n';
    output.flush();
    if (!output) throw std::runtime_error("failed writing state file: " + temporary.string());
  }
  std::filesystem::rename(temporary, path);
}

}  // namespace specter
