#include "specter_sense/config.hpp"
#include "specter_sense/pipeline.hpp"
#include "specter_sense/socket_publisher.hpp"
#include "specter_sense/state_writer.hpp"

#include <boost/json.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

specter::SensorConfig test_config() {
  specter::SensorConfig config;
  config.name = "test";
  config.processing = {0.5, 4.0, 0.2, 0.0, 1};
  config.tracking.enabled = false;
  config.camera_to_room.matrix = {1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 1, 0, 0, 0, 1};
  specter::ZoneConfig zone;
  zone.name = "room";
  zone.floor_polygon = {{-1, 0}, {1, 0}, {1, 3}, {-1, 3}};
  zone.min_height_m = 0;
  zone.max_height_m = 2;
  zone.enter_points = 1;
  zone.exit_points = 1;
  zone.enter_after = std::chrono::milliseconds(0);
  zone.exit_after = std::chrono::milliseconds(0);
  config.zones.push_back(zone);
  return config;
}

specter::DepthFrame tracking_frame(bool objects, int horizontal_offset = 0) {
  constexpr std::size_t width = 40;
  constexpr std::size_t height = 30;
  specter::DepthFrame result{width, height, std::vector<float>(width * height, 3000.0F),
                             {25.0, 25.0, 19.5, 14.5}, std::chrono::system_clock::now()};
  if (!objects) return result;
  for (std::size_t y = 5; y < 25; ++y) {
    for (int x = 4 + horizontal_offset; x < 12 + horizontal_offset; ++x)
      result.depth_mm[y * width + static_cast<std::size_t>(x)] = 2000.0F;
    for (std::size_t x = 28; x < 36; ++x) result.depth_mm[y * width + x] = 2000.0F;
  }
  return result;
}

specter::IgnorePlaneConfig test_ignore_plane() {
  return {"mirror", true,
          {{{-0.3, 2.0, 0.0}, {0.3, 2.0, 0.0}, {0.3, 2.0, 2.0}, {-0.3, 2.0, 2.0}}},
          0.05, 0.03, std::nullopt};
}

specter::DepthFrame frame(float depth) {
  return {1, 1, {depth}, {1, 1, 0, 0}, std::chrono::system_clock::now()};
}

void geometry_test() {
  const std::vector<specter::Point2> square{{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  require(specter::point_in_polygon({1, 1}, square), "inside point rejected");
  require(!specter::point_in_polygon({3, 1}, square), "outside point accepted");
  specter::Transform transform;
  transform.matrix = {1, 0, 0, 1, 0, 1, 0, 2, 0, 0, 1, 3, 0, 0, 0, 1};
  const auto point = specter::transform_point(transform, {1, 2, 3});
  require(point.x == 2 && point.y == 4 && point.z == 6, "transform is incorrect");
  const auto recovered = specter::transform_point(specter::invert_transform(transform), point);
  require(std::abs(recovered.x - 1) < 1e-9 && std::abs(recovered.y - 2) < 1e-9 &&
          std::abs(recovered.z - 3) < 1e-9, "inverse camera transform is incorrect");
  const auto left = specter::deproject_depth({100, 100, 50, 40}, 25, 40, 2);
  const auto right = specter::deproject_depth({100, 100, 50, 40}, 75, 40, 2);
  require(left.x < 0 && right.x > 0, "depth deprojection changed camera-space handedness");
  const specter::Intrinsics intrinsics{100, 100, 50, 40};
  const auto room = specter::transform_point(transform, specter::deproject_depth(intrinsics, 25, 30, 2));
  const auto pixel = specter::project_room_to_depth(transform, intrinsics, room);
  require(pixel && std::abs(pixel->x - 25) < 1e-9 && std::abs(pixel->y - 30) < 1e-9,
          "room point did not round-trip through camera projection");
  require(!specter::project_room_to_depth(transform, intrinsics,
          specter::transform_point(transform, {0, 0, -1})), "point behind camera projected into depth image");

  const auto plane = test_ignore_plane();
  const auto hit = specter::ray_ignore_plane_intersection(plane, {0, 0, 1}, {0, 1, 0});
  require(hit && std::abs(*hit - 2.0) < 0.0001, "bounded ray did not hit ignore plane");
  require(!specter::ray_ignore_plane_intersection(plane, {0, 0, 1}, {1, 0, 0}),
          "parallel ray hit ignore plane");
  require(!specter::ray_ignore_plane_intersection(plane, {0, 0, 1}, {0.2, 1, 0}),
          "ray outside bounded plane was accepted");
  auto margin_plane = plane;
  margin_plane.margin_m = 0.15;
  require(specter::ray_ignore_plane_intersection(margin_plane, {0, 0, 1}, {0.2, 1, 0}).has_value(),
          "ignore plane margin did not expand ray bounds");
}

void pipeline_test() {
  specter::OccupancyPipeline pipeline(test_config());
  require(!pipeline.process(frame(2500))[0].occupied, "background frame occupied");
  require(!pipeline.process(frame(2500))[0].occupied, "warm background occupied");
  const auto occupied = pipeline.process(frame(1500))[0];
  require(occupied.occupied, "foreground was not occupied");
  require(occupied.foreground_points == 1, "foreground evidence mismatch");
  require(occupied.centroid_m.has_value(), "metric centroid omitted");
  require(std::abs(occupied.centroid_m->y - 1.5) < 0.001, "centroid room range mismatch");
  require(!pipeline.process(frame(2500))[0].occupied, "zero-delay exit failed");

  const auto invalid = pipeline.process(frame(std::numeric_limits<float>::quiet_NaN()))[0];
  require(invalid.foreground_points == 0, "invalid depth became foreground evidence");
}

void tracking_test() {
  auto config = test_config();
  config.tracking.enabled = true;
  config.tracking.voxel_size_m = 0.08;
  config.tracking.min_cluster_points = 20;
  config.tracking.association_max_distance_m = 0.8;
  config.tracking.confirmation_frames = 2;
  config.tracking.max_missed_frames = 2;
  config.camera_to_room.matrix = {1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 1.2, 0, 0, 0, 1};
  config.zones[0].floor_polygon = {{-3, 0}, {3, 0}, {3, 4}, {-3, 4}};
  config.zones[0].max_height_m = 3;
  specter::OccupancyPipeline pipeline(config);
  pipeline.process(tracking_frame(false));
  pipeline.process(tracking_frame(false));
  pipeline.process(tracking_frame(true));
  require(pipeline.last_tracks().empty(), "tentative tracks were published");
  pipeline.process(tracking_frame(true));
  require(pipeline.last_tracks().size() == 2, "two foreground objects were not tracked separately");
  const auto first_id = pipeline.last_tracks().front().id;
  require(pipeline.last_tracks().front().classification == "likely_human", "human geometry was not classified");
  require(pipeline.last_tracks().front().posture == "standing", "confirmed upright track was not standing");
  pipeline.process(tracking_frame(true, 1));
  require(pipeline.last_tracks().front().id == first_id, "track id changed after bounded motion");
  pipeline.process(tracking_frame(false));
  require(pipeline.last_tracks().size() == 2 && pipeline.last_tracks().front().occluded,
          "temporarily missing tracks were not coasted");
  pipeline.process(tracking_frame(false));
  require(pipeline.last_tracks().size() == 2, "tracks expired before the configured missing-frame window");
  pipeline.process(tracking_frame(false));
  require(pipeline.last_tracks().empty(), "tracks survived beyond the configured missing-frame window");
  pipeline.set_tracking_enabled(false);
  pipeline.process(tracking_frame(true));
  pipeline.process(tracking_frame(true));
  require(pipeline.last_tracks().empty(), "disabled tracking published tracks");
  pipeline.set_tracking_enabled(true);
  pipeline.process(tracking_frame(true));
  pipeline.process(tracking_frame(true));
  require(pipeline.last_tracks().size() == 2, "re-enabled tracking did not resume without background reset");
  pipeline.reset_tracking();
  require(pipeline.last_tracks().empty(), "tracking reset retained published tracks");
}

void ignore_plane_pipeline_test() {
  auto config = test_config();
  config.tracking.enabled = true;
  config.tracking.min_cluster_points = 1;
  config.tracking.confirmation_frames = 1;
  config.camera_to_room.matrix = {1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 1, 0, 0, 0, 1};
  config.zones[0].floor_polygon = {{-3, 0}, {3, 0}, {3, 5}, {-3, 5}};
  config.zones[0].max_height_m = 3;
  config.ignore_planes = {test_ignore_plane()};
  specter::OccupancyPipeline pipeline(config, true);
  auto scene = [] (float depth) {
    return specter::DepthFrame{5, 5, std::vector<float>(25, depth), {4, 4, 2, 2},
                               std::chrono::system_clock::now()};
  };
  pipeline.process(scene(4000));
  pipeline.process(scene(4000));
  const auto baseline_rejected = pipeline.last_ignore_plane_states()[0].rejected_points;
  auto before_plane = scene(4000);
  before_plane.depth_mm[2 * 5 + 2] = 1500;
  pipeline.process(before_plane);
  require(pipeline.last_ignore_plane_states()[0].rejected_points + 1 == baseline_rejected,
          "depth return before ignore plane was rejected");
  auto on_surface_tolerance = scene(4000);
  on_surface_tolerance.depth_mm[2 * 5 + 2] = 1975;
  pipeline.process(on_surface_tolerance);
  require(pipeline.last_ignore_plane_states()[0].rejected_points == baseline_rejected,
          "surface tolerance did not reject a near-plane sample");
  auto reflected = scene(4000);
  reflected.depth_mm[2 * 5 + 2] = 3000;
  reflected.depth_mm[2 * 5 + 4] = 3000;
  const auto states = pipeline.process(reflected);
  require(states[0].foreground_points == 1, "behind-plane reflection reached occupancy evidence");
  require(pipeline.last_tracks().size() == 1, "legitimate point outside ignore plane did not reach tracking");
  require(pipeline.last_ignore_plane_states().size() == 1 &&
          pipeline.last_ignore_plane_states()[0].rejected_points > 0,
          "ignore plane diagnostics omitted rejected samples");
  require(pipeline.last_ignore_plane_states()[0].matched_points ==
              pipeline.last_ignore_plane_states()[0].rejected_points,
          "full-ignore plane activity did not report all matched samples");
  require(!pipeline.last_ignored_points().empty(), "calibration diagnostics omitted ignored room points");

  auto thresholded_config = config;
  thresholded_config.ignore_planes[0].noise_threshold_points = 0;
  specter::OccupancyPipeline thresholded(thresholded_config, true);
  thresholded.process(scene(4000));
  thresholded.process(scene(4000));
  require(thresholded.last_ignore_plane_states()[0].matched_points > 0 &&
              thresholded.last_ignore_plane_states()[0].activity_points == 0 &&
              thresholded.last_ignore_plane_states()[0].rejected_points == 0,
          "static behind-plane background inflated threshold activity");
  const auto thresholded_states = thresholded.process(reflected);
  require(thresholded.last_ignore_plane_states()[0].rejected_points == 0,
          "above-threshold plane evidence was partially rejected");
  require(thresholded.last_ignore_plane_states()[0].activity_points > 0,
          "above-threshold plane activity was not reported");
  require(thresholded.last_ignore_plane_states()[0].noise_threshold_points == 0,
          "plane activity diagnostics omitted sensitivity");
  require(thresholded_states[0].foreground_points > states[0].foreground_points,
          "above-threshold plane evidence did not pass through to occupancy");
  thresholded_config.ignore_planes[0].noise_threshold_points = 1000000;
  thresholded.set_ignore_planes(thresholded_config.ignore_planes);
  thresholded.process(reflected);
  require(thresholded.last_ignore_plane_states()[0].rejected_points > 0,
          "below-threshold plane noise was not rejected");
  require(thresholded.last_ignore_plane_states()[0].activity_points ==
              thresholded.last_ignore_plane_states()[0].rejected_points,
          "suppressed plane activity counters disagree");

  pipeline.set_ignore_planes({});
  pipeline.process(reflected);
  require(pipeline.last_ignore_plane_states().empty(), "removed ignore plane retained diagnostics");
}

void validation_test() {
  auto config = test_config();
  config.zones.push_back(config.zones.front());
  bool rejected = false;
  try {
    specter::validate_sensor(config);
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected, "duplicate zone names were accepted");

  config = test_config();
  config.zones[0].floor_polygon = {{0, 0}, {1, 1}, {0, 1}, {1, 0}};
  rejected = false;
  try {
    specter::validate_sensor(config);
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected, "self-intersecting polygon was accepted");

  config = test_config();
  auto invalid_plane = test_ignore_plane();
  invalid_plane.corners_m[2].y += 0.1;
  config.ignore_planes = {invalid_plane};
  rejected = false;
  try {
    specter::validate_sensor(config);
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected, "non-planar ignore rectangle was accepted");

  config = test_config();
  auto camera_plane = test_ignore_plane();
  for (auto& corner : camera_plane.corners_m) corner.y = 0;
  config.ignore_planes = {camera_plane};
  rejected = false;
  try {
    specter::validate_sensor(config);
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected, "ignore plane intersecting the camera origin was accepted");
}

void config_round_trip_test() {
  auto config = test_config();
  config.ignore_planes = {test_ignore_plane()};
  config.ignore_planes[0].noise_threshold_points = 42;
  specter::AppConfig app_config{{config}};
  const auto text = specter::serialize_config(app_config);
  const auto json = boost::json::parse(text).as_object();
  const auto& sensor = json.at("sensors").as_array().front().as_object();
  require(sensor.at("camera_to_room").as_array().size() == 16, "serialized transform size mismatch");
  require(sensor.at("zones").as_array().front().as_object().at("name").as_string() == "room",
          "serialized zone mismatch");
  require(sensor.at("tracking").as_object().at("enabled").as_bool() == config.tracking.enabled,
          "serialized tracking config mismatch");
  require(sensor.at("ignore_planes").as_array().size() == 1, "serialized ignore plane mismatch");
  require(sensor.at("ignore_planes").as_array().front().as_object().at("noise_threshold_points").to_number<std::size_t>() == 42,
          "serialized ignore-plane noise threshold mismatch");
  const auto path = std::filesystem::temp_directory_path() /
      ("specter-sense-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".json");
  specter::save_config_atomic(path, app_config);
  const auto loaded = specter::load_config(path);
  std::filesystem::remove(path);
  require(loaded.sensors.size() == 1 && loaded.sensors.front().zones.front().name == "room" &&
              loaded.sensors.front().ignore_planes.size() == 1,
          "atomic config round-trip failed");
  require(loaded.sensors.front().ignore_planes.front().noise_threshold_points == 42,
          "ignore-plane noise threshold round-trip failed");

  auto duplicate = app_config;
  duplicate.sensors.push_back(config);
  bool rejected = false;
  try {
    specter::validate_config(duplicate);
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected, "duplicate sensor name was accepted");

  auto multiple = app_config;
  auto second = config;
  second.name = "second";
  multiple.sensors.push_back(second);
  specter::validate_config(multiple);
}

std::string receive_message(int fd) {
  pollfd descriptor{fd, POLLIN, 0};
  require(::poll(&descriptor, 1, 1000) == 1, "timed out waiting for socket publisher");
  std::array<char, 16384> buffer{};
  const auto received = ::recv(fd, buffer.data(), buffer.size(), 0);
  require(received > 0, "socket publisher closed without a message");
  return {buffer.data(), static_cast<std::size_t>(received)};
}

void socket_publisher_test() {
  const auto path = std::filesystem::temp_directory_path() /
      ("specter-sense-socket-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".sock");
  int client = -1;
  {
    std::unique_ptr<specter::SocketPublisher> publisher;
    try {
      publisher = std::make_unique<specter::SocketPublisher>(path, std::chrono::hours(1));
    } catch (const std::runtime_error& error) {
      if (std::string(error.what()).find("Operation not permitted") != std::string::npos) {
        std::cout << "socket publisher test skipped: sandbox forbids Unix sockets\n";
        return;
      }
      throw;
    }
    specter::Snapshot snapshot;
    snapshot.generated_at = std::chrono::system_clock::now();
    specter::SensorSnapshot sensor;
    sensor.last_valid_frame_at = snapshot.generated_at;
    sensor.sensor = {true, false, "streaming"};
    sensor.zones = {{"room", false, 0, 0, std::nullopt, std::nullopt, std::nullopt, snapshot.generated_at}};
    snapshot.sensors = {{"test", sensor}};
    publisher->publish(snapshot);

    client = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    require(client >= 0, "failed to create socket test client");
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    const auto path_text = path.string();
    require(path_text.size() < sizeof(address.sun_path), "test socket path too long");
    std::memcpy(address.sun_path, path_text.c_str(), path_text.size() + 1);
    require(::connect(client, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0,
            "failed to connect socket test client");
    publisher->publish(snapshot);
    const auto first = boost::json::parse(receive_message(client)).as_object();
    require(first.at("type").as_string() == "snapshot", "new client did not receive a snapshot");
    require(first.at("state").as_object().at("sensors").as_object().at("test").as_object()
                .at("zones").as_object().contains("room"), "snapshot omitted zone state");

    snapshot.generated_at = std::chrono::system_clock::now();
    snapshot.sensors[0].second.zones[0].occupied = true;
    publisher->publish(snapshot);
    const auto changed = boost::json::parse(receive_message(client)).as_object();
    require(changed.at("type").as_string() == "state", "occupancy change did not emit state event");
    require(changed.at("reason").as_string() == "occupancy_changed", "occupancy event reason mismatch");
    require(changed.at("sequence").to_number<std::uint64_t>() > first.at("sequence").to_number<std::uint64_t>(),
            "sequence did not increase");

    struct stat metadata{};
    require(::stat(path.c_str(), &metadata) == 0, "publisher socket path disappeared");
    require((metadata.st_mode & 0777) == 0600, "publisher socket permissions are not 0600");
    ::close(client);
    client = -1;
  }
  if (client >= 0) ::close(client);
  require(!std::filesystem::exists(path), "publisher did not remove its socket on shutdown");
}

void json_test() {
  specter::Snapshot snapshot;
  snapshot.generated_at = std::chrono::system_clock::now();
  specter::SensorSnapshot sensor;
  sensor.last_valid_frame_at = snapshot.generated_at;
  sensor.sensor = {true, false, "streaming"};
  sensor.zones = {{"desk", true, 1.0, 42, specter::Point3{1, 2, 3}, 1.2, 2.1, snapshot.generated_at}};
  sensor.tracks = {{"track-7", "confirmed", "likely_human", 0.8, "standing", 0.7,
                      {1, 2, 0.9}, {0.1, 0, 0}, {0.5, 0.4, 1.7}, 500, {"desk"}, false,
                      snapshot.generated_at}};
  sensor.ignore_planes = {{"mirror", true, 321, 456, 123, 500}};
  snapshot.sensors = {{"test", sensor}};
  const auto json = specter::snapshot_to_json(snapshot).as_object();
  require(json.at("schema_version").as_int64() == 2, "schema version mismatch");
  const auto& sensor_json = json.at("sensors").as_object().at("test").as_object();
  require(sensor_json.at("zones").as_object().at("desk").as_object().at("occupied").as_bool(), "zone JSON mismatch");
  const auto& track = sensor_json.at("tracks").as_object().at("track-7").as_object();
  require(track.at("posture").as_string() == "standing", "track JSON mismatch");
  require(track.at("bounds_m").as_object().at("height").as_double() == 1.7, "track bounds JSON mismatch");
  require(sensor_json.at("ignore_planes").as_object().at("mirror").as_object().at("rejected_points").to_number<std::size_t>() == 321,
          "ignore plane diagnostics JSON mismatch");
  require(sensor_json.at("ignore_planes").as_object().at("mirror").as_object().at("matched_points").to_number<std::size_t>() == 456,
          "ignore plane matched activity JSON mismatch");
  require(sensor_json.at("ignore_planes").as_object().at("mirror").as_object().at("activity_points").to_number<std::size_t>() == 123,
          "ignore plane foreground activity JSON mismatch");
  require(sensor_json.at("ignore_planes").as_object().at("mirror").as_object().at("noise_threshold_points").to_number<std::size_t>() == 500,
          "ignore plane sensitivity JSON mismatch");
}

}  // namespace

int main() {
  try {
    geometry_test();
    pipeline_test();
    tracking_test();
    ignore_plane_pipeline_test();
    validation_test();
    config_round_trip_test();
    socket_publisher_test();
    json_test();
    std::cout << "all core tests passed\n";
  } catch (const std::exception& error) {
    std::cerr << "test failure: " << error.what() << '\n';
    return 1;
  }
}
