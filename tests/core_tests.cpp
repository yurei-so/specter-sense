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

specter::AppConfig test_config() {
  specter::AppConfig config;
  config.processing = {0.5, 4.0, 0.2, 0.0, 1};
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
  const auto left = specter::deproject_depth({100, 100, 50, 40}, 25, 40, 2);
  const auto right = specter::deproject_depth({100, 100, 50, 40}, 75, 40, 2);
  require(left.x < 0 && right.x > 0, "depth deprojection changed camera-space handedness");
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

void validation_test() {
  auto config = test_config();
  config.zones.push_back(config.zones.front());
  bool rejected = false;
  try {
    specter::validate_config(config);
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected, "duplicate zone names were accepted");

  config = test_config();
  config.zones[0].floor_polygon = {{0, 0}, {1, 1}, {0, 1}, {1, 0}};
  rejected = false;
  try {
    specter::validate_config(config);
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected, "self-intersecting polygon was accepted");
}

void config_round_trip_test() {
  const auto config = test_config();
  const auto text = specter::serialize_config(config);
  const auto json = boost::json::parse(text).as_object();
  require(json.at("camera_to_room").as_array().size() == 16, "serialized transform size mismatch");
  require(json.at("zones").as_array().front().as_object().at("name").as_string() == "room",
          "serialized zone mismatch");
  const auto path = std::filesystem::temp_directory_path() /
      ("specter-sense-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".json");
  specter::save_config_atomic(path, config);
  const auto loaded = specter::load_config(path);
  std::filesystem::remove(path);
  require(loaded.zones.size() == 1 && loaded.zones.front().name == "room", "atomic config round-trip failed");
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
    snapshot.last_valid_frame_at = snapshot.generated_at;
    snapshot.sensor = {true, false, "streaming"};
    snapshot.zones = {{"room", false, 0, 0, std::nullopt, std::nullopt, std::nullopt, snapshot.generated_at}};
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
    require(first.at("state").as_object().at("zones").as_object().contains("room"), "snapshot omitted zone state");

    snapshot.generated_at = std::chrono::system_clock::now();
    snapshot.zones[0].occupied = true;
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
  snapshot.last_valid_frame_at = snapshot.generated_at;
  snapshot.sensor = {true, false, "streaming"};
  snapshot.zones = {{"desk", true, 1.0, 42, specter::Point3{1, 2, 3}, 1.2, 2.1, snapshot.generated_at}};
  const auto json = specter::snapshot_to_json(snapshot).as_object();
  require(json.at("schema_version").as_int64() == 1, "schema version mismatch");
  require(json.at("zones").as_object().at("desk").as_object().at("occupied").as_bool(), "zone JSON mismatch");
}

}  // namespace

int main() {
  try {
    geometry_test();
    pipeline_test();
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
