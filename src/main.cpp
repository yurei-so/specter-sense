#include "specter_sense/config.hpp"
#include "specter_sense/environment.hpp"
#include "specter_sense/frame_source.hpp"
#include "specter_sense/pipeline.hpp"
#include "specter_sense/socket_publisher.hpp"
#include "specter_sense/state_writer.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <syncstream>
#include <thread>
#include <vector>

#include <unistd.h>

namespace {

std::atomic_bool running{true};
void stop(int) { running = false; }

struct Options {
  std::filesystem::path config{"config/specter-sense.example.json"};
  std::optional<std::filesystem::path> output;
  std::optional<std::filesystem::path> socket;
  std::chrono::milliseconds socket_interval{100};
  std::size_t frames{};
};

std::filesystem::path default_socket_path() {
  if (const char* runtime = std::getenv("XDG_RUNTIME_DIR"); runtime && *runtime)
    return std::filesystem::path(runtime) / "specter-sense.sock";
  return std::filesystem::path("/tmp") / ("specter-sense-" + std::to_string(::getuid()) + ".sock");
}

Options parse_options(int argc, char** argv) {
  Options options;
  const char* env_file = std::getenv("SPECTER_SENSE_ENV");
  specter::load_dotenv_if_present(env_file && *env_file ? env_file : ".env");
  auto environment = [](const char* name) -> std::optional<std::string> {
    if (const char* value = std::getenv(name)) return std::string(value);
    return std::nullopt;
  };
  if (const auto value = environment("SPECTER_SENSE_CONFIG"); value && !value->empty()) options.config = *value;
  if (const auto value = environment("SPECTER_SENSE_OUTPUT"); value && !value->empty()) options.output = *value;
  options.socket = default_socket_path();
  if (const auto value = environment("SPECTER_SENSE_SOCKET"); value && !value->empty()) {
    if (*value == "off" || *value == "none") options.socket.reset();
    else options.socket = *value;
  }
  if (const auto value = environment("SPECTER_SENSE_SOCKET_INTERVAL_MS"); value && !value->empty())
    options.socket_interval = std::chrono::milliseconds(std::stoll(*value));
  if (const auto value = environment("SPECTER_SENSE_FRAMES"); value && !value->empty())
    options.frames = static_cast<std::size_t>(std::stoull(*value));
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() -> std::string {
      if (++i >= argc) throw std::runtime_error("missing value after " + arg);
      return argv[i];
    };
    if (arg == "--config") options.config = value();
    else if (arg == "--output") options.output = value();
    else if (arg == "--no-state-file") options.output.reset();
    else if (arg == "--socket") options.socket = value();
    else if (arg == "--no-socket") options.socket.reset();
    else if (arg == "--socket-interval-ms") options.socket_interval = std::chrono::milliseconds(std::stoll(value()));
    else if (arg == "--frames") options.frames = static_cast<std::size_t>(std::stoull(value()));
    else if (arg == "--help") {
      std::cout << "Usage: specter-sense [--config PATH] [--output PATH|--no-state-file] "
                   "[--socket PATH|--no-socket] [--socket-interval-ms N] "
                   "[--frames N]\n"
                   "Defaults are read from .env (or $SPECTER_SENSE_ENV); command-line options win.\n";
      std::exit(0);
    } else throw std::runtime_error("unknown argument: " + arg);
  }
  return options;
}

std::unique_ptr<specter::FrameSource> make_source(const specter::SensorConfig& sensor) {
  if (sensor.source == "synthetic") return specter::make_synthetic_source();
#ifdef SPECTER_SENSE_HAS_KINECT_V1
  if (sensor.source == "kinect-v1") return specter::make_kinect_v1_source(sensor.serial);
#else
  if (sensor.source == "kinect-v1") throw std::runtime_error("binary was built without libfreenect support");
#endif
#ifdef SPECTER_SENSE_HAS_KINECT_V2
  if (sensor.source == "kinect" || sensor.source == "kinect-v2")
    return specter::make_kinect_v2_source(sensor.serial);
#else
  if (sensor.source == "kinect" || sensor.source == "kinect-v2")
    throw std::runtime_error("binary was built without libfreenect2 support");
#endif
  throw std::runtime_error("unknown source: " + sensor.source);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = parse_options(argc, argv);
    const auto config = specter::load_config(options.config);
    std::unique_ptr<specter::SocketPublisher> socket;
    if (options.socket) socket = std::make_unique<specter::SocketPublisher>(*options.socket, options.socket_interval);
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    specter::Snapshot snapshot;
    snapshot.generated_at = std::chrono::system_clock::now();
    for (const auto& sensor : config.sensors) {
      specter::SensorSnapshot state;
      state.sensor = sensor.enabled ? specter::SensorState{false, true, "connecting"}
                                    : specter::SensorState{false, false, "disabled"};
      snapshot.sensors.push_back({sensor.name, std::move(state)});
    }
    std::mutex snapshot_mutex;
    std::atomic_size_t completed{};
    auto next_file_write = std::chrono::steady_clock::time_point::min();
    auto publish_outputs = [&](bool force_file = false) {
      specter::Snapshot current;
      {
        std::lock_guard lock(snapshot_mutex);
        snapshot.generated_at = std::chrono::system_clock::now();
        current = snapshot;
      }
      if (socket) socket->publish(current);
      const auto now = std::chrono::steady_clock::now();
      if (options.output && (force_file || now >= next_file_write)) {
        write_snapshot_atomic(*options.output, current);
        next_file_write = now + std::chrono::seconds(1);
      }
    };
    publish_outputs(true);
    if (socket) std::cerr << "{\"event\":\"socket_listening\",\"path\":\"" << socket->path().string() << "\"}\n";
    std::vector<std::thread> workers;
    const auto enabled_sensor_count = std::count_if(config.sensors.begin(), config.sensors.end(),
        [](const auto& sensor) { return sensor.enabled; });
    if (options.frames != 0 && enabled_sensor_count == 0)
      throw std::runtime_error("--frames requires at least one enabled sensor");
    for (std::size_t sensor_index = 0; sensor_index < config.sensors.size(); ++sensor_index) {
      if (!config.sensors[sensor_index].enabled) continue;
      workers.emplace_back([&, sensor_index] {
        const auto& sensor_config = config.sensors[sensor_index];
        specter::OccupancyPipeline pipeline(sensor_config);
        std::unique_ptr<specter::FrameSource> source;
        std::size_t missed{};
        unsigned reconnect_attempt{};
        while (running && (options.frames == 0 || completed.load() < options.frames)) {
          if (!source) {
            try {
              source = make_source(sensor_config);
              reconnect_attempt = 0;
              missed = 0;
              {
                std::lock_guard lock(snapshot_mutex);
                snapshot.sensors[sensor_index].second.sensor = {true, false, "streaming"};
              }
              std::osyncstream(std::cerr) << "{\"event\":\"source_connected\",\"sensor\":\"" << sensor_config.name
                                          << "\",\"source\":\"" << source->name() << "\"}\n";
            } catch (const std::exception& error) {
              ++reconnect_attempt;
              {
                std::lock_guard lock(snapshot_mutex);
                snapshot.sensors[sensor_index].second.sensor = {false, true, "reconnect_backoff"};
              }
              const auto exponent = std::min(reconnect_attempt - 1, 5U);
              const auto backoff = std::chrono::seconds(1U << exponent);
              std::osyncstream(std::cerr) << "{\"event\":\"source_connect_failed\",\"sensor\":\"" << sensor_config.name
                                          << "\",\"attempt\":" << reconnect_attempt << ",\"backoff_ms\":"
                                          << std::chrono::duration_cast<std::chrono::milliseconds>(backoff).count()
                                          << ",\"error\":\"" << error.what() << "\"}\n";
              const auto deadline = std::chrono::steady_clock::now() + backoff;
              while (running && std::chrono::steady_clock::now() < deadline)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
              continue;
            }
          }
          std::optional<specter::DepthFrame> frame;
          try {
            frame = source->next(std::chrono::milliseconds(1000));
          } catch (const std::exception& error) {
            std::osyncstream(std::cerr) << "{\"event\":\"source_error\",\"sensor\":\"" << sensor_config.name
                                        << "\",\"error\":\"" << error.what() << "\"}\n";
          }
          if (frame) {
            if (options.frames == 0) {
              ++completed;
            } else {
              auto count = completed.load();
              while (count < options.frames && !completed.compare_exchange_weak(count, count + 1)) {}
              if (count >= options.frames) break;
            }
          }
          std::lock_guard lock(snapshot_mutex);
          auto& state = snapshot.sensors[sensor_index].second;
          if (frame) {
            state.zones = pipeline.process(*frame);
            state.tracks = pipeline.last_tracks();
            state.ignore_planes = pipeline.last_ignore_plane_states();
            state.last_valid_frame_at = frame->observed_at;
            state.sensor = {true, false, "streaming"};
            missed = 0;
          } else {
            ++missed;
            state.sensor = {false, true, missed >= 3 ? "stale" : "frame_timeout"};
            if (missed >= 3) {
              source.reset();
              pipeline.reset_tracking();
              state.tracks.clear();
              state.ignore_planes.clear();
            }
          }
        }
        pipeline.reset_tracking();
        std::lock_guard lock(snapshot_mutex);
        auto& state = snapshot.sensors[sensor_index].second;
        state.sensor = {false, false, "stopped"};
        state.tracks.clear();
        state.ignore_planes.clear();
      });
    }
    while (running && (options.frames == 0 || completed.load() < options.frames)) {
      publish_outputs();
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    running = false;
    for (auto& worker : workers) worker.join();
    publish_outputs(true);
    std::cerr << "{\"event\":\"stopped\",\"frames\":" << completed.load() << "}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "{\"event\":\"fatal\",\"error\":\"" << error.what() << "\"}\n";
    return 1;
  }
}
