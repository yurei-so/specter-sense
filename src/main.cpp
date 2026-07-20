#include "specter_sense/config.hpp"
#include "specter_sense/environment.hpp"
#include "specter_sense/frame_source.hpp"
#include "specter_sense/pipeline.hpp"
#include "specter_sense/socket_publisher.hpp"
#include "specter_sense/state_writer.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <unistd.h>

namespace {

std::atomic_bool running{true};
void stop(int) { running = false; }

struct Options {
  std::filesystem::path config{"config/specter-sense.example.json"};
  std::optional<std::filesystem::path> output;
  std::optional<std::filesystem::path> socket;
  std::chrono::milliseconds socket_interval{100};
  std::string source{"synthetic"};
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
  if (const auto value = environment("SPECTER_SENSE_SOURCE"); value && !value->empty()) options.source = *value;
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
    else if (arg == "--source") options.source = value();
    else if (arg == "--frames") options.frames = static_cast<std::size_t>(std::stoull(value()));
    else if (arg == "--help") {
      std::cout << "Usage: specter-sense [--config PATH] [--output PATH|--no-state-file] "
                   "[--socket PATH|--no-socket] [--socket-interval-ms N] "
                   "[--source synthetic|kinect] [--frames N]\n"
                   "Defaults are read from .env (or $SPECTER_SENSE_ENV); command-line options win.\n";
      std::exit(0);
    } else throw std::runtime_error("unknown argument: " + arg);
  }
  return options;
}

std::unique_ptr<specter::FrameSource> make_source(const std::string& name) {
  if (name == "synthetic") return specter::make_synthetic_source();
#ifdef SPECTER_SENSE_HAS_KINECT
  if (name == "kinect") return specter::make_kinect_source();
#else
  if (name == "kinect") throw std::runtime_error("binary was built without libfreenect2 support");
#endif
  throw std::runtime_error("unknown source: " + name);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = parse_options(argc, argv);
    const auto config = specter::load_config(options.config);
    specter::OccupancyPipeline pipeline(config);
    std::unique_ptr<specter::SocketPublisher> socket;
    if (options.socket) socket = std::make_unique<specter::SocketPublisher>(*options.socket, options.socket_interval);
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    specter::Snapshot snapshot;
    snapshot.sensor = {false, true, "connecting"};
    std::unique_ptr<specter::FrameSource> source;
    std::size_t completed{};
    std::size_t missed{};
    unsigned reconnect_attempt{};
    auto next_file_write = std::chrono::steady_clock::time_point::min();
    auto publish_outputs = [&](bool force_file = false) {
      if (socket) socket->publish(snapshot);
      const auto now = std::chrono::steady_clock::now();
      if (options.output && (force_file || now >= next_file_write)) {
        write_snapshot_atomic(*options.output, snapshot);
        next_file_write = now + std::chrono::seconds(1);
      }
    };
    publish_outputs(true);
    if (socket) std::cerr << "{\"event\":\"socket_listening\",\"path\":\"" << socket->path().string() << "\"}\n";
    while (running && (options.frames == 0 || completed < options.frames)) {
      snapshot.generated_at = std::chrono::system_clock::now();
      if (!source) {
        try {
          source = make_source(options.source);
          reconnect_attempt = 0;
          missed = 0;
          snapshot.sensor = {true, false, "streaming"};
          std::cerr << "{\"event\":\"source_connected\",\"source\":\"" << source->name() << "\"}\n";
        } catch (const std::exception& error) {
          ++reconnect_attempt;
          snapshot.sensor = {false, true, "reconnect_backoff"};
          publish_outputs(true);
          const auto exponent = std::min(reconnect_attempt - 1, 5U);
          const auto backoff = std::chrono::seconds(1U << exponent);
          std::cerr << "{\"event\":\"source_connect_failed\",\"attempt\":" << reconnect_attempt
                    << ",\"backoff_ms\":" << std::chrono::duration_cast<std::chrono::milliseconds>(backoff).count()
                    << ",\"error\":\"" << error.what() << "\"}\n";
          const auto deadline = std::chrono::steady_clock::now() + backoff;
          while (running && std::chrono::steady_clock::now() < deadline) {
            publish_outputs();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
          continue;
        }
      }
      std::optional<specter::DepthFrame> frame;
      try {
        frame = source->next(std::chrono::milliseconds(1000));
      } catch (const std::exception& error) {
        std::cerr << "{\"event\":\"source_error\",\"error\":\"" << error.what() << "\"}\n";
      }
      snapshot.generated_at = std::chrono::system_clock::now();
      if (frame) {
        snapshot.zones = pipeline.process(*frame);
        snapshot.last_valid_frame_at = frame->observed_at;
        snapshot.sensor = {true, false, "streaming"};
        missed = 0;
        ++completed;
      } else {
        ++missed;
        snapshot.sensor = {false, true, missed >= 3 ? "stale" : "frame_timeout"};
        if (missed >= 3) source.reset();
      }
      publish_outputs();
    }
    snapshot.generated_at = std::chrono::system_clock::now();
    snapshot.sensor = {false, false, "stopped"};
    publish_outputs(true);
    std::cerr << "{\"event\":\"stopped\",\"frames\":" << completed << "}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "{\"event\":\"fatal\",\"error\":\"" << error.what() << "\"}\n";
    return 1;
  }
}
