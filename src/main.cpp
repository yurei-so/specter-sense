#include "specter_sense/config.hpp"
#include "specter_sense/frame_source.hpp"
#include "specter_sense/pipeline.hpp"
#include "specter_sense/state_writer.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

std::atomic_bool running{true};
void stop(int) { running = false; }

struct Options {
  std::filesystem::path config{"config/specter-sense.example.json"};
  std::filesystem::path output{"state/specter-sense.json"};
  std::string source{"synthetic"};
  std::size_t frames{};
};

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() -> std::string {
      if (++i >= argc) throw std::runtime_error("missing value after " + arg);
      return argv[i];
    };
    if (arg == "--config") options.config = value();
    else if (arg == "--output") options.output = value();
    else if (arg == "--source") options.source = value();
    else if (arg == "--frames") options.frames = static_cast<std::size_t>(std::stoull(value()));
    else if (arg == "--help") {
      std::cout << "Usage: specter-sense [--config PATH] [--output PATH] "
                   "[--source synthetic|kinect] [--frames N]\n";
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

void interruptible_sleep(std::chrono::milliseconds duration) {
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (running && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = parse_options(argc, argv);
    const auto config = specter::load_config(options.config);
    specter::OccupancyPipeline pipeline(config);
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    specter::Snapshot snapshot;
    snapshot.sensor = {false, true, "connecting"};
    std::unique_ptr<specter::FrameSource> source;
    std::size_t completed{};
    std::size_t missed{};
    unsigned reconnect_attempt{};
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
          write_snapshot_atomic(options.output, snapshot);
          const auto exponent = std::min(reconnect_attempt - 1, 5U);
          const auto backoff = std::chrono::seconds(1U << exponent);
          std::cerr << "{\"event\":\"source_connect_failed\",\"attempt\":" << reconnect_attempt
                    << ",\"backoff_ms\":" << std::chrono::duration_cast<std::chrono::milliseconds>(backoff).count()
                    << ",\"error\":\"" << error.what() << "\"}\n";
          interruptible_sleep(backoff);
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
      write_snapshot_atomic(options.output, snapshot);
    }
    std::cerr << "{\"event\":\"stopped\",\"frames\":" << completed << "}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "{\"event\":\"fatal\",\"error\":\"" << error.what() << "\"}\n";
    return 1;
  }
}
