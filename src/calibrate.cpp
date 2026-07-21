#include "specter_sense/config.hpp"
#include "specter_sense/environment.hpp"
#include "specter_sense/frame_source.hpp"
#include "specter_sense/pipeline.hpp"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

constexpr double pi = 3.14159265358979323846;
constexpr int panel_width = 330;
std::atomic_bool calibration_running{true};
void stop_calibration(int) { calibration_running = false; }

struct Vec3 {
  double x{}, y{}, z{};
};

Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator*(Vec3 a, double scale) { return {a.x * scale, a.y * scale, a.z * scale}; }
double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double length(Vec3 value) { return std::sqrt(dot(value, value)); }
Vec3 normalized(Vec3 value) {
  const double size = length(value);
  return size > 1e-9 ? value * (1.0 / size) : Vec3{};
}

const char* glyph(char input) {
  static const std::unordered_map<char, const char*> font{
      {'A', "01110/10001/10001/11111/10001/10001/10001"}, {'B', "11110/10001/10001/11110/10001/10001/11110"},
      {'C', "01111/10000/10000/10000/10000/10000/01111"}, {'D', "11110/10001/10001/10001/10001/10001/11110"},
      {'E', "11111/10000/10000/11110/10000/10000/11111"}, {'F', "11111/10000/10000/11110/10000/10000/10000"},
      {'G', "01111/10000/10000/10111/10001/10001/01111"}, {'H', "10001/10001/10001/11111/10001/10001/10001"},
      {'I', "11111/00100/00100/00100/00100/00100/11111"}, {'J', "00111/00010/00010/00010/10010/10010/01100"},
      {'K', "10001/10010/10100/11000/10100/10010/10001"}, {'L', "10000/10000/10000/10000/10000/10000/11111"},
      {'M', "10001/11011/10101/10101/10001/10001/10001"}, {'N', "10001/11001/10101/10011/10001/10001/10001"},
      {'O', "01110/10001/10001/10001/10001/10001/01110"}, {'P', "11110/10001/10001/11110/10000/10000/10000"},
      {'Q', "01110/10001/10001/10001/10101/10010/01101"}, {'R', "11110/10001/10001/11110/10100/10010/10001"},
      {'S', "01111/10000/10000/01110/00001/00001/11110"}, {'T', "11111/00100/00100/00100/00100/00100/00100"},
      {'U', "10001/10001/10001/10001/10001/10001/01110"}, {'V', "10001/10001/10001/10001/10001/01010/00100"},
      {'W', "10001/10001/10001/10101/10101/10101/01010"}, {'X', "10001/10001/01010/00100/01010/10001/10001"},
      {'Y', "10001/10001/01010/00100/00100/00100/00100"}, {'Z', "11111/00001/00010/00100/01000/10000/11111"},
      {'0', "01110/10001/10011/10101/11001/10001/01110"}, {'1', "00100/01100/00100/00100/00100/00100/01110"},
      {'2', "01110/10001/00001/00010/00100/01000/11111"}, {'3', "11110/00001/00001/01110/00001/00001/11110"},
      {'4', "00010/00110/01010/10010/11111/00010/00010"}, {'5', "11111/10000/10000/11110/00001/00001/11110"},
      {'6', "01110/10000/10000/11110/10001/10001/01110"}, {'7', "11111/00001/00010/00100/01000/01000/01000"},
      {'8', "01110/10001/10001/01110/10001/10001/01110"}, {'9', "01110/10001/10001/01111/00001/00001/01110"},
      {'-', "00000/00000/00000/11111/00000/00000/00000"}, {'_', "00000/00000/00000/00000/00000/00000/11111"},
      {'.', "00000/00000/00000/00000/00000/00110/00110"}, {':', "00000/00110/00110/00000/00110/00110/00000"},
      {'/', "00001/00010/00100/01000/10000/00000/00000"}, {'[', "01110/01000/01000/01000/01000/01000/01110"},
      {']', "01110/00010/00010/00010/00010/00010/01110"}, {'+', "00000/00100/00100/11111/00100/00100/00000"},
      {'=', "00000/00000/11111/00000/11111/00000/00000"}, {'?', "01110/10001/00001/00010/00100/00000/00100"},
      {' ', "00000/00000/00000/00000/00000/00000/00000"}};
  const char upper = input >= 'a' && input <= 'z' ? static_cast<char>(input - 'a' + 'A') : input;
  const auto found = font.find(upper);
  return found == font.end() ? font.at('?') : found->second;
}

void draw_text(double x, double y, const std::string& text, double scale = 2.0) {
  glBegin(GL_QUADS);
  for (const char character : text) {
    const char* pattern = glyph(character);
    int row = 0, col = 0;
    for (const char* p = pattern; *p; ++p) {
      if (*p == '/') { ++row; col = 0; continue; }
      if (*p == '1') {
        const double px = x + col * scale;
        const double py = y + row * scale;
        glVertex2d(px, py); glVertex2d(px + scale, py);
        glVertex2d(px + scale, py + scale); glVertex2d(px, py + scale);
      }
      ++col;
    }
    x += 6 * scale;
  }
  glEnd();
}

struct Options {
  std::filesystem::path config{"config/specter-sense.example.json"};
  std::optional<std::string> sensor;
  bool object_tracking{};
  bool camera_view{};
};

Options parse_options(int argc, char** argv) {
  Options options;
  const char* env_file = std::getenv("SPECTER_SENSE_ENV");
  specter::load_dotenv_if_present(env_file && *env_file ? env_file : ".env");
  if (const char* value = std::getenv("SPECTER_SENSE_CONFIG"); value && *value) options.config = value;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() -> std::string {
      if (++i >= argc) throw std::runtime_error("missing value after " + arg);
      return argv[i];
    };
    if (arg == "--config") options.config = value();
    else if (arg == "--sensor") options.sensor = value();
    else if (arg == "--object-tracking") options.object_tracking = true;
    else if (arg == "--camera-view") options.camera_view = true;
    else if (arg == "--help") {
      std::cout << "Usage: specter-sense-calibrate [--config PATH] [--sensor NAME] "
                   "[--object-tracking] [--camera-view]\n";
      std::exit(0);
    } else throw std::runtime_error("unknown argument: " + arg);
  }
  return options;
}

class CalibrationApp {
 public:
  explicit CalibrationApp(Options options)
      : options_(std::move(options)), app_config_(specter::load_config(options_.config)),
        sensor_index_(select_sensor(app_config_, options_.sensor)), config_(app_config_.sensors[sensor_index_]),
        original_(config_), pipeline_(std::make_unique<specter::OccupancyPipeline>(config_, true)),
        object_tracking_(options_.object_tracking) {
    pipeline_->set_tracking_enabled(object_tracking_);
    if (options_.camera_view) view_mode_ = ViewMode::camera;
  }

  int run() {
    if (!glfwInit()) throw std::runtime_error("GLFW initialization failed");
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    window_ = glfwCreateWindow(1280, 800, "specter-sense calibration", nullptr, nullptr);
    if (!window_) { glfwTerminate(); throw std::runtime_error("cannot create OpenGL window"); }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(window_, this);
    glfwSetKeyCallback(window_, key_callback);
    glfwSetCharCallback(window_, char_callback);
    glfwSetCursorPosCallback(window_, cursor_callback);
    glfwSetMouseButtonCallback(window_, mouse_callback);
    glfwSetScrollCallback(window_, scroll_callback);
    std::signal(SIGINT, stop_calibration);
    std::signal(SIGTERM, stop_calibration);
    connect_source();
    while (calibration_running && !glfwWindowShouldClose(window_)) {
      glfwPollEvents();
      acquire();
      render();
      glfwSwapBuffers(window_);
    }
    source_.reset();
    glfwDestroyWindow(window_);
    glfwTerminate();
    return 0;
  }

 private:
  static std::size_t select_sensor(const specter::AppConfig& config, const std::optional<std::string>& name) {
    if (!name) return 0;
    const auto found = std::find_if(config.sensors.begin(), config.sensors.end(),
        [&](const auto& sensor) { return sensor.name == *name; });
    if (found == config.sensors.end()) throw std::runtime_error("unknown configured sensor: " + *name);
    return static_cast<std::size_t>(found - config.sensors.begin());
  }
  enum class GizmoAxis { none, x, y, z };
  enum class ViewMode { perspective, top_down, camera };

  bool top_down() const { return view_mode_ == ViewMode::top_down; }
  bool camera_view() const { return view_mode_ == ViewMode::camera; }

  static CalibrationApp& self(GLFWwindow* window) {
    return *static_cast<CalibrationApp*>(glfwGetWindowUserPointer(window));
  }
  static void key_callback(GLFWwindow* window, int key, int, int action, int mods) {
    if (action == GLFW_PRESS || action == GLFW_REPEAT) self(window).on_key(key, action, mods);
  }
  static void char_callback(GLFWwindow* window, unsigned int codepoint) { self(window).on_char(codepoint); }
  static void cursor_callback(GLFWwindow* window, double x, double y) { self(window).on_cursor(x, y); }
  static void mouse_callback(GLFWwindow* window, int button, int action, int mods) {
    self(window).on_mouse(button, action, mods);
  }
  static void scroll_callback(GLFWwindow* window, double, double y) { self(window).on_scroll(y); }

  void connect_source() {
    if (config_.source == "synthetic") source_ = specter::make_synthetic_source();
#ifdef SPECTER_SENSE_HAS_KINECT_V1
    else if (config_.source == "kinect-v1") source_ = specter::make_kinect_v1_source(config_.serial);
#else
    else if (config_.source == "kinect-v1") throw std::runtime_error("calibrator built without Kinect V1 support");
#endif
#ifdef SPECTER_SENSE_HAS_KINECT_V2
    else if (config_.source == "kinect" || config_.source == "kinect-v2")
      source_ = specter::make_kinect_v2_source(config_.serial);
#else
    else if (config_.source == "kinect" || config_.source == "kinect-v2")
      throw std::runtime_error("calibrator built without Kinect V2 support");
#endif
    else throw std::runtime_error("unknown source: " + config_.source);
    status_ = "CONNECTED TO " + source_->name();
  }

  void acquire() {
    if (frozen_ || !source_) return;
    auto next = source_->next(std::chrono::milliseconds(1));
    if (!next) return;
    frame_ = std::move(*next);
    camera_points_.clear();
    room_points_.clear();
    constexpr std::size_t stride = 3;
    for (std::size_t v = 0; v < frame_->height; v += stride) {
      for (std::size_t u = 0; u < frame_->width; u += stride) {
        const double depth = frame_->depth_mm[v * frame_->width + u] / 1000.0;
        if (!std::isfinite(depth) || depth < config_.processing.min_depth_m || depth > config_.processing.max_depth_m) continue;
        const specter::Point3 camera = specter::deproject_depth(
            frame_->intrinsics, static_cast<double>(u), static_cast<double>(v), depth);
        camera_points_.push_back({camera.x, camera.y, camera.z});
        const auto room = specter::transform_point(config_.camera_to_room, camera);
        room_points_.push_back({room.x, room.y, room.z});
      }
    }
    try {
      zone_states_ = pipeline_->process(*frame_);
      foreground_points_ = pipeline_->last_foreground_points();
      track_states_ = pipeline_->last_tracks();
      ignore_plane_states_ = pipeline_->last_ignore_plane_states();
      ignored_points_ = pipeline_->last_ignored_points();
    } catch (const std::exception& error) {
      status_ = std::string("PIPELINE: ") + error.what();
    }
  }

  void rebuild_pipeline() {
    try {
      specter::validate_sensor(config_);
      pipeline_ = std::make_unique<specter::OccupancyPipeline>(config_, true);
      pipeline_->set_tracking_enabled(object_tracking_);
      track_states_.clear();
      status_ = "CONFIGURATION VALID";
    } catch (const std::exception& error) {
      status_ = std::string("INVALID: ") + error.what();
    }
    if (frame_) refresh_room_points();
  }

  void refresh_room_points() {
    room_points_.clear();
    room_points_.reserve(camera_points_.size());
    for (const auto point : camera_points_) {
      const auto room = specter::transform_point(config_.camera_to_room, {point.x, point.y, point.z});
      room_points_.push_back({room.x, room.y, room.z});
    }
  }

  void remember(const specter::SensorConfig& before) {
    undo_.push_back(before);
    if (undo_.size() > 100) undo_.erase(undo_.begin());
    redo_.clear();
    rebuild_pipeline();
  }

  void remember_ignore_planes(const specter::SensorConfig& before) {
    undo_.push_back(before);
    if (undo_.size() > 100) undo_.erase(undo_.begin());
    redo_.clear();
    refresh_ignore_planes();
  }

  void refresh_ignore_planes() {
    try {
      pipeline_->set_ignore_planes(config_.ignore_planes);
      track_states_.clear();
      status_ = "IGNORE PLANE VALID - RAY MASK WILL REFRESH";
    } catch (const std::exception& error) {
      status_ = std::string("INVALID PLANE: ") + error.what();
    }
  }

  void undo() {
    if (undo_.empty()) return;
    redo_.push_back(config_);
    config_ = undo_.back();
    undo_.pop_back();
    selected_ = config_.zones.empty() ? -1 : std::min(selected_, static_cast<int>(config_.zones.size() - 1));
    selected_plane_ = config_.ignore_planes.empty() ? -1 :
        std::min(selected_plane_, static_cast<int>(config_.ignore_planes.size() - 1));
    rebuild_pipeline();
  }

  void redo() {
    if (redo_.empty()) return;
    undo_.push_back(config_);
    config_ = redo_.back();
    redo_.pop_back();
    rebuild_pipeline();
  }

  void on_key(int key, int action, int mods) {
    if (renaming_) {
      if (key == GLFW_KEY_BACKSPACE && action == GLFW_PRESS && !rename_buffer_.empty()) rename_buffer_.pop_back();
      return;
    }
    const bool control = (mods & GLFW_MOD_CONTROL) != 0;
    if (control && key == GLFW_KEY_N && action == GLFW_PRESS) create_box();
    else if (control && key == GLFW_KEY_M && action == GLFW_PRESS) create_ignore_plane();
  }

  void create_box() {
    const auto before = config_;
    specter::ZoneConfig zone;
    std::size_t suffix = config_.zones.size() + 1;
    do {
      zone.name = "zone_" + std::to_string(suffix++);
    } while (std::any_of(config_.zones.begin(), config_.zones.end(), [&](const auto& item) { return item.name == zone.name; }));
    constexpr double half = 0.5;
    zone.floor_polygon = {
        {target_x_ - half, target_y_ - half}, {target_x_ + half, target_y_ - half},
        {target_x_ + half, target_y_ + half}, {target_x_ - half, target_y_ + half}};
    zone.min_height_m = 0;
    zone.max_height_m = 2;
    config_.zones.push_back(std::move(zone));
    selected_ = static_cast<int>(config_.zones.size() - 1);
    selected_plane_ = -1;
    selected_vertex_ = 0;
    selected_top_ = false;
    remember(before);
    status_ = "BOUNDING BOX CREATED - CLICK A CORNER OR GIZMO AXIS";
  }

  void create_ignore_plane() {
    const auto before = config_;
    specter::IgnorePlaneConfig plane;
    std::size_t suffix = config_.ignore_planes.size() + 1;
    do {
      plane.name = "ignore_plane_" + std::to_string(suffix++);
    } while (std::any_of(config_.ignore_planes.begin(), config_.ignore_planes.end(),
                         [&](const auto& item) { return item.name == plane.name; }));
    const auto camera_point = specter::transform_point(config_.camera_to_room, {0, 0, 0});
    Vec3 normal = normalized({camera_point.x - target_x_, camera_point.y - target_y_, 0});
    if (length(normal) < 0.5) normal = {0, -1, 0};
    const Vec3 horizontal = normalized(cross({0, 0, 1}, normal));
    const Vec3 vertical{0, 0, 1};
    const Vec3 center{target_x_, target_y_, std::max(0.75, target_z_)};
    constexpr double half_width = 0.5, half_height = 0.75;
    const std::array<Vec3, 4> corners{
        center - horizontal * half_width - vertical * half_height,
        center + horizontal * half_width - vertical * half_height,
        center + horizontal * half_width + vertical * half_height,
        center - horizontal * half_width + vertical * half_height};
    for (std::size_t i = 0; i < 4; ++i)
      plane.corners_m[i] = {corners[i].x, corners[i].y, corners[i].z};
    config_.ignore_planes.push_back(std::move(plane));
    selected_plane_ = static_cast<int>(config_.ignore_planes.size() - 1);
    selected_plane_corner_ = -1;
    selected_ = -1;
    selected_vertex_ = -1;
    remember_ignore_planes(before);
    status_ = "IGNORE PLANE CREATED - MOVE CENTER OR RESIZE CORNERS";
  }

  void on_char(unsigned int codepoint) {
    if (renaming_ && codepoint >= 32 && codepoint < 127 && rename_buffer_.size() < 32)
      rename_buffer_.push_back(static_cast<char>(codepoint));
  }

  specter::Point2 screen_to_floor(double x, double y) const {
    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);
    const double viewport_width = std::max(1, width - panel_width);
    const double aspect = viewport_width / std::max(1.0, static_cast<double>(height));
    return {pan_x_ + ((1.0 - x / viewport_width) * 2.0 - 1.0) * top_scale_ * aspect,
            pan_y_ + (1.0 - y / static_cast<double>(height) * 2.0) * top_scale_};
  }

  struct CameraFrame { Vec3 eye, forward, right, up; };
  struct ScreenViewport { double x{}, y{}, width{}, height{}; };

  ScreenViewport camera_screen_viewport(int width, int height) const {
    const double available_width = std::max(1, width - panel_width);
    if (!frame_ || frame_->width == 0 || frame_->height == 0)
      return {0, 0, available_width, static_cast<double>(height)};
    const double source_aspect = static_cast<double>(frame_->width) / static_cast<double>(frame_->height);
    const double available_aspect = available_width / std::max(1.0, static_cast<double>(height));
    if (available_aspect > source_aspect) {
      const double fitted_width = height * source_aspect;
      return {(available_width - fitted_width) * 0.5, 0, fitted_width, static_cast<double>(height)};
    }
    const double fitted_height = available_width / source_aspect;
    return {0, (height - fitted_height) * 0.5, available_width, fitted_height};
  }

  std::optional<specter::Point2> screen_to_depth_pixel(double x, double y) const {
    if (!frame_) return std::nullopt;
    int width, height;
    glfwGetWindowSize(window_, &width, &height);
    const auto viewport = camera_screen_viewport(width, height);
    if (x < viewport.x || x > viewport.x + viewport.width ||
        y < viewport.y || y > viewport.y + viewport.height) return std::nullopt;
    const double displayed_x = (viewport.x + viewport.width - x) / viewport.width *
                               static_cast<double>(frame_->width);
    const double displayed_y = (y - viewport.y) / viewport.height *
                               static_cast<double>(frame_->height);
    return specter::Point2{
        frame_->intrinsics.cx + (displayed_x - frame_->intrinsics.cx) / camera_zoom_,
        frame_->intrinsics.cy + (displayed_y - frame_->intrinsics.cy) / camera_zoom_};
  }

  CameraFrame camera_frame() const {
    const Vec3 target{target_x_, target_y_, target_z_};
    const double horizontal = orbit_distance_ * std::cos(orbit_pitch_);
    const Vec3 eye{
        target.x + horizontal * std::sin(orbit_yaw_),
        target.y - horizontal * std::cos(orbit_yaw_),
        target.z + orbit_distance_ * std::sin(orbit_pitch_)};
    const Vec3 forward = normalized(target - eye);
    const Vec3 right = normalized(cross(forward, {0, 0, 1}));
    return {eye, forward, right, cross(right, forward)};
  }

  std::optional<specter::Point2> project(Vec3 point) const {
    int width, height;
    glfwGetWindowSize(window_, &width, &height);
    const int viewport_width = std::max(1, width - panel_width);
    if (camera_view()) {
      if (!frame_) return std::nullopt;
      const auto pixel = specter::project_room_to_depth(
          config_.camera_to_room, frame_->intrinsics, {point.x, point.y, point.z});
      if (!pixel) return std::nullopt;
      const auto viewport = camera_screen_viewport(width, height);
      const double displayed_x = frame_->intrinsics.cx +
          (pixel->x - frame_->intrinsics.cx) * camera_zoom_;
      const double displayed_y = frame_->intrinsics.cy +
          (pixel->y - frame_->intrinsics.cy) * camera_zoom_;
      return specter::Point2{
          viewport.x + viewport.width -
              displayed_x / static_cast<double>(frame_->width) * viewport.width,
          viewport.y + displayed_y / static_cast<double>(frame_->height) * viewport.height};
    }
    if (top_down()) {
      const double aspect = static_cast<double>(viewport_width) / std::max(1, height);
      return specter::Point2{
          viewport_width - ((point.x - pan_x_) / (top_scale_ * aspect) + 1.0) * 0.5 * viewport_width,
          (1.0 - (point.y - pan_y_) / top_scale_) * 0.5 * height};
    }
    const auto camera = camera_frame();
    const Vec3 relative = point - camera.eye;
    const double depth = dot(relative, camera.forward);
    if (depth <= 0.05) return std::nullopt;
    const double focal = static_cast<double>(height) / (2.0 * std::tan(55.0 * pi / 360.0));
    return specter::Point2{
        viewport_width * 0.5 - dot(relative, camera.right) / depth * focal,
        height * 0.5 - dot(relative, camera.up) / depth * focal};
  }

  Vec3 selected_point() const {
    if (selected_plane_ >= 0) {
      const auto& plane = config_.ignore_planes[static_cast<std::size_t>(selected_plane_)];
      if (selected_plane_corner_ >= 0) {
        const auto& corner = plane.corners_m[static_cast<std::size_t>(selected_plane_corner_)];
        return {corner.x, corner.y, corner.z};
      }
      Vec3 center{};
      for (const auto& corner : plane.corners_m) center = center + Vec3{corner.x, corner.y, corner.z};
      return center * 0.25;
    }
    if (selected_ < 0 || selected_vertex_ < 0) return {};
    const auto& zone = config_.zones[static_cast<std::size_t>(selected_)];
    const auto& point = zone.floor_polygon[static_cast<std::size_t>(selected_vertex_)];
    return {point.x, point.y, selected_top_ ? zone.max_height_m : zone.min_height_m};
  }

  bool pick_corner(double x, double y) {
    double best = 14.0;
    int zone_index = -1, vertex_index = -1;
    bool top = false;
    for (std::size_t z = 0; z < config_.zones.size(); ++z) {
      const auto& zone = config_.zones[z];
      for (std::size_t v = 0; v < zone.floor_polygon.size(); ++v) {
        for (const bool is_top : {false, true}) {
          if (top_down() && is_top) continue;
          const auto& floor = zone.floor_polygon[v];
          const auto screen = project({floor.x, floor.y, is_top ? zone.max_height_m : zone.min_height_m});
          if (!screen) continue;
          const double distance = std::hypot(screen->x - x, screen->y - y);
          if (distance < best) {
            best = distance; zone_index = static_cast<int>(z); vertex_index = static_cast<int>(v); top = is_top;
          }
        }
      }
    }
    if (zone_index < 0) return false;
    selected_ = zone_index; selected_vertex_ = vertex_index; selected_top_ = top;
    selected_plane_ = -1; selected_plane_corner_ = -1;
    status_ = "CORNER SELECTED - DRAG A COLORED AXIS";
    return true;
  }

  bool pick_ignore_plane(double x, double y) {
    double best = 14.0;
    int plane_index = -1, corner_index = -1;
    for (std::size_t index = 0; index < config_.ignore_planes.size(); ++index) {
      const auto& plane = config_.ignore_planes[index];
      for (std::size_t corner = 0; corner < plane.corners_m.size(); ++corner) {
        const auto& point = plane.corners_m[corner];
        const auto screen = project({point.x, point.y, point.z});
        if (!screen) continue;
        const double candidate = std::hypot(screen->x - x, screen->y - y);
        if (candidate < best) { best = candidate; plane_index = static_cast<int>(index); corner_index = static_cast<int>(corner); }
      }
      Vec3 center{};
      for (const auto& point : plane.corners_m) center = center + Vec3{point.x, point.y, point.z};
      center = center * 0.25;
      const auto screen = project(center);
      if (screen) {
        const double candidate = std::hypot(screen->x - x, screen->y - y);
        if (candidate < best) { best = candidate; plane_index = static_cast<int>(index); corner_index = -1; }
      }
    }
    if (plane_index < 0) return false;
    selected_plane_ = plane_index;
    selected_plane_corner_ = corner_index;
    selected_ = -1;
    selected_vertex_ = -1;
    status_ = corner_index >= 0 ? "IGNORE CORNER SELECTED - DRAG AXIS TO RESIZE" :
                                  "IGNORE PLANE SELECTED - DRAG AXIS TO MOVE";
    return true;
  }

  bool pick_camera(double x, double y) {
    if (camera_view()) return false;
    const auto origin = specter::transform_point(config_.camera_to_room, {0, 0, 0});
    const auto screen = project({origin.x, origin.y, origin.z});
    if (!screen || std::hypot(screen->x - x, screen->y - y) >= 18.0) return false;
    view_mode_ = ViewMode::camera;
    status_ = frame_ ? "EXACT KINECT DEPTH VIEW" : "CAMERA VIEW WAITING FOR DEPTH INTRINSICS";
    return true;
  }

  static double distance_to_segment(double px, double py, specter::Point2 a, specter::Point2 b) {
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length_squared = dx * dx + dy * dy;
    const double t = length_squared > 0
        ? std::clamp(((px - a.x) * dx + (py - a.y) * dy) / length_squared, 0.0, 1.0) : 0.0;
    return std::hypot(px - (a.x + t * dx), py - (a.y + t * dy));
  }

  GizmoAxis pick_gizmo(double x, double y) const {
    if (selected_plane_ < 0 && (selected_ < 0 || selected_vertex_ < 0)) return GizmoAxis::none;
    const Vec3 origin = selected_point();
    const auto start = project(origin);
    if (!start) return GizmoAxis::none;
    if (std::hypot(start->x - x, start->y - y) < 14.0) return GizmoAxis::none;
    const std::array<std::pair<GizmoAxis, Vec3>, 3> axes{{
        {GizmoAxis::x, {0.45, 0, 0}}, {GizmoAxis::y, {0, 0.45, 0}}, {GizmoAxis::z, {0, 0, 0.45}}}};
    GizmoAxis best_axis = GizmoAxis::none;
    double best = 9.0;
    for (const auto& [axis, delta] : axes) {
      if (top_down() && axis == GizmoAxis::z) continue;
      const auto end = project(origin + delta);
      if (!end) continue;
      const double distance = distance_to_segment(x, y, *start, *end);
      if (distance < best) { best = distance; best_axis = axis; }
    }
    return best_axis;
  }

  void move_or_resize_ignore_plane(GizmoAxis axis, double metres) {
    auto& plane = config_.ignore_planes[static_cast<std::size_t>(selected_plane_)];
    Vec3 delta{};
    if (axis == GizmoAxis::x) delta.x = metres;
    if (axis == GizmoAxis::y) delta.y = metres;
    if (axis == GizmoAxis::z) delta.z = metres;
    if (selected_plane_corner_ < 0) {
      for (auto& corner : plane.corners_m) {
        corner.x += delta.x; corner.y += delta.y; corner.z += delta.z;
      }
      return;
    }
    const auto& selected_point = plane.corners_m[static_cast<std::size_t>(selected_plane_corner_)];
    resize_selected_plane_to({selected_point.x + delta.x, selected_point.y + delta.y, selected_point.z + delta.z});
  }

  void resize_selected_plane_to(Vec3 candidate) {
    auto& plane = config_.ignore_planes[static_cast<std::size_t>(selected_plane_)];
    const auto original = plane.corners_m;
    const Vec3 u = normalized(Vec3{original[1].x - original[0].x, original[1].y - original[0].y,
                                   original[1].z - original[0].z});
    const Vec3 v = normalized(Vec3{original[3].x - original[0].x, original[3].y - original[0].y,
                                   original[3].z - original[0].z});
    const std::size_t selected = static_cast<std::size_t>(selected_plane_corner_);
    const std::size_t opposite = (selected + 2) % 4;
    const Vec3 opposite_point{original[opposite].x, original[opposite].y, original[opposite].z};
    const Vec3 diagonal = candidate - opposite_point;
    const double width = std::max(0.05, std::abs(dot(diagonal, u)));
    const double height = std::max(0.05, std::abs(dot(diagonal, v)));
    const Vec3 center = (candidate + opposite_point) * 0.5;
    const std::array<Vec3, 4> rebuilt{
        center - u * (width * 0.5) - v * (height * 0.5),
        center + u * (width * 0.5) - v * (height * 0.5),
        center + u * (width * 0.5) + v * (height * 0.5),
        center - u * (width * 0.5) + v * (height * 0.5)};
    for (std::size_t i = 0; i < 4; ++i)
      plane.corners_m[i] = {rebuilt[i].x, rebuilt[i].y, rebuilt[i].z};
  }

  void on_cursor(double x, double y) {
    const double dx = x - mouse_x_;
    const double dy = y - mouse_y_;
    mouse_x_ = x; mouse_y_ = y;
    int width, height;
    glfwGetWindowSize(window_, &width, &height);
    if (gizmo_drag_ != GizmoAxis::none &&
        (selected_plane_ >= 0 || (selected_ >= 0 && selected_vertex_ >= 0))) {
      const Vec3 origin = selected_point();
      Vec3 delta{};
      if (gizmo_drag_ == GizmoAxis::x) delta.x = 0.45;
      if (gizmo_drag_ == GizmoAxis::y) delta.y = 0.45;
      if (gizmo_drag_ == GizmoAxis::z) delta.z = 0.45;
      const auto start = project(origin);
      const auto end = project(origin + delta);
      if (start && end) {
        const double axis_x = end->x - start->x, axis_y = end->y - start->y;
        const double pixels = std::hypot(axis_x, axis_y);
        if (pixels > 1) {
          const double metres = (dx * axis_x + dy * axis_y) / (pixels * pixels) * 0.45;
          if (selected_plane_ >= 0) {
            move_or_resize_ignore_plane(gizmo_drag_, metres);
          } else {
            auto& zone = config_.zones[static_cast<std::size_t>(selected_)];
            auto& vertex = zone.floor_polygon[static_cast<std::size_t>(selected_vertex_)];
            if (gizmo_drag_ == GizmoAxis::x) vertex.x += metres;
            if (gizmo_drag_ == GizmoAxis::y) vertex.y += metres;
            if (gizmo_drag_ == GizmoAxis::z) {
              if (selected_top_) zone.max_height_m = std::max(zone.min_height_m + 0.05, zone.max_height_m + metres);
              else zone.min_height_m = std::min(zone.max_height_m - 0.05, zone.min_height_m + metres);
            }
          }
        }
      }
      if (selected_plane_ >= 0) refresh_ignore_planes(); else rebuild_pipeline();
      return;
    }
    if (right_down_) {
      if (top_down()) {
        const auto current = screen_to_floor(x, y);
        const auto previous = screen_to_floor(x - dx, y - dy);
        pan_x_ -= current.x - previous.x; pan_y_ -= current.y - previous.y;
      } else {
        const double scale = orbit_distance_ * 0.0015;
        const Vec3 right{std::cos(orbit_yaw_), std::sin(orbit_yaw_), 0};
        const Vec3 forward{-std::sin(orbit_yaw_), std::cos(orbit_yaw_), 0};
        target_x_ += right.x * dx * scale + forward.x * dy * scale;
        target_y_ += right.y * dx * scale + forward.y * dy * scale;
      }
      return;
    }
    if (!left_down_) return;
    if (corner_click_) {
      if (camera_view() && selected_plane_ >= 0 && selected_plane_corner_ >= 0 && frame_) {
        const auto pixel = screen_to_depth_pixel(x, y);
        if (pixel) {
          const auto camera_ray = specter::deproject_depth(frame_->intrinsics, pixel->x, pixel->y, 1.0);
          const auto origin_point = specter::transform_point(config_.camera_to_room, {0, 0, 0});
          const auto ray_point = specter::transform_point(config_.camera_to_room, camera_ray);
          specter::IgnorePlaneConfig geometric = config_.ignore_planes[static_cast<std::size_t>(selected_plane_)];
          geometric.enabled = true;
          geometric.margin_m = 50;
          const specter::Point3 origin{origin_point.x, origin_point.y, origin_point.z};
          const specter::Point3 direction{ray_point.x - origin.x, ray_point.y - origin.y, ray_point.z - origin.z};
          const auto hit = specter::ray_ignore_plane_intersection(geometric, origin, direction);
          if (hit) {
            resize_selected_plane_to({origin.x + direction.x * *hit,
                                      origin.y + direction.y * *hit,
                                      origin.z + direction.z * *hit});
            plane_corner_dragged_ = true;
            refresh_ignore_planes();
          }
        }
      }
      return;
    }
    if (top_down() && translating_ && selected_ >= 0) {
      const auto current = screen_to_floor(x, y);
      const auto previous = screen_to_floor(x - dx, y - dy);
      for (auto& point : config_.zones[static_cast<std::size_t>(selected_)].floor_polygon) {
        point.x += current.x - previous.x; point.y += current.y - previous.y;
      }
      rebuild_pipeline();
    } else if (!top_down() && !camera_view()) {
      orbit_yaw_ += dx * 0.006;
      orbit_pitch_ = std::clamp(orbit_pitch_ + dy * 0.006, -1.45, 1.45);
    }
  }

  void on_mouse(int button, int action, int) {
    int width, height;
    glfwGetWindowSize(window_, &width, &height);
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS &&
        (save_preview_ || mouse_x_ > width - panel_width)) {
      handle_panel_click(mouse_x_, mouse_y_, width, height);
      return;
    }
    if (button == GLFW_MOUSE_BUTTON_RIGHT) { right_down_ = action == GLFW_PRESS; return; }
    if (button != GLFW_MOUSE_BUTTON_LEFT) return;
    left_down_ = action == GLFW_PRESS;
    if (action == GLFW_RELEASE) {
      if (gizmo_drag_ != GizmoAxis::none || translating_ || plane_corner_dragged_) {
        if (editing_plane_) remember_ignore_planes(edit_before_); else remember(edit_before_);
      }
      gizmo_drag_ = GizmoAxis::none; translating_ = false; corner_click_ = false;
      editing_plane_ = false; plane_corner_dragged_ = false;
      return;
    }
    const auto axis = pick_gizmo(mouse_x_, mouse_y_);
    if (axis != GizmoAxis::none) {
      edit_before_ = config_; gizmo_drag_ = axis; editing_plane_ = selected_plane_ >= 0; return;
    }
    if (pick_camera(mouse_x_, mouse_y_)) return;
    const auto before_pick = config_;
    if (pick_ignore_plane(mouse_x_, mouse_y_)) {
      edit_before_ = before_pick; editing_plane_ = true; corner_click_ = true; return;
    }
    if (pick_corner(mouse_x_, mouse_y_)) { corner_click_ = true; return; }
    if (!top_down()) return;
    const auto point = screen_to_floor(mouse_x_, mouse_y_);
    edit_before_ = config_;
    if (selected_ >= 0 && specter::point_in_polygon(point, config_.zones[static_cast<std::size_t>(selected_)].floor_polygon)) {
      translating_ = true; return;
    }
    for (std::size_t z = 0; z < config_.zones.size(); ++z)
      if (specter::point_in_polygon(point, config_.zones[z].floor_polygon)) {
        selected_ = static_cast<int>(z); selected_vertex_ = -1;
      }
  }

  void on_scroll(double amount) {
    if (top_down()) top_scale_ = std::clamp(top_scale_ * std::pow(0.88, amount), 0.5, 20.0);
    else if (camera_view()) camera_zoom_ = std::clamp(camera_zoom_ * std::pow(1.15, amount), 0.25, 4.0);
    else orbit_distance_ = std::clamp(orbit_distance_ * std::pow(0.88, amount), 0.5, 20.0);
  }

  void duplicate_zone() {
    const auto before = config_;
    auto copy = config_.zones[static_cast<std::size_t>(selected_)];
    const std::string base = copy.name + "_copy";
    copy.name = base;
    std::size_t suffix = 2;
    while (std::any_of(config_.zones.begin(), config_.zones.end(), [&](const auto& item) { return item.name == copy.name; }))
      copy.name = base + std::to_string(suffix++);
    for (auto& point : copy.floor_polygon) { point.x += 0.15; point.y += 0.15; }
    config_.zones.push_back(std::move(copy));
    selected_ = static_cast<int>(config_.zones.size() - 1);
    selected_vertex_ = 0;
    selected_top_ = false;
    remember(before);
  }

  void duplicate_ignore_plane() {
    const auto before = config_;
    auto copy = config_.ignore_planes[static_cast<std::size_t>(selected_plane_)];
    const std::string base = copy.name + "_copy";
    copy.name = base;
    std::size_t suffix = 2;
    while (std::any_of(config_.ignore_planes.begin(), config_.ignore_planes.end(),
                       [&](const auto& item) { return item.name == copy.name; }))
      copy.name = base + std::to_string(suffix++);
    for (auto& point : copy.corners_m) point.x += 0.15;
    config_.ignore_planes.push_back(std::move(copy));
    selected_plane_ = static_cast<int>(config_.ignore_planes.size() - 1);
    selected_plane_corner_ = -1;
    remember_ignore_planes(before);
  }

  void duplicate_selected() {
    if (selected_plane_ >= 0) duplicate_ignore_plane();
    else if (selected_ >= 0) duplicate_zone();
  }

  void delete_zone() {
    const auto before = config_;
    config_.zones.erase(config_.zones.begin() + selected_);
    selected_ = config_.zones.empty() ? -1 : std::min(selected_, static_cast<int>(config_.zones.size() - 1));
    selected_vertex_ = -1;
    remember(before);
  }

  void delete_selected() {
    if (selected_plane_ >= 0) {
      const auto before = config_;
      config_.ignore_planes.erase(config_.ignore_planes.begin() + selected_plane_);
      selected_plane_ = config_.ignore_planes.empty() ? -1 :
          std::min(selected_plane_, static_cast<int>(config_.ignore_planes.size() - 1));
      selected_plane_corner_ = -1;
      remember_ignore_planes(before);
    } else if (selected_ >= 0) delete_zone();
  }

  static bool inside(double x, double y, double left, double top, double width, double height) {
    return x >= left && x <= left + width && y >= top && y <= top + height;
  }

  void apply_rename() {
    if ((selected_ < 0 && selected_plane_ < 0) || rename_buffer_.empty()) return;
    const auto before = config_;
    if (selected_plane_ >= 0) config_.ignore_planes[static_cast<std::size_t>(selected_plane_)].name = rename_buffer_;
    else config_.zones[static_cast<std::size_t>(selected_)].name = rename_buffer_;
    renaming_ = false;
    if (selected_plane_ >= 0) remember_ignore_planes(before); else remember(before);
  }

  void rotate_selected_plane(Vec3 axis, double radians) {
    if (selected_plane_ < 0) return;
    const auto before = config_;
    auto& plane = config_.ignore_planes[static_cast<std::size_t>(selected_plane_)];
    axis = normalized(axis);
    Vec3 center{};
    for (const auto& point : plane.corners_m) center = center + Vec3{point.x, point.y, point.z};
    center = center * 0.25;
    const double cosine = std::cos(radians), sine = std::sin(radians);
    for (auto& point : plane.corners_m) {
      const Vec3 relative = Vec3{point.x, point.y, point.z} - center;
      const Vec3 rotated = relative * cosine + cross(axis, relative) * sine + axis * dot(axis, relative) * (1 - cosine);
      point = {center.x + rotated.x, center.y + rotated.y, center.z + rotated.z};
    }
    remember_ignore_planes(before);
  }

  void change_ignore_margin(double delta) {
    if (selected_plane_ < 0) return;
    const auto before = config_;
    auto& margin = config_.ignore_planes[static_cast<std::size_t>(selected_plane_)].margin_m;
    margin = std::clamp(margin + delta, 0.0, 1.0);
    remember_ignore_planes(before);
  }

  void change_ignore_noise_threshold(int delta) {
    if (selected_plane_ < 0) return;
    const auto before = config_;
    auto& threshold = config_.ignore_planes[static_cast<std::size_t>(selected_plane_)].noise_threshold_points;
    if (delta > 0) threshold = std::min<std::size_t>(1000000, threshold.value_or(0) + 10);
    else if (threshold && *threshold > 10) threshold = *threshold - 10;
    else threshold.reset();
    remember_ignore_planes(before);
  }

  void confirm_save() {
    try {
      app_config_.sensors[sensor_index_] = config_;
      specter::save_config_atomic(options_.config, app_config_);
      original_ = config_;
      status_ = "SAVED ATOMICALLY: " + options_.config.string();
    } catch (const std::exception& error) { status_ = std::string("SAVE FAILED: ") + error.what(); }
    save_preview_ = false;
  }

  void change_selected(const std::function<void(specter::ZoneConfig&)>& change) {
    if (selected_ < 0) return;
    const auto before = config_;
    change(config_.zones[static_cast<std::size_t>(selected_)]);
    remember(before);
  }

  void handle_panel_click(double x, double y, int width, int height) {
    const double left = width - panel_width + 16;
    if (save_preview_) {
      if (inside(x, y, 125, height - 175, 360, 35)) confirm_save();
      else if (inside(x, y, 510, height - 175, 180, 35)) save_preview_ = false;
      return;
    }
    if (renaming_) {
      if (inside(x, y, left, 316, 135, 30)) apply_rename();
      else if (inside(x, y, left + 145, 316, 135, 30)) renaming_ = false;
      return;
    }
    if (inside(x, y, left, 88, 94, 30)) view_mode_ = ViewMode::perspective;
    else if (inside(x, y, left + 100, 88, 94, 30)) view_mode_ = ViewMode::top_down;
    else if (inside(x, y, left + 200, 88, 94, 30)) {
      view_mode_ = ViewMode::camera;
      status_ = frame_ ? "EXACT KINECT DEPTH VIEW" : "CAMERA VIEW WAITING FOR DEPTH INTRINSICS";
    }
    else if (inside(x, y, left, 126, 142, 30)) { frozen_ = !frozen_; status_ = frozen_ ? "DEPTH FRAME FROZEN" : "LIVE DEPTH"; }
    else if (inside(x, y, left + 152, 126, 142, 30)) estimate_floor();
    else if (inside(x, y, left, 164, 142, 30)) live_validation_ = !live_validation_;
    else if (inside(x, y, left + 152, 164, 68, 30)) undo();
    else if (inside(x, y, left + 226, 164, 68, 30)) redo();
    else if (inside(x, y, left, 202, 68, 30)) {
      if (selected_plane_ >= 0 && !config_.ignore_planes.empty()) {
        selected_plane_ = selected_plane_ <= 0 ? static_cast<int>(config_.ignore_planes.size() - 1) : selected_plane_ - 1;
        selected_plane_corner_ = -1;
      } else if (!config_.zones.empty()) {
        selected_ = (selected_ <= 0) ? static_cast<int>(config_.zones.size() - 1) : selected_ - 1; selected_vertex_ = -1;
      }
    } else if (inside(x, y, left + 74, 202, 68, 30)) {
      if (selected_plane_ >= 0 && !config_.ignore_planes.empty()) {
        selected_plane_ = (selected_plane_ + 1) % static_cast<int>(config_.ignore_planes.size());
        selected_plane_corner_ = -1;
      } else if (!config_.zones.empty()) {
        selected_ = (selected_ + 1) % static_cast<int>(config_.zones.size()); selected_vertex_ = -1;
      }
    } else if (inside(x, y, left + 152, 202, 68, 30) && (selected_ >= 0 || selected_plane_ >= 0)) {
      renaming_ = true;
      rename_buffer_ = selected_plane_ >= 0 ? config_.ignore_planes[static_cast<std::size_t>(selected_plane_)].name :
                                              config_.zones[static_cast<std::size_t>(selected_)].name;
    } else if (inside(x, y, left + 226, 202, 68, 30) && (selected_ >= 0 || selected_plane_ >= 0)) duplicate_selected();
    else if (inside(x, y, left, 238, 294, 30) && (selected_ >= 0 || selected_plane_ >= 0)) delete_selected();
    else if (selected_plane_ >= 0 && inside(x, y, left, 388, 34, 26)) change_ignore_noise_threshold(-10);
    else if (selected_plane_ >= 0 && inside(x, y, left + 260, 388, 34, 26)) change_ignore_noise_threshold(10);
    else if (selected_plane_ >= 0 && inside(x, y, left, 426, 34, 26)) change_ignore_margin(-0.01);
    else if (selected_plane_ >= 0 && inside(x, y, left + 260, 426, 34, 26)) change_ignore_margin(0.01);
    else if (selected_plane_ >= 0 && inside(x, y, left, 464, 34, 26)) rotate_selected_plane({0, 0, 1}, -5 * pi / 180);
    else if (selected_plane_ >= 0 && inside(x, y, left + 260, 464, 34, 26)) rotate_selected_plane({0, 0, 1}, 5 * pi / 180);
    else if (selected_plane_ >= 0 && inside(x, y, left, 502, 34, 26)) {
      const auto& p = config_.ignore_planes[static_cast<std::size_t>(selected_plane_)];
      rotate_selected_plane(normalized(Vec3{p.corners_m[1].x - p.corners_m[0].x,
                                            p.corners_m[1].y - p.corners_m[0].y,
                                            p.corners_m[1].z - p.corners_m[0].z}), -5 * pi / 180);
    } else if (selected_plane_ >= 0 && inside(x, y, left + 260, 502, 34, 26)) {
      const auto& p = config_.ignore_planes[static_cast<std::size_t>(selected_plane_)];
      rotate_selected_plane(normalized(Vec3{p.corners_m[1].x - p.corners_m[0].x,
                                            p.corners_m[1].y - p.corners_m[0].y,
                                            p.corners_m[1].z - p.corners_m[0].z}), 5 * pi / 180);
    } else if (selected_plane_ >= 0 && inside(x, y, left, 540, 294, 26)) {
      const auto before = config_;
      auto& enabled = config_.ignore_planes[static_cast<std::size_t>(selected_plane_)].enabled;
      enabled = !enabled;
      remember_ignore_planes(before);
    }
    else if (inside(x, y, left, 388, 34, 26)) change_selected([](auto& z) { z.enter_points = std::max(z.exit_points, z.enter_points > 10 ? z.enter_points - 10 : z.exit_points); });
    else if (inside(x, y, left + 260, 388, 34, 26)) change_selected([](auto& z) { z.enter_points += 10; });
    else if (inside(x, y, left, 426, 34, 26)) change_selected([](auto& z) { z.exit_points = std::max<std::size_t>(1, z.exit_points > 10 ? z.exit_points - 10 : 1); });
    else if (inside(x, y, left + 260, 426, 34, 26)) change_selected([](auto& z) { z.exit_points = std::min(z.enter_points, z.exit_points + 10); });
    else if (inside(x, y, left, 464, 34, 26)) change_selected([](auto& z) { z.enter_after = std::max(std::chrono::milliseconds(0), z.enter_after - std::chrono::milliseconds(50)); });
    else if (inside(x, y, left + 260, 464, 34, 26)) change_selected([](auto& z) { z.enter_after += std::chrono::milliseconds(50); });
    else if (inside(x, y, left, 502, 34, 26)) change_selected([](auto& z) { z.exit_after = std::max(std::chrono::milliseconds(0), z.exit_after - std::chrono::milliseconds(100)); });
    else if (inside(x, y, left + 260, 502, 34, 26)) change_selected([](auto& z) { z.exit_after += std::chrono::milliseconds(100); });
    else if (inside(x, y, left, height - 168, 294, 30)) {
      object_tracking_ = !object_tracking_;
      pipeline_->set_tracking_enabled(object_tracking_);
      track_states_.clear();
      status_ = object_tracking_ ? "OBJECT TRACKING ENABLED" : "OBJECT TRACKING DISABLED";
    }
    else if (inside(x, y, left, height - 52, 294, 34)) validate_for_preview();
  }

  void estimate_floor() {
    if (camera_points_.size() < 50) { status_ = "CAPTURE A DEPTH FRAME BEFORE FLOOR ESTIMATION"; return; }
    std::mt19937 random(42);
    std::uniform_int_distribution<std::size_t> choose(0, camera_points_.size() - 1);
    Vec3 best_normal{};
    double best_d{};
    std::size_t best_count{};
    for (int iteration = 0; iteration < 500; ++iteration) {
      const Vec3 a = camera_points_[choose(random)];
      const Vec3 b = camera_points_[choose(random)];
      const Vec3 c = camera_points_[choose(random)];
      Vec3 normal = normalized(cross(b - a, c - a));
      if (length(normal) < 0.5 || std::abs(dot(normal, {0, -1, 0})) < 0.55) continue;
      if (dot(normal, {0, -1, 0}) < 0) normal = normal * -1.0;
      const double d = -dot(normal, a);
      std::size_t count{};
      for (std::size_t i = 0; i < camera_points_.size(); i += 3)
        if (std::abs(dot(normal, camera_points_[i]) + d) < 0.025) ++count;
      if (count > best_count) { best_count = count; best_normal = normal; best_d = d; }
    }
    if (best_count < 20) { status_ = "NO RELIABLE FLOOR PLANE FOUND - FREEZE A CLEAR FRAME"; return; }
    Vec3 x_axis = normalized(Vec3{1, 0, 0} - best_normal * dot({1, 0, 0}, best_normal));
    Vec3 y_axis = normalized(cross(best_normal, x_axis));
    const auto before = config_;
    config_.camera_to_room.matrix = {
        x_axis.x, x_axis.y, x_axis.z, 0,
        y_axis.x, y_axis.y, y_axis.z, 0,
        best_normal.x, best_normal.y, best_normal.z, best_d,
        0, 0, 0, 1};
    remember(before);
    std::ostringstream message;
    message << "FLOOR ESTIMATED FROM " << best_count << " INLIER SAMPLES - ALT KEYS TO CORRECT";
    status_ = message.str();
  }

  void validate_for_preview() {
    try {
      specter::validate_sensor(config_);
      save_preview_ = true;
      status_ = "SAVE PREVIEW VALID - ENTER CONFIRMS, ESC CANCELS";
    } catch (const std::exception& error) { status_ = std::string("CANNOT SAVE: ") + error.what(); }
  }

  void set_perspective(int width, int height) {
    const double aspect = static_cast<double>(width) / std::max(1, height);
    const double near_plane = 0.05, far_plane = 100.0;
    const double top = near_plane * std::tan(55.0 * pi / 360.0);
    glFrustum(-top * aspect, top * aspect, -top, top, near_plane, far_plane);
    glScaled(-1, 1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    const Vec3 target{target_x_, target_y_, target_z_};
    const double horizontal = orbit_distance_ * std::cos(orbit_pitch_);
    const Vec3 eye{
        target.x + horizontal * std::sin(orbit_yaw_),
        target.y - horizontal * std::cos(orbit_yaw_),
        target.z + orbit_distance_ * std::sin(orbit_pitch_)};
    const Vec3 forward = normalized(target - eye);
    const Vec3 right = normalized(cross(forward, {0, 0, 1}));
    const Vec3 up = cross(right, forward);
    const std::array<double, 16> view{
        right.x, up.x, -forward.x, 0,
        right.y, up.y, -forward.y, 0,
        right.z, up.z, -forward.z, 0,
        -dot(right, eye), -dot(up, eye), dot(forward, eye), 1};
    glMultMatrixd(view.data());
  }

  void set_camera_view() {
    if (!frame_) {
      glOrtho(-1, 1, -1, 1, -10, 10);
      glMatrixMode(GL_MODELVIEW); glLoadIdentity();
      return;
    }
    const auto& intrinsics = frame_->intrinsics;
    constexpr double near_plane = 0.05;
    const double far_plane = std::max(near_plane + 1.0, config_.processing.max_depth_m + 2.0);
    const double left = -intrinsics.cx * near_plane / intrinsics.fx / camera_zoom_;
    const double right = (static_cast<double>(frame_->width) - intrinsics.cx) * near_plane /
                         intrinsics.fx / camera_zoom_;
    const double top = intrinsics.cy * near_plane / intrinsics.fy / camera_zoom_;
    const double bottom = -(static_cast<double>(frame_->height) - intrinsics.cy) * near_plane /
                          intrinsics.fy / camera_zoom_;
    glFrustum(left, right, bottom, top, near_plane, far_plane);
    glScaled(-1, 1, 1);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    const auto inverse = specter::invert_transform(config_.camera_to_room).matrix;
    const std::array<double, 16> view{
        inverse[0], -inverse[4], -inverse[8], 0,
        inverse[1], -inverse[5], -inverse[9], 0,
        inverse[2], -inverse[6], -inverse[10], 0,
        inverse[3], -inverse[7], -inverse[11], 1};
    glMultMatrixd(view.data());
  }

  void render() {
    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);
    const int viewport_width = std::max(1, width - panel_width);
    glViewport(0, 0, width, height);
    glClearColor(0.025F, 0.03F, 0.045F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (camera_view()) {
      const auto viewport = camera_screen_viewport(width, height);
      glViewport(static_cast<int>(std::lround(viewport.x)),
                 static_cast<int>(std::lround(height - viewport.y - viewport.height)),
                 std::max(1, static_cast<int>(std::lround(viewport.width))),
                 std::max(1, static_cast<int>(std::lround(viewport.height))));
    } else glViewport(0, 0, viewport_width, height);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    if (camera_view()) {
      set_camera_view();
    } else if (top_down()) {
      const double aspect = static_cast<double>(viewport_width) / std::max(1, height);
      glOrtho(pan_x_ + top_scale_ * aspect, pan_x_ - top_scale_ * aspect,
              pan_y_ - top_scale_, pan_y_ + top_scale_, -10, 10);
      glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    } else {
      set_perspective(viewport_width, height);
    }
    draw_scene();
    draw_overlay(width, height);
  }

  void draw_scene() {
    glLineWidth(1.0F);
    glColor4f(0.18F, 0.22F, 0.28F, 1);
    glBegin(GL_LINES);
    for (int i = -10; i <= 10; ++i) {
      glVertex3d(i, -10, 0); glVertex3d(i, 10, 0);
      glVertex3d(-10, i, 0); glVertex3d(10, i, 0);
    }
    glEnd();
    if (!camera_view()) draw_physical_camera();
    glLineWidth(3.0F);
    glBegin(GL_LINES);
    glColor3f(1, 0.2F, 0.2F); glVertex3d(0, 0, 0); glVertex3d(1, 0, 0);
    glColor3f(0.2F, 1, 0.2F); glVertex3d(0, 0, 0); glVertex3d(0, 1, 0);
    glColor3f(0.2F, 0.5F, 1); glVertex3d(0, 0, 0); glVertex3d(0, 0, 1);
    glEnd();

    glPointSize(2.0F);
    glBegin(GL_POINTS);
    for (const auto point : room_points_) {
      const float normalized_height = static_cast<float>(std::clamp(point.z / 2.5, 0.0, 1.0));
      glColor3f(0.15F + 0.35F * normalized_height, 0.45F + 0.4F * normalized_height, 0.8F);
      glVertex3d(point.x, point.y, top_down() ? 0.01 : point.z);
    }
    glEnd();
    if (live_validation_) {
      glPointSize(4.0F); glColor3f(1, 0.25F, 0.1F); glBegin(GL_POINTS);
      for (const auto& point : foreground_points_) glVertex3d(point.x, point.y, top_down() ? 0.03 : point.z);
      glEnd();
      glPointSize(3.0F); glColor3f(0.9F, 0.15F, 0.75F); glBegin(GL_POINTS);
      for (const auto& point : ignored_points_) glVertex3d(point.x, point.y, top_down() ? 0.04 : point.z);
      glEnd();
    }
    for (std::size_t i = 0; i < config_.ignore_planes.size(); ++i)
      draw_ignore_plane(config_.ignore_planes[i], static_cast<int>(i) == selected_plane_);
    for (std::size_t i = 0; i < config_.zones.size(); ++i) draw_zone(config_.zones[i], static_cast<int>(i) == selected_);
    if (object_tracking_) draw_tracks();
    draw_gizmo();
  }

  void draw_physical_camera() {
    const auto room = [&](Vec3 camera) {
      const auto point = specter::transform_point(config_.camera_to_room, {camera.x, camera.y, camera.z});
      return Vec3{point.x, point.y, point.z};
    };
    const Vec3 origin = room({0, 0, 0});
    const std::array<Vec3, 8> body{
        room({-0.14, -0.045, -0.08}), room({0.14, -0.045, -0.08}),
        room({0.14, 0.045, -0.08}), room({-0.14, 0.045, -0.08}),
        room({-0.14, -0.045, 0.02}), room({0.14, -0.045, 0.02}),
        room({0.14, 0.045, 0.02}), room({-0.14, 0.045, 0.02})};
    glColor3f(0.82F, 0.86F, 0.92F); glLineWidth(3.0F);
    glBegin(GL_LINE_LOOP); for (std::size_t i = 0; i < 4; ++i) glVertex3d(body[i].x, body[i].y, body[i].z); glEnd();
    glBegin(GL_LINE_LOOP); for (std::size_t i = 4; i < 8; ++i) glVertex3d(body[i].x, body[i].y, body[i].z); glEnd();
    glBegin(GL_LINES);
    for (std::size_t i = 0; i < 4; ++i) {
      glVertex3d(body[i].x, body[i].y, body[i].z);
      glVertex3d(body[i + 4].x, body[i + 4].y, body[i + 4].z);
    }
    glEnd();
    const Vec3 x_axis = room({0.3, 0, 0});
    const Vec3 y_axis = room({0, 0.3, 0});
    const Vec3 forward = room({0, 0, 0.5});
    glBegin(GL_LINES);
    glColor3f(1, 0.2F, 0.2F); glVertex3d(origin.x, origin.y, origin.z); glVertex3d(x_axis.x, x_axis.y, x_axis.z);
    glColor3f(0.2F, 1, 0.2F); glVertex3d(origin.x, origin.y, origin.z); glVertex3d(y_axis.x, y_axis.y, y_axis.z);
    glColor3f(0.2F, 0.55F, 1); glVertex3d(origin.x, origin.y, origin.z); glVertex3d(forward.x, forward.y, forward.z);
    glEnd();
    if (!frame_) return;
    const double depth = std::min(1.25, config_.processing.max_depth_m);
    const std::array<specter::Point2, 4> pixels{{
        {0, 0}, {static_cast<double>(frame_->width), 0},
        {static_cast<double>(frame_->width), static_cast<double>(frame_->height)},
        {0, static_cast<double>(frame_->height)}}};
    std::array<Vec3, 4> frustum;
    for (std::size_t i = 0; i < pixels.size(); ++i) {
      const auto camera = specter::deproject_depth(frame_->intrinsics, pixels[i].x, pixels[i].y, depth);
      frustum[i] = room({camera.x, camera.y, camera.z});
    }
    glColor4f(0.35F, 0.72F, 1.0F, 0.75F); glLineWidth(1.5F);
    glBegin(GL_LINES);
    for (const auto& corner : frustum) {
      glVertex3d(origin.x, origin.y, origin.z); glVertex3d(corner.x, corner.y, corner.z);
    }
    glEnd();
    glBegin(GL_LINE_LOOP); for (const auto& corner : frustum) glVertex3d(corner.x, corner.y, corner.z); glEnd();
  }

  void draw_ignore_plane(const specter::IgnorePlaneConfig& plane, bool selected) {
    const float red = plane.enabled ? 1.0F : 0.45F;
    const float green = selected ? 0.42F : 0.18F;
    const float blue = plane.enabled ? 0.08F : 0.45F;
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(red, green, blue, plane.enabled ? 0.22F : 0.10F);
    glBegin(GL_QUADS);
    for (const auto& point : plane.corners_m) glVertex3d(point.x, point.y, point.z);
    glEnd();
    glColor4f(red, selected ? 0.75F : 0.4F, blue, 0.95F);
    glLineWidth(selected ? 4.0F : 2.0F);
    glBegin(GL_LINE_LOOP);
    for (const auto& point : plane.corners_m) glVertex3d(point.x, point.y, point.z);
    glEnd();
    if (camera_view() && plane.margin_m > 0) {
      const Vec3 u = normalized(Vec3{plane.corners_m[1].x - plane.corners_m[0].x,
                                     plane.corners_m[1].y - plane.corners_m[0].y,
                                     plane.corners_m[1].z - plane.corners_m[0].z});
      const Vec3 v = normalized(Vec3{plane.corners_m[3].x - plane.corners_m[0].x,
                                     plane.corners_m[3].y - plane.corners_m[0].y,
                                     plane.corners_m[3].z - plane.corners_m[0].z});
      const auto as_vec = [](const specter::Point3& point) { return Vec3{point.x, point.y, point.z}; };
      const std::array<Vec3, 4> expanded{
          as_vec(plane.corners_m[0]) - u * plane.margin_m - v * plane.margin_m,
          as_vec(plane.corners_m[1]) + u * plane.margin_m - v * plane.margin_m,
          as_vec(plane.corners_m[2]) + u * plane.margin_m + v * plane.margin_m,
          as_vec(plane.corners_m[3]) - u * plane.margin_m + v * plane.margin_m};
      glColor4f(1.0F, 0.85F, 0.15F, 0.95F); glLineWidth(2.0F);
      glBegin(GL_LINE_LOOP); for (const auto& point : expanded) glVertex3d(point.x, point.y, point.z); glEnd();
    }
    if (selected) {
      glDisable(GL_DEPTH_TEST);
      glPointSize(10.0F); glBegin(GL_POINTS);
      for (const auto& point : plane.corners_m) glVertex3d(point.x, point.y, point.z);
      glEnd();
      Vec3 center{};
      for (const auto& point : plane.corners_m) center = center + Vec3{point.x, point.y, point.z};
      center = center * 0.25;
      const Vec3 u{plane.corners_m[1].x - plane.corners_m[0].x,
                   plane.corners_m[1].y - plane.corners_m[0].y,
                   plane.corners_m[1].z - plane.corners_m[0].z};
      const Vec3 v{plane.corners_m[3].x - plane.corners_m[0].x,
                   plane.corners_m[3].y - plane.corners_m[0].y,
                   plane.corners_m[3].z - plane.corners_m[0].z};
      const Vec3 normal = normalized(cross(u, v));
      glLineWidth(3.0F); glBegin(GL_LINES);
      glVertex3d(center.x, center.y, center.z);
      glVertex3d(center.x + normal.x * 0.35, center.y + normal.y * 0.35, center.z + normal.z * 0.35);
      glEnd();
      glEnable(GL_DEPTH_TEST);
    }
    glDisable(GL_BLEND);
  }

  void draw_tracks() {
    for (const auto& track : track_states_) {
      if (track.classification == "likely_human") glColor3f(0.15F, 1.0F, 0.8F);
      else if (track.classification == "likely_animal") glColor3f(0.9F, 0.55F, 1.0F);
      else if (track.classification == "likely_object") glColor3f(1.0F, 0.75F, 0.2F);
      else glColor3f(0.75F, 0.78F, 0.82F);
      const double min_x = track.centroid_m.x - track.bounds_m.x * 0.5;
      const double max_x = track.centroid_m.x + track.bounds_m.x * 0.5;
      const double min_y = track.centroid_m.y - track.bounds_m.y * 0.5;
      const double max_y = track.centroid_m.y + track.bounds_m.y * 0.5;
      const double min_z = track.centroid_m.z - track.bounds_m.z * 0.5;
      const double max_z = track.centroid_m.z + track.bounds_m.z * 0.5;
      glLineWidth(track.occluded ? 1.5F : 3.0F);
      if (top_down()) {
        glBegin(GL_LINE_LOOP);
        glVertex3d(min_x, min_y, 0.09); glVertex3d(max_x, min_y, 0.09);
        glVertex3d(max_x, max_y, 0.09); glVertex3d(min_x, max_y, 0.09);
        glEnd();
        continue;
      }
      glBegin(GL_LINE_LOOP);
      glVertex3d(min_x, min_y, min_z); glVertex3d(max_x, min_y, min_z);
      glVertex3d(max_x, max_y, min_z); glVertex3d(min_x, max_y, min_z);
      glEnd();
      glBegin(GL_LINE_LOOP);
      glVertex3d(min_x, min_y, max_z); glVertex3d(max_x, min_y, max_z);
      glVertex3d(max_x, max_y, max_z); glVertex3d(min_x, max_y, max_z);
      glEnd();
      glBegin(GL_LINES);
      glVertex3d(min_x, min_y, min_z); glVertex3d(min_x, min_y, max_z);
      glVertex3d(max_x, min_y, min_z); glVertex3d(max_x, min_y, max_z);
      glVertex3d(max_x, max_y, min_z); glVertex3d(max_x, max_y, max_z);
      glVertex3d(min_x, max_y, min_z); glVertex3d(min_x, max_y, max_z);
      glEnd();
    }
  }

  void draw_zone(const specter::ZoneConfig& zone, bool selected) {
    const auto& polygon = zone.floor_polygon;
    const float r = selected ? 1.0F : 0.15F, g = selected ? 0.55F : 0.75F, b = selected ? 0.1F : 0.95F;
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    if (!top_down()) {
      glColor4f(r, g, b, 0.12F); glBegin(GL_QUADS);
      for (std::size_t i = 0; i < polygon.size(); ++i) {
        const auto& a = polygon[i]; const auto& c = polygon[(i + 1) % polygon.size()];
        glVertex3d(a.x, a.y, zone.min_height_m); glVertex3d(c.x, c.y, zone.min_height_m);
        glVertex3d(c.x, c.y, zone.max_height_m); glVertex3d(a.x, a.y, zone.max_height_m);
      }
      glEnd();
    }
    glColor4f(r, g, b, 0.95F); glLineWidth(selected ? 4.0F : 2.0F);
    const double low = top_down() ? 0.05 : zone.min_height_m;
    const double high = top_down() ? 0.05 : zone.max_height_m;
    glBegin(GL_LINE_LOOP); for (const auto point : polygon) glVertex3d(point.x, point.y, low); glEnd();
    if (!top_down()) {
      glBegin(GL_LINE_LOOP); for (const auto point : polygon) glVertex3d(point.x, point.y, high); glEnd();
      glBegin(GL_LINES); for (const auto point : polygon) { glVertex3d(point.x, point.y, low); glVertex3d(point.x, point.y, high); } glEnd();
    }
    glDisable(GL_DEPTH_TEST);
    if (top_down() && selected) {
      glPointSize(9); glBegin(GL_POINTS); for (const auto point : polygon) glVertex3d(point.x, point.y, 0.08); glEnd();
    } else if (!top_down()) {
      glPointSize(selected ? 9.0F : 6.0F); glBegin(GL_POINTS);
      for (const auto point : polygon) {
        glVertex3d(point.x, point.y, zone.min_height_m);
        glVertex3d(point.x, point.y, zone.max_height_m);
      }
      glEnd();
    }
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
  }

  void draw_gizmo() {
    if (selected_plane_ < 0 && (selected_ < 0 || selected_vertex_ < 0)) return;
    const auto origin = selected_point();
    glDisable(GL_DEPTH_TEST);
    glLineWidth(5.0F);
    glBegin(GL_LINES);
    glColor3f(gizmo_drag_ == GizmoAxis::x ? 1.0F : 0.85F, 0.12F, 0.12F);
    glVertex3d(origin.x, origin.y, origin.z); glVertex3d(origin.x + 0.45, origin.y, origin.z);
    glColor3f(0.12F, gizmo_drag_ == GizmoAxis::y ? 1.0F : 0.85F, 0.15F);
    glVertex3d(origin.x, origin.y, origin.z); glVertex3d(origin.x, origin.y + 0.45, origin.z);
    if (!top_down()) {
      glColor3f(0.12F, 0.4F, gizmo_drag_ == GizmoAxis::z ? 1.0F : 0.95F);
      glVertex3d(origin.x, origin.y, origin.z); glVertex3d(origin.x, origin.y, origin.z + 0.45);
    }
    glEnd();
    glPointSize(11.0F); glBegin(GL_POINTS);
    glColor3f(1, 0.15F, 0.15F); glVertex3d(origin.x + 0.45, origin.y, origin.z);
    glColor3f(0.15F, 1, 0.2F); glVertex3d(origin.x, origin.y + 0.45, origin.z);
    if (!top_down()) { glColor3f(0.15F, 0.45F, 1); glVertex3d(origin.x, origin.y, origin.z + 0.45); }
    glEnd();
    glEnable(GL_DEPTH_TEST);
  }

  void draw_overlay(int width, int height) {
    glViewport(0, 0, width, height);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, width, height, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    if (camera_view()) {
      const auto viewport = camera_screen_viewport(width, height);
      glColor3f(0.25F, 0.8F, 1.0F);
      glLineWidth(2.0F);
      glBegin(GL_LINE_LOOP);
      glVertex2d(viewport.x + 1, viewport.y + 1);
      glVertex2d(viewport.x + viewport.width - 1, viewport.y + 1);
      glVertex2d(viewport.x + viewport.width - 1, viewport.y + viewport.height - 1);
      glVertex2d(viewport.x + 1, viewport.y + viewport.height - 1);
      glEnd();
      draw_text(viewport.x + 10, viewport.y + 10,
                "MIRRORED SENSOR FOV  " + short_number(camera_zoom_) + "X", 1.05);
    }
    if (top_down()) draw_topdown_labels(width, height);
    if (!camera_view()) draw_camera_label();
    draw_ignore_plane_labels();
    if (object_tracking_) draw_track_labels();
    glColor4f(0.055F, 0.065F, 0.085F, 0.98F);
    glBegin(GL_QUADS); glVertex2d(width - panel_width, 0); glVertex2d(width, 0); glVertex2d(width, height); glVertex2d(width - panel_width, height); glEnd();
    const double x = width - panel_width + 16;
    glColor3f(0.75F, 0.9F, 1.0F); draw_text(x, 16, "SPECTER-SENSE", 2.2);
    glColor3f(0.8F, 0.82F, 0.86F); draw_text(x, 48, status_.substr(0, 42), 1.15);
    draw_button(x, 88, 94, 30, "3D", view_mode_ == ViewMode::perspective);
    draw_button(x + 100, 88, 94, 30, "TOP", view_mode_ == ViewMode::top_down);
    draw_button(x + 200, 88, 94, 30, "CAMERA", view_mode_ == ViewMode::camera);
    draw_button(x, 126, 142, 30, frozen_ ? "RESUME LIVE" : "FREEZE FRAME", frozen_);
    draw_button(x + 152, 126, 142, 30, "ESTIMATE FLOOR", false);
    draw_button(x, 164, 142, 30, live_validation_ ? "VALIDATION ON" : "VALIDATION OFF", live_validation_);
    draw_button(x + 152, 164, 68, 30, "UNDO", false);
    draw_button(x + 226, 164, 68, 30, "REDO", false);
    draw_button(x, 202, 68, 30, "PREV", false);
    draw_button(x + 74, 202, 68, 30, "NEXT", false);
    draw_button(x + 152, 202, 68, 30, "RENAME", renaming_);
    draw_button(x + 226, 202, 68, 30, "COPY", false);
    draw_button(x, 238, 294, 30, selected_plane_ >= 0 ? "DELETE SELECTED PLANE" : "DELETE SELECTED ZONE", false, true);
    if (renaming_) {
      glColor3f(1, 0.82F, 0.25F); draw_text(x, 280, "NAME: " + rename_buffer_ + "_", 1.4);
      draw_button(x, 316, 135, 30, "APPLY NAME", false);
      draw_button(x + 145, 316, 135, 30, "CANCEL", false);
    }
    double y = renaming_ ? 360 : 284;
    if (selected_plane_ >= 0 && selected_plane_ < static_cast<int>(config_.ignore_planes.size())) {
      const auto& plane = config_.ignore_planes[static_cast<std::size_t>(selected_plane_)];
      glColor3f(1, 0.35F, 0.12F); draw_text(x, y, "IGNORE: " + plane.name, 1.5); y += 22;
      glColor3f(0.78F, 0.82F, 0.86F);
      draw_text(x, y, std::string("STATE: ") + (plane.enabled ? "ENABLED" : "DISABLED"), 1.2); y += 20;
      if (static_cast<std::size_t>(selected_plane_) < ignore_plane_states_.size()) {
        const auto& state = ignore_plane_states_[static_cast<std::size_t>(selected_plane_)];
        draw_text(x, y, (plane.noise_threshold_points ? "ACTIVITY: " : "RAW MATCHES: ") +
            std::to_string(plane.noise_threshold_points ? state.activity_points : state.matched_points) + " PTS", 1.2); y += 20;
        draw_text(x, y, "REJECTED: " + std::to_string(state.rejected_points) + " PTS", 1.2); y += 20;
        if (plane.noise_threshold_points) {
          const bool passing = state.activity_points > *plane.noise_threshold_points;
          glColor3f(passing ? 1.0F : 0.35F, passing ? 0.65F : 0.85F, 0.2F);
          draw_text(x, y, passing ? "SENSITIVITY: ACTIVITY PASSING" : "SENSITIVITY: NOISE SUPPRESSED", 1.05);
          y += 20;
        }
      }
      draw_stepper(x, 388, "NOISE LIMIT", plane.noise_threshold_points
          ? std::to_string(*plane.noise_threshold_points) + " PTS" : "IGNORE ALL");
      draw_stepper(x, 426, "MARGIN", short_number(plane.margin_m) + " M");
      draw_stepper(x, 464, "YAW", "5 DEG");
      draw_stepper(x, 502, "PITCH", "5 DEG");
      draw_button(x, 540, 294, 26, plane.enabled ? "DISABLE PLANE" : "ENABLE PLANE", plane.enabled);
    } else if (selected_ >= 0 && selected_ < static_cast<int>(config_.zones.size())) {
      const auto& zone = config_.zones[static_cast<std::size_t>(selected_)];
      glColor3f(1, 0.65F, 0.2F); draw_text(x, y, "ZONE: " + zone.name, 1.6); y += 22;
      glColor3f(0.78F, 0.82F, 0.86F);
      draw_text(x, y, "HEIGHT: " + short_number(zone.min_height_m) + " TO " + short_number(zone.max_height_m) + " M", 1.25); y += 20;
      if (selected_vertex_ >= 0) {
        draw_text(x, y, std::string("CORNER: ") + (selected_top_ ? "TOP " : "BOTTOM ") + std::to_string(selected_vertex_ + 1), 1.25); y += 20;
      }
      if (live_validation_ && static_cast<std::size_t>(selected_) < zone_states_.size()) {
        const auto& state = zone_states_[static_cast<std::size_t>(selected_)];
        glColor3f(state.occupied ? 0.2F : 0.75F, state.occupied ? 1.0F : 0.75F, 0.25F);
        draw_text(x, y, (state.occupied ? "OCCUPIED  " : "CLEAR  ") + std::to_string(state.foreground_points) + " PTS", 1.3); y += 20;
      }
      draw_stepper(x, 388, "ENTER POINTS", std::to_string(zone.enter_points));
      draw_stepper(x, 426, "EXIT POINTS", std::to_string(zone.exit_points));
      draw_stepper(x, 464, "ENTER DELAY", std::to_string(zone.enter_after.count()) + " MS");
      draw_stepper(x, 502, "EXIT DELAY", std::to_string(zone.exit_after.count()) + " MS");
    } else { glColor3f(0.7F, 0.7F, 0.72F); draw_text(x, y, "NO ZONE SELECTED", 1.5); }
    glColor3f(0.62F, 0.68F, 0.75F);
    draw_button(x, height - 168, 294, 30,
                object_tracking_ ? "OBJECT TRACKING ON  " + std::to_string(track_states_.size())
                                 : "OBJECT TRACKING OFF",
                object_tracking_);
    draw_text(x, height - 128, "CTRL+N BOX  CTRL+M IGNORE PLANE", 1.05);
    draw_text(x, height - 106, "CLICK CORNER, DRAG X/Y/Z", 1.25);
    draw_text(x, height - 86, camera_view() ? "DRAG CORNERS  WHEEL ZOOM" :
                                             "LEFT ORBIT  RIGHT PAN  WHEEL ZOOM", 1.05);
    draw_button(x, height - 52, 294, 34, "REVIEW AND SAVE", false);
    if (save_preview_) draw_save_preview(width, height);
  }

  void draw_button(double x, double y, double width, double height, const std::string& label,
                   bool active, bool danger = false) {
    if (danger) glColor3f(0.38F, 0.1F, 0.12F);
    else if (active) glColor3f(0.12F, 0.38F, 0.5F);
    else glColor3f(0.13F, 0.16F, 0.21F);
    glBegin(GL_QUADS); glVertex2d(x, y); glVertex2d(x + width, y); glVertex2d(x + width, y + height); glVertex2d(x, y + height); glEnd();
    glColor3f(danger ? 1.0F : 0.78F, danger ? 0.55F : 0.84F, danger ? 0.55F : 0.9F);
    const double scale = width < 90 ? 1.0 : 1.15;
    draw_text(x + 8, y + (height - 7 * scale) * 0.5, label, scale);
  }

  void draw_stepper(double x, double y, const std::string& label, const std::string& value) {
    draw_button(x, y, 34, 26, "-", false);
    glColor3f(0.75F, 0.8F, 0.86F); draw_text(x + 46, y + 2, label, 1.05); draw_text(x + 150, y + 2, value, 1.05);
    draw_button(x + 260, y, 34, 26, "+", false);
  }

  void draw_topdown_labels(int width, int height) {
    const double viewport_width = std::max(1, width - panel_width);
    const double aspect = viewport_width / std::max(1.0, static_cast<double>(height));
    for (std::size_t index = 0; index < config_.zones.size(); ++index) {
      const auto& zone = config_.zones[index];
      specter::Point2 center{};
      for (const auto point : zone.floor_polygon) { center.x += point.x; center.y += point.y; }
      center.x /= static_cast<double>(zone.floor_polygon.size());
      center.y /= static_cast<double>(zone.floor_polygon.size());
      const double screen_x = viewport_width - ((center.x - pan_x_) / (top_scale_ * aspect) + 1.0) * 0.5 * viewport_width;
      const double screen_y = (1.0 - (center.y - pan_y_) / top_scale_) * 0.5 * height;
      glColor3f(static_cast<int>(index) == selected_ ? 1.0F : 0.65F,
                static_cast<int>(index) == selected_ ? 0.7F : 0.85F, 0.25F);
      draw_text(screen_x - static_cast<double>(zone.name.size()) * 3.5, screen_y, zone.name, 1.2);
    }
  }

  void draw_ignore_plane_labels() {
    for (std::size_t index = 0; index < config_.ignore_planes.size(); ++index) {
      const auto& plane = config_.ignore_planes[index];
      Vec3 center{};
      for (const auto& point : plane.corners_m) center = center + Vec3{point.x, point.y, point.z};
      center = center * 0.25;
      const auto screen = project(center);
      if (!screen) continue;
      glColor3f(static_cast<int>(index) == selected_plane_ ? 1.0F : 0.85F,
                plane.enabled ? 0.35F : 0.55F, plane.enabled ? 0.1F : 0.55F);
      draw_text(screen->x + 8, screen->y + 8, "IGNORE " + plane.name, 1.15);
    }
  }

  void draw_camera_label() {
    const auto origin = specter::transform_point(config_.camera_to_room, {0, 0, 0});
    const auto screen = project({origin.x, origin.y, origin.z});
    if (!screen) return;
    glColor3f(0.45F, 0.8F, 1.0F);
    draw_text(screen->x + 10, screen->y - 10, "KINECT DEPTH CAMERA", 1.15);
  }

  void draw_track_labels() {
    for (const auto& track : track_states_) {
      const auto screen = project({track.centroid_m.x, track.centroid_m.y,
                                   top_down() ? 0.1 : track.centroid_m.z + track.bounds_m.z * 0.55});
      if (!screen) continue;
      if (track.classification == "likely_human") glColor3f(0.15F, 1.0F, 0.8F);
      else if (track.classification == "likely_animal") glColor3f(0.9F, 0.55F, 1.0F);
      else if (track.classification == "likely_object") glColor3f(1.0F, 0.75F, 0.2F);
      else glColor3f(0.8F, 0.82F, 0.86F);
      draw_text(screen->x + 8, screen->y - 8,
                track.id + " " + track.classification + " " + track.posture,
                1.15);
    }
  }

  static std::string short_number(double value) {
    std::ostringstream out; out.setf(std::ios::fixed); out.precision(2); out << value; return out.str();
  }

  void draw_save_preview(int width, int height) {
    glColor4f(0.02F, 0.025F, 0.035F, 0.96F); glBegin(GL_QUADS);
    glVertex2d(90, 90); glVertex2d(width - 90, 90); glVertex2d(width - 90, height - 90); glVertex2d(90, height - 90); glEnd();
    glColor3f(1, 0.75F, 0.2F); draw_text(125, 125, "CONFIGURATION SAVE PREVIEW", 2.2);
    glColor3f(0.8F, 0.85F, 0.9F);
    draw_text(125, 175, "FILE: " + options_.config.string(), 1.5);
    draw_text(125, 205, "ZONES: " + std::to_string(original_.zones.size()) + " -> " + std::to_string(config_.zones.size()), 1.5);
    draw_text(125, 235, "IGNORE PLANES: " + std::to_string(original_.ignore_planes.size()) + " -> " +
        std::to_string(config_.ignore_planes.size()), 1.5);
    draw_text(125, 265, original_.camera_to_room.matrix == config_.camera_to_room.matrix
        ? "ROOM TRANSFORM: UNCHANGED" : "ROOM TRANSFORM: CHANGED", 1.5);
    draw_text(125, 295, specter::serialize_config(specter::AppConfig{{original_}}) ==
        specter::serialize_config(specter::AppConfig{{config_}})
        ? "CONTENT: NO CHANGES" : "CONTENT: MODIFIED", 1.5);
    draw_button(125, height - 175, 360, 35, "ATOMICALLY REPLACE CONFIG", true);
    draw_button(510, height - 175, 180, 35, "CANCEL", false, true);
  }

  Options options_;
  specter::AppConfig app_config_;
  std::size_t sensor_index_{};
  specter::SensorConfig config_;
  specter::SensorConfig original_;
  std::unique_ptr<specter::OccupancyPipeline> pipeline_;
  std::unique_ptr<specter::FrameSource> source_;
  GLFWwindow* window_{};
  std::optional<specter::DepthFrame> frame_;
  std::vector<Vec3> camera_points_;
  std::vector<Vec3> room_points_;
  std::vector<specter::Point3> foreground_points_;
  std::vector<specter::ZoneState> zone_states_;
  std::vector<specter::TrackState> track_states_;
  std::vector<specter::IgnorePlaneState> ignore_plane_states_;
  std::vector<specter::Point3> ignored_points_;
  std::vector<specter::SensorConfig> undo_, redo_;
  specter::SensorConfig edit_before_;
  std::string status_{"STARTING"};
  std::string rename_buffer_;
  bool frozen_{}, left_down_{}, right_down_{}, translating_{}, corner_click_{}, renaming_{}, save_preview_{};
  bool live_validation_{};
  bool object_tracking_{};
  bool editing_plane_{}, plane_corner_dragged_{};
  ViewMode view_mode_{ViewMode::perspective};
  int selected_{-1}, selected_vertex_{-1};
  int selected_plane_{-1}, selected_plane_corner_{-1};
  bool selected_top_{};
  GizmoAxis gizmo_drag_{GizmoAxis::none};
  double mouse_x_{}, mouse_y_{};
  double orbit_yaw_{0.6}, orbit_pitch_{0.45}, orbit_distance_{4.0};
  double camera_zoom_{1.0};
  double target_x_{}, target_y_{1.5}, target_z_{0.7};
  double top_scale_{3.0}, pan_x_{}, pan_y_{1.5};
};

}  // namespace

int main(int argc, char** argv) {
  try {
    return CalibrationApp(parse_options(argc, argv)).run();
  } catch (const std::exception& error) {
    std::cerr << "specter-sense-calibrate: " << error.what() << '\n';
    return 1;
  }
}
