#include "specter_sense/config.hpp"
#include "specter_sense/frame_source.hpp"
#include "specter_sense/pipeline.hpp"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
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
  std::string source{"kinect"};
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
    else if (arg == "--source") options.source = value();
    else if (arg == "--help") {
      std::cout << "Usage: specter-sense-calibrate [--config PATH] [--source kinect|synthetic]\n";
      std::exit(0);
    } else throw std::runtime_error("unknown argument: " + arg);
  }
  return options;
}

class CalibrationApp {
 public:
  explicit CalibrationApp(Options options)
      : options_(std::move(options)), config_(specter::load_config(options_.config)),
        original_(config_), pipeline_(std::make_unique<specter::OccupancyPipeline>(config_, true)) {}

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
  enum class GizmoAxis { none, x, y, z };

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
    if (options_.source == "synthetic") source_ = specter::make_synthetic_source();
#ifdef SPECTER_SENSE_HAS_KINECT
    else if (options_.source == "kinect") source_ = specter::make_kinect_source();
#else
    else if (options_.source == "kinect") throw std::runtime_error("calibrator built without Kinect support");
#endif
    else throw std::runtime_error("unknown source: " + options_.source);
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
    } catch (const std::exception& error) {
      status_ = std::string("PIPELINE: ") + error.what();
    }
  }

  void rebuild_pipeline() {
    try {
      specter::validate_config(config_);
      pipeline_ = std::make_unique<specter::OccupancyPipeline>(config_, true);
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

  void remember(const specter::AppConfig& before) {
    undo_.push_back(before);
    if (undo_.size() > 100) undo_.erase(undo_.begin());
    redo_.clear();
    rebuild_pipeline();
  }

  void undo() {
    if (undo_.empty()) return;
    redo_.push_back(config_);
    config_ = undo_.back();
    undo_.pop_back();
    selected_ = config_.zones.empty() ? -1 : std::min(selected_, static_cast<int>(config_.zones.size() - 1));
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
    selected_vertex_ = 0;
    selected_top_ = false;
    remember(before);
    status_ = "BOUNDING BOX CREATED - CLICK A CORNER OR GIZMO AXIS";
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
    if (top_down_) {
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
          if (top_down_ && is_top) continue;
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
    status_ = "CORNER SELECTED - DRAG A COLORED AXIS";
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
    if (selected_ < 0 || selected_vertex_ < 0) return GizmoAxis::none;
    const Vec3 origin = selected_point();
    const auto start = project(origin);
    if (!start) return GizmoAxis::none;
    if (std::hypot(start->x - x, start->y - y) < 14.0) return GizmoAxis::none;
    const std::array<std::pair<GizmoAxis, Vec3>, 3> axes{{
        {GizmoAxis::x, {0.45, 0, 0}}, {GizmoAxis::y, {0, 0.45, 0}}, {GizmoAxis::z, {0, 0, 0.45}}}};
    GizmoAxis best_axis = GizmoAxis::none;
    double best = 9.0;
    for (const auto& [axis, delta] : axes) {
      if (top_down_ && axis == GizmoAxis::z) continue;
      const auto end = project(origin + delta);
      if (!end) continue;
      const double distance = distance_to_segment(x, y, *start, *end);
      if (distance < best) { best = distance; best_axis = axis; }
    }
    return best_axis;
  }

  void on_cursor(double x, double y) {
    const double dx = x - mouse_x_;
    const double dy = y - mouse_y_;
    mouse_x_ = x; mouse_y_ = y;
    int width, height;
    glfwGetWindowSize(window_, &width, &height);
    if (gizmo_drag_ != GizmoAxis::none && selected_ >= 0 && selected_vertex_ >= 0) {
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
      rebuild_pipeline();
      return;
    }
    if (right_down_) {
      if (top_down_) {
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
    if (corner_click_) return;
    if (top_down_ && translating_ && selected_ >= 0) {
      const auto current = screen_to_floor(x, y);
      const auto previous = screen_to_floor(x - dx, y - dy);
      for (auto& point : config_.zones[static_cast<std::size_t>(selected_)].floor_polygon) {
        point.x += current.x - previous.x; point.y += current.y - previous.y;
      }
      rebuild_pipeline();
    } else if (!top_down_) {
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
      if (gizmo_drag_ != GizmoAxis::none || translating_) remember(edit_before_);
      gizmo_drag_ = GizmoAxis::none; translating_ = false; corner_click_ = false;
      return;
    }
    const auto axis = pick_gizmo(mouse_x_, mouse_y_);
    if (axis != GizmoAxis::none) {
      edit_before_ = config_; gizmo_drag_ = axis; return;
    }
    if (pick_corner(mouse_x_, mouse_y_)) { corner_click_ = true; return; }
    if (!top_down_) return;
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
    if (top_down_) top_scale_ = std::clamp(top_scale_ * std::pow(0.88, amount), 0.5, 20.0);
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

  void delete_zone() {
    const auto before = config_;
    config_.zones.erase(config_.zones.begin() + selected_);
    selected_ = config_.zones.empty() ? -1 : std::min(selected_, static_cast<int>(config_.zones.size() - 1));
    selected_vertex_ = -1;
    remember(before);
  }

  static bool inside(double x, double y, double left, double top, double width, double height) {
    return x >= left && x <= left + width && y >= top && y <= top + height;
  }

  void apply_rename() {
    if (selected_ < 0 || rename_buffer_.empty()) return;
    const auto before = config_;
    config_.zones[static_cast<std::size_t>(selected_)].name = rename_buffer_;
    renaming_ = false;
    remember(before);
  }

  void confirm_save() {
    try {
      specter::save_config_atomic(options_.config, config_);
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
    if (inside(x, y, left, 88, 142, 30)) top_down_ = false;
    else if (inside(x, y, left + 152, 88, 142, 30)) top_down_ = true;
    else if (inside(x, y, left, 126, 142, 30)) { frozen_ = !frozen_; status_ = frozen_ ? "DEPTH FRAME FROZEN" : "LIVE DEPTH"; }
    else if (inside(x, y, left + 152, 126, 142, 30)) estimate_floor();
    else if (inside(x, y, left, 164, 142, 30)) live_validation_ = !live_validation_;
    else if (inside(x, y, left + 152, 164, 68, 30)) undo();
    else if (inside(x, y, left + 226, 164, 68, 30)) redo();
    else if (inside(x, y, left, 202, 68, 30) && !config_.zones.empty()) {
      selected_ = (selected_ <= 0) ? static_cast<int>(config_.zones.size() - 1) : selected_ - 1; selected_vertex_ = -1;
    } else if (inside(x, y, left + 74, 202, 68, 30) && !config_.zones.empty()) {
      selected_ = (selected_ + 1) % static_cast<int>(config_.zones.size()); selected_vertex_ = -1;
    } else if (inside(x, y, left + 152, 202, 68, 30) && selected_ >= 0) {
      renaming_ = true; rename_buffer_ = config_.zones[static_cast<std::size_t>(selected_)].name;
    } else if (inside(x, y, left + 226, 202, 68, 30) && selected_ >= 0) duplicate_zone();
    else if (inside(x, y, left, 238, 294, 30) && selected_ >= 0) delete_zone();
    else if (inside(x, y, left, 388, 34, 26)) change_selected([](auto& z) { z.enter_points = std::max(z.exit_points, z.enter_points > 10 ? z.enter_points - 10 : z.exit_points); });
    else if (inside(x, y, left + 260, 388, 34, 26)) change_selected([](auto& z) { z.enter_points += 10; });
    else if (inside(x, y, left, 426, 34, 26)) change_selected([](auto& z) { z.exit_points = std::max<std::size_t>(1, z.exit_points > 10 ? z.exit_points - 10 : 1); });
    else if (inside(x, y, left + 260, 426, 34, 26)) change_selected([](auto& z) { z.exit_points = std::min(z.enter_points, z.exit_points + 10); });
    else if (inside(x, y, left, 464, 34, 26)) change_selected([](auto& z) { z.enter_after = std::max(std::chrono::milliseconds(0), z.enter_after - std::chrono::milliseconds(50)); });
    else if (inside(x, y, left + 260, 464, 34, 26)) change_selected([](auto& z) { z.enter_after += std::chrono::milliseconds(50); });
    else if (inside(x, y, left, 502, 34, 26)) change_selected([](auto& z) { z.exit_after = std::max(std::chrono::milliseconds(0), z.exit_after - std::chrono::milliseconds(100)); });
    else if (inside(x, y, left + 260, 502, 34, 26)) change_selected([](auto& z) { z.exit_after += std::chrono::milliseconds(100); });
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
      specter::validate_config(config_);
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

  void render() {
    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);
    const int viewport_width = std::max(1, width - panel_width);
    glViewport(0, 0, viewport_width, height);
    glClearColor(0.025F, 0.03F, 0.045F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    if (top_down_) {
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
      glVertex3d(point.x, point.y, top_down_ ? 0.01 : point.z);
    }
    glEnd();
    if (live_validation_) {
      glPointSize(4.0F); glColor3f(1, 0.25F, 0.1F); glBegin(GL_POINTS);
      for (const auto& point : foreground_points_) glVertex3d(point.x, point.y, top_down_ ? 0.03 : point.z);
      glEnd();
    }
    for (std::size_t i = 0; i < config_.zones.size(); ++i) draw_zone(config_.zones[i], static_cast<int>(i) == selected_);
    draw_gizmo();
  }

  void draw_zone(const specter::ZoneConfig& zone, bool selected) {
    const auto& polygon = zone.floor_polygon;
    const float r = selected ? 1.0F : 0.15F, g = selected ? 0.55F : 0.75F, b = selected ? 0.1F : 0.95F;
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    if (!top_down_) {
      glColor4f(r, g, b, 0.12F); glBegin(GL_QUADS);
      for (std::size_t i = 0; i < polygon.size(); ++i) {
        const auto& a = polygon[i]; const auto& c = polygon[(i + 1) % polygon.size()];
        glVertex3d(a.x, a.y, zone.min_height_m); glVertex3d(c.x, c.y, zone.min_height_m);
        glVertex3d(c.x, c.y, zone.max_height_m); glVertex3d(a.x, a.y, zone.max_height_m);
      }
      glEnd();
    }
    glColor4f(r, g, b, 0.95F); glLineWidth(selected ? 4.0F : 2.0F);
    const double low = top_down_ ? 0.05 : zone.min_height_m;
    const double high = top_down_ ? 0.05 : zone.max_height_m;
    glBegin(GL_LINE_LOOP); for (const auto point : polygon) glVertex3d(point.x, point.y, low); glEnd();
    if (!top_down_) {
      glBegin(GL_LINE_LOOP); for (const auto point : polygon) glVertex3d(point.x, point.y, high); glEnd();
      glBegin(GL_LINES); for (const auto point : polygon) { glVertex3d(point.x, point.y, low); glVertex3d(point.x, point.y, high); } glEnd();
    }
    glDisable(GL_DEPTH_TEST);
    if (top_down_ && selected) {
      glPointSize(9); glBegin(GL_POINTS); for (const auto point : polygon) glVertex3d(point.x, point.y, 0.08); glEnd();
    } else if (!top_down_) {
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
    if (selected_ < 0 || selected_vertex_ < 0) return;
    const auto origin = selected_point();
    glDisable(GL_DEPTH_TEST);
    glLineWidth(5.0F);
    glBegin(GL_LINES);
    glColor3f(gizmo_drag_ == GizmoAxis::x ? 1.0F : 0.85F, 0.12F, 0.12F);
    glVertex3d(origin.x, origin.y, origin.z); glVertex3d(origin.x + 0.45, origin.y, origin.z);
    glColor3f(0.12F, gizmo_drag_ == GizmoAxis::y ? 1.0F : 0.85F, 0.15F);
    glVertex3d(origin.x, origin.y, origin.z); glVertex3d(origin.x, origin.y + 0.45, origin.z);
    if (!top_down_) {
      glColor3f(0.12F, 0.4F, gizmo_drag_ == GizmoAxis::z ? 1.0F : 0.95F);
      glVertex3d(origin.x, origin.y, origin.z); glVertex3d(origin.x, origin.y, origin.z + 0.45);
    }
    glEnd();
    glPointSize(11.0F); glBegin(GL_POINTS);
    glColor3f(1, 0.15F, 0.15F); glVertex3d(origin.x + 0.45, origin.y, origin.z);
    glColor3f(0.15F, 1, 0.2F); glVertex3d(origin.x, origin.y + 0.45, origin.z);
    if (!top_down_) { glColor3f(0.15F, 0.45F, 1); glVertex3d(origin.x, origin.y, origin.z + 0.45); }
    glEnd();
    glEnable(GL_DEPTH_TEST);
  }

  void draw_overlay(int width, int height) {
    glViewport(0, 0, width, height);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, width, height, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    if (top_down_) draw_topdown_labels(width, height);
    glColor4f(0.055F, 0.065F, 0.085F, 0.98F);
    glBegin(GL_QUADS); glVertex2d(width - panel_width, 0); glVertex2d(width, 0); glVertex2d(width, height); glVertex2d(width - panel_width, height); glEnd();
    const double x = width - panel_width + 16;
    glColor3f(0.75F, 0.9F, 1.0F); draw_text(x, 16, "SPECTER-SENSE", 2.2);
    glColor3f(0.8F, 0.82F, 0.86F); draw_text(x, 48, status_.substr(0, 42), 1.15);
    draw_button(x, 88, 142, 30, "PERSPECTIVE", !top_down_);
    draw_button(x + 152, 88, 142, 30, "TOP-DOWN", top_down_);
    draw_button(x, 126, 142, 30, frozen_ ? "RESUME LIVE" : "FREEZE FRAME", frozen_);
    draw_button(x + 152, 126, 142, 30, "ESTIMATE FLOOR", false);
    draw_button(x, 164, 142, 30, live_validation_ ? "VALIDATION ON" : "VALIDATION OFF", live_validation_);
    draw_button(x + 152, 164, 68, 30, "UNDO", false);
    draw_button(x + 226, 164, 68, 30, "REDO", false);
    draw_button(x, 202, 68, 30, "PREV", false);
    draw_button(x + 74, 202, 68, 30, "NEXT", false);
    draw_button(x + 152, 202, 68, 30, "RENAME", renaming_);
    draw_button(x + 226, 202, 68, 30, "COPY", false);
    draw_button(x, 238, 294, 30, "DELETE SELECTED ZONE", false, true);
    if (renaming_) {
      glColor3f(1, 0.82F, 0.25F); draw_text(x, 280, "NAME: " + rename_buffer_ + "_", 1.4);
      draw_button(x, 316, 135, 30, "APPLY NAME", false);
      draw_button(x + 145, 316, 135, 30, "CANCEL", false);
    }
    double y = renaming_ ? 360 : 284;
    if (selected_ >= 0 && selected_ < static_cast<int>(config_.zones.size())) {
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
    draw_text(x, height - 128, "CTRL+N  NEW BOUNDING BOX", 1.35);
    draw_text(x, height - 106, "CLICK CORNER, DRAG X/Y/Z", 1.25);
    draw_text(x, height - 86, "LEFT ORBIT  RIGHT PAN  WHEEL ZOOM", 1.05);
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
    draw_text(125, 235, original_.camera_to_room.matrix == config_.camera_to_room.matrix
        ? "ROOM TRANSFORM: UNCHANGED" : "ROOM TRANSFORM: CHANGED", 1.5);
    draw_text(125, 265, specter::serialize_config(original_) == specter::serialize_config(config_)
        ? "CONTENT: NO CHANGES" : "CONTENT: MODIFIED", 1.5);
    draw_button(125, height - 175, 360, 35, "ATOMICALLY REPLACE CONFIG", true);
    draw_button(510, height - 175, 180, 35, "CANCEL", false, true);
  }

  Options options_;
  specter::AppConfig config_;
  specter::AppConfig original_;
  std::unique_ptr<specter::OccupancyPipeline> pipeline_;
  std::unique_ptr<specter::FrameSource> source_;
  GLFWwindow* window_{};
  std::optional<specter::DepthFrame> frame_;
  std::vector<Vec3> camera_points_;
  std::vector<Vec3> room_points_;
  std::vector<specter::Point3> foreground_points_;
  std::vector<specter::ZoneState> zone_states_;
  std::vector<specter::AppConfig> undo_, redo_;
  specter::AppConfig edit_before_;
  std::string status_{"STARTING"};
  std::string rename_buffer_;
  bool frozen_{}, top_down_{}, left_down_{}, right_down_{}, translating_{}, corner_click_{}, renaming_{}, save_preview_{};
  bool live_validation_{};
  int selected_{-1}, selected_vertex_{-1};
  bool selected_top_{};
  GizmoAxis gizmo_drag_{GizmoAxis::none};
  double mouse_x_{}, mouse_y_{};
  double orbit_yaw_{0.6}, orbit_pitch_{0.45}, orbit_distance_{4.0};
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
