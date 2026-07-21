#include "specter_sense/socket_publisher.hpp"

#include "specter_sense/state_writer.hpp"

#include <boost/json.hpp>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <deque>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace specter {
namespace {

constexpr std::size_t max_pending_bytes = 1024 * 1024;

std::runtime_error system_error(const std::string& action) {
  return std::runtime_error(action + ": " + std::strerror(errno));
}

sockaddr_un address_for(const std::filesystem::path& path) {
  const auto text = path.string();
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  if (text.size() >= sizeof(address.sun_path)) throw std::runtime_error("Unix socket path is too long: " + text);
  std::memcpy(address.sun_path, text.c_str(), text.size() + 1);
  return address;
}

bool same_health(const SensorState& left, const SensorState& right) {
  return left.connected == right.connected && left.reconnecting == right.reconnecting && left.status == right.status;
}

bool same_occupancy(const Snapshot& left, const Snapshot& right) {
  if (left.sensors.size() != right.sensors.size()) return false;
  for (std::size_t sensor = 0; sensor < left.sensors.size(); ++sensor) {
    if (left.sensors[sensor].first != right.sensors[sensor].first) return false;
    const auto& left_zones = left.sensors[sensor].second.zones;
    const auto& right_zones = right.sensors[sensor].second.zones;
    if (left_zones.size() != right_zones.size()) return false;
    for (std::size_t i = 0; i < left_zones.size(); ++i)
      if (left_zones[i].name != right_zones[i].name || left_zones[i].occupied != right_zones[i].occupied) return false;
  }
  return true;
}

bool same_health(const Snapshot& left, const Snapshot& right) {
  if (left.sensors.size() != right.sensors.size()) return false;
  for (std::size_t i = 0; i < left.sensors.size(); ++i)
    if (left.sensors[i].first != right.sensors[i].first ||
        !same_health(left.sensors[i].second.sensor, right.sensors[i].second.sensor)) return false;
  return true;
}

std::string envelope(const char* type, std::uint64_t sequence, const Snapshot& snapshot, const char* reason = nullptr) {
  boost::json::object message{
      {"type", type},
      {"sequence", sequence},
      {"state", snapshot_to_json(snapshot)}};
  if (reason) message["reason"] = reason;
  return boost::json::serialize(message) + '\n';
}

}  // namespace

struct SocketPublisher::Impl {
  struct Client {
    int fd{-1};
    std::string pending;
    std::size_t offset{};
  };

  explicit Impl(std::filesystem::path requested_path, std::chrono::milliseconds requested_interval)
      : socket_path(std::move(requested_path)), observation_interval(requested_interval) {
    if (observation_interval.count() <= 0) throw std::runtime_error("socket observation interval must be positive");
    if (socket_path.has_parent_path()) std::filesystem::create_directories(socket_path.parent_path());
    listener = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (listener < 0) throw system_error("create Unix socket");
    bool bound_here = false;
    try {
      const auto address = address_for(socket_path);
      if (::bind(listener, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0) {
        if (errno != EADDRINUSE) throw system_error("bind Unix socket " + socket_path.string());
        const int probe = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
        if (probe < 0) throw system_error("probe existing Unix socket");
        const int connected = ::connect(probe, reinterpret_cast<const sockaddr*>(&address), sizeof(address));
        const int connect_error = errno;
        ::close(probe);
        if (connected == 0) throw std::runtime_error("another specter-sense publisher is listening at " + socket_path.string());
        struct stat existing{};
        if (::lstat(socket_path.c_str(), &existing) < 0 || !S_ISSOCK(existing.st_mode) ||
            (connect_error != ECONNREFUSED && connect_error != ENOENT))
          throw std::runtime_error("refusing to replace non-stale socket path: " + socket_path.string());
        if (::unlink(socket_path.c_str()) < 0) throw system_error("remove stale Unix socket");
        if (::bind(listener, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0)
          throw system_error("bind Unix socket after stale cleanup");
      }
      bound_here = true;
      if (::chmod(socket_path.c_str(), S_IRUSR | S_IWUSR) < 0) throw system_error("set Unix socket permissions");
      if (::listen(listener, 16) < 0) throw system_error("listen on Unix socket");
      struct stat created{};
      if (::lstat(socket_path.c_str(), &created) < 0) throw system_error("inspect created Unix socket");
      if (!S_ISSOCK(created.st_mode)) throw std::runtime_error("created path is not a Unix socket: " + socket_path.string());
      socket_device = created.st_dev;
      socket_inode = created.st_ino;
    } catch (...) {
      ::close(listener);
      listener = -1;
      if (bound_here) ::unlink(socket_path.c_str());
      throw;
    }
  }

  ~Impl() {
    for (auto& client : clients) if (client.fd >= 0) ::close(client.fd);
    if (listener >= 0) ::close(listener);
    struct stat current{};
    if (::lstat(socket_path.c_str(), &current) == 0 && S_ISSOCK(current.st_mode) &&
        current.st_dev == socket_device && current.st_ino == socket_inode)
      ::unlink(socket_path.c_str());
  }

  void queue(Client& client, const std::string& message) {
    if (client.offset > 0) {
      client.pending.erase(0, client.offset);
      client.offset = 0;
    }
    client.pending += message;
    if (client.pending.size() > max_pending_bytes) disconnect(client);
  }

  static void disconnect(Client& client) {
    if (client.fd >= 0) ::close(client.fd);
    client.fd = -1;
    client.pending.clear();
    client.offset = 0;
  }

  void accept_clients() {
    while (true) {
      const int fd = ::accept4(listener, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
      if (fd < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return;
        if (errno == EINTR) continue;
        throw system_error("accept Unix socket client");
      }
      clients.push_back({fd, {}, 0});
      if (latest) queue(clients.back(), envelope("snapshot", sequence, *latest));
    }
  }

  void broadcast(const std::string& message) {
    for (auto& client : clients) if (client.fd >= 0) queue(client, message);
  }

  void flush() {
    for (auto& client : clients) {
      while (client.fd >= 0 && client.offset < client.pending.size()) {
        const auto* data = client.pending.data() + client.offset;
        const auto remaining = client.pending.size() - client.offset;
        const auto sent = ::send(client.fd, data, remaining, MSG_NOSIGNAL);
        if (sent > 0) client.offset += static_cast<std::size_t>(sent);
        else if (sent < 0 && errno == EINTR) continue;
        else if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
        else { disconnect(client); break; }
      }
      if (client.fd >= 0 && client.offset == client.pending.size()) {
        client.pending.clear();
        client.offset = 0;
      }
    }
    std::erase_if(clients, [](const Client& client) { return client.fd < 0; });
  }

  void publish(const Snapshot& snapshot) {
    const auto now = std::chrono::steady_clock::now();
    const bool health_changed = !latest || !same_health(*latest, snapshot);
    const bool occupancy_changed = !latest || !same_occupancy(*latest, snapshot);
    const bool observation_due = !last_broadcast || now - *last_broadcast >= observation_interval;
    accept_clients();
    if (health_changed || occupancy_changed || observation_due) {
      latest = snapshot;
      ++sequence;
      const char* reason = health_changed ? "health_changed" : occupancy_changed ? "occupancy_changed" : "observation";
      broadcast(envelope("state", sequence, snapshot, reason));
      last_broadcast = now;
    }
    flush();
  }

  std::filesystem::path socket_path;
  std::chrono::milliseconds observation_interval;
  int listener{-1};
  dev_t socket_device{};
  ino_t socket_inode{};
  std::deque<Client> clients;
  std::optional<Snapshot> latest;
  std::optional<std::chrono::steady_clock::time_point> last_broadcast;
  std::uint64_t sequence{};
};

SocketPublisher::SocketPublisher(std::filesystem::path path, std::chrono::milliseconds interval)
    : impl_(std::make_unique<Impl>(std::move(path), interval)) {}
SocketPublisher::~SocketPublisher() = default;
SocketPublisher::SocketPublisher(SocketPublisher&&) noexcept = default;
SocketPublisher& SocketPublisher::operator=(SocketPublisher&&) noexcept = default;
void SocketPublisher::publish(const Snapshot& snapshot) { impl_->publish(snapshot); }
const std::filesystem::path& SocketPublisher::path() const { return impl_->socket_path; }
std::size_t SocketPublisher::client_count() const { return impl_->clients.size(); }

}  // namespace specter
