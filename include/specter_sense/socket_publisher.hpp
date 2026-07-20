#pragma once

#include "specter_sense/types.hpp"

#include <chrono>
#include <filesystem>
#include <memory>

namespace specter {

class SocketPublisher {
 public:
  explicit SocketPublisher(
      std::filesystem::path path,
      std::chrono::milliseconds observation_interval = std::chrono::milliseconds(100));
  ~SocketPublisher();

  SocketPublisher(const SocketPublisher&) = delete;
  SocketPublisher& operator=(const SocketPublisher&) = delete;
  SocketPublisher(SocketPublisher&&) noexcept;
  SocketPublisher& operator=(SocketPublisher&&) noexcept;

  void publish(const Snapshot& snapshot);
  const std::filesystem::path& path() const;
  std::size_t client_count() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace specter
